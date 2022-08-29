/*
 * CanFD2040.cpp
 *
 *  Created on: 22 Aug 2022
 *      Author: David
 */

#include "CanFD2040.h"

#include "hardware/regs/dreq.h" // DREQ_PIO0_RX1
#include "hardware/structs/dma.h" // dma_hw
#include "hardware/structs/iobank0.h" // iobank0_hw
#include "hardware/structs/padsbank0.h" // padsbank0_hw
#include "hardware/structs/resets.h" // RESETS_RESET_PIO0_BITS
#include <RP2040.h>

#include <cstring>

extern "C" void PIO_isr() noexcept;

/****************************************************************
 * rp2040 and low-level helper functions
 ****************************************************************/

// Helper compiler definitions
#define barrier() __asm__ __volatile__("": : :"memory")
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

// rp2040 helper function to clear a hardware reset bit
static void rp2040_clear_reset(uint32_t reset_bit)
{
    if (resets_hw->reset & reset_bit)
    {
        hw_clear_bits(&resets_hw->reset, reset_bit);
        while (!(resets_hw->reset_done & reset_bit)) { }
    }
}

// Helper to set the mode and extended function of a pin
//TODO use SetPinFunction instead
static void rp2040_gpio_peripheral(uint32_t gpio, int func, int pull_up)
{
    padsbank0_hw->io[gpio] = (
        PADS_BANK0_GPIO0_IE_BITS
        | (PADS_BANK0_GPIO0_DRIVE_VALUE_4MA << PADS_BANK0_GPIO0_DRIVE_MSB)
        | (pull_up > 0 ? PADS_BANK0_GPIO0_PUE_BITS : 0)
        | (pull_up < 0 ? PADS_BANK0_GPIO0_PDE_BITS : 0));
    iobank0_hw->io[gpio].ctrl = func << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
}

/****************************************************************
 * rp2040 PIO support
 ****************************************************************/

#define PIO_CLOCK_PER_BIT 32

#define can2040_offset_sync_found_end_of_message 2u
#define can2040_offset_sync_signal_start 4u
#define can2040_offset_sync_entry 6u
#define can2040_offset_sync_end 13u
#define can2040_offset_shared_rx_read 13u
#define can2040_offset_shared_rx_end 15u
#define can2040_offset_match_load_next 18u
#define can2040_offset_match_end 25u
#define can2040_offset_tx_got_recessive 25u
#define can2040_offset_tx_start 26u
#define can2040_offset_tx_conflict 31u

static const uint16_t can2040_program_instructions[] = {
    0x0085, //  0: jmp    y--, 5
    0x0048, //  1: jmp    x--, 8
    0xe13a, //  2: set    x, 26                  [1]
    0x00cc, //  3: jmp    pin, 12
    0xc000, //  4: irq    nowait 0
    0x00c0, //  5: jmp    pin, 0
    0xc040, //  6: irq    clear 0
    0xe228, //  7: set    x, 8                   [2]
    0xf242, //  8: set    y, 2                   [18]
    0xc104, //  9: irq    nowait 4               [1]
    0x03c5, // 10: jmp    pin, 5                 [3]
    0x0307, // 11: jmp    7                      [3]
    0x0043, // 12: jmp    x--, 3
    0x20c4, // 13: wait   1 irq, 4
    0x4001, // 14: in     pins, 1
    0xa046, // 15: mov    y, isr
    0x00b2, // 16: jmp    x != y, 18
    0xc002, // 17: irq    nowait 2
    0x40eb, // 18: in     osr, 11
    0x4054, // 19: in     y, 20
    0xa047, // 20: mov    y, osr
    0x8080, // 21: pull   noblock
    0xa027, // 22: mov    x, osr
    0x0098, // 23: jmp    y--, 24
    0xa0e2, // 24: mov    osr, y
    0xa242, // 25: nop                           [2]
    0x6021, // 26: out    x, 1
    0xa001, // 27: mov    pins, x
    0x20c4, // 28: wait   1 irq, 4
    0x00d9, // 29: jmp    pin, 25
    0x023a, // 30: jmp    !x, 26                 [2]
    0xc027, // 31: irq    wait 7
};

/****************************************************************
 * CRC calculation
 ****************************************************************/

// Calculated 8-bit crc table (see scripts/crc.py)
static constexpr uint16_t crc_table[256] =
{
    0x0000,0x4599,0x4eab,0x0b32,0x58cf,0x1d56,0x1664,0x53fd,0x7407,0x319e,
    0x3aac,0x7f35,0x2cc8,0x6951,0x6263,0x27fa,0x2d97,0x680e,0x633c,0x26a5,
    0x7558,0x30c1,0x3bf3,0x7e6a,0x5990,0x1c09,0x173b,0x52a2,0x015f,0x44c6,
    0x4ff4,0x0a6d,0x5b2e,0x1eb7,0x1585,0x501c,0x03e1,0x4678,0x4d4a,0x08d3,
    0x2f29,0x6ab0,0x6182,0x241b,0x77e6,0x327f,0x394d,0x7cd4,0x76b9,0x3320,
    0x3812,0x7d8b,0x2e76,0x6bef,0x60dd,0x2544,0x02be,0x4727,0x4c15,0x098c,
    0x5a71,0x1fe8,0x14da,0x5143,0x73c5,0x365c,0x3d6e,0x78f7,0x2b0a,0x6e93,
    0x65a1,0x2038,0x07c2,0x425b,0x4969,0x0cf0,0x5f0d,0x1a94,0x11a6,0x543f,
    0x5e52,0x1bcb,0x10f9,0x5560,0x069d,0x4304,0x4836,0x0daf,0x2a55,0x6fcc,
    0x64fe,0x2167,0x729a,0x3703,0x3c31,0x79a8,0x28eb,0x6d72,0x6640,0x23d9,
    0x7024,0x35bd,0x3e8f,0x7b16,0x5cec,0x1975,0x1247,0x57de,0x0423,0x41ba,
    0x4a88,0x0f11,0x057c,0x40e5,0x4bd7,0x0e4e,0x5db3,0x182a,0x1318,0x5681,
    0x717b,0x34e2,0x3fd0,0x7a49,0x29b4,0x6c2d,0x671f,0x2286,0x2213,0x678a,
    0x6cb8,0x2921,0x7adc,0x3f45,0x3477,0x71ee,0x5614,0x138d,0x18bf,0x5d26,
    0x0edb,0x4b42,0x4070,0x05e9,0x0f84,0x4a1d,0x412f,0x04b6,0x574b,0x12d2,
    0x19e0,0x5c79,0x7b83,0x3e1a,0x3528,0x70b1,0x234c,0x66d5,0x6de7,0x287e,
    0x793d,0x3ca4,0x3796,0x720f,0x21f2,0x646b,0x6f59,0x2ac0,0x0d3a,0x48a3,
    0x4391,0x0608,0x55f5,0x106c,0x1b5e,0x5ec7,0x54aa,0x1133,0x1a01,0x5f98,
    0x0c65,0x49fc,0x42ce,0x0757,0x20ad,0x6534,0x6e06,0x2b9f,0x7862,0x3dfb,
    0x36c9,0x7350,0x51d6,0x144f,0x1f7d,0x5ae4,0x0919,0x4c80,0x47b2,0x022b,
    0x25d1,0x6048,0x6b7a,0x2ee3,0x7d1e,0x3887,0x33b5,0x762c,0x7c41,0x39d8,
    0x32ea,0x7773,0x248e,0x6117,0x6a25,0x2fbc,0x0846,0x4ddf,0x46ed,0x0374,
    0x5089,0x1510,0x1e22,0x5bbb,0x0af8,0x4f61,0x4453,0x01ca,0x5237,0x17ae,
    0x1c9c,0x5905,0x7eff,0x3b66,0x3054,0x75cd,0x2630,0x63a9,0x689b,0x2d02,
    0x276f,0x62f6,0x69c4,0x2c5d,0x7fa0,0x3a39,0x310b,0x7492,0x5368,0x16f1,
    0x1dc3,0x585a,0x0ba7,0x4e3e,0x450c,0x0095
};

// Update a crc with 8 bits of data
static uint32_t crc_byte(uint32_t crc, uint32_t data)
{
    return (crc << 8) ^ crc_table[((crc >> 7) ^ data) & 0xff];
}

// Update a crc with 8, 16, 24, or 32 bits of data
static inline uint32_t crc_bytes(uint32_t crc, uint32_t data, uint32_t num)
{
    switch (num) {
    default: crc = crc_byte(crc, data >> 24);
    // no break
    case 3:  crc = crc_byte(crc, data >> 16);
    // no break
    case 2:  crc = crc_byte(crc, data >> 8);
    // no break
    case 1:  crc = crc_byte(crc, data);
    }
    return crc;
}

// Calculate CRC17 on a number of bits
uint32_t Crc17(uint32_t initialValue, const uint32_t *buf, unsigned int numBits) noexcept
{
	//TODO
	return initialValue;		//temporary!
}

// Calculate CRC21 on a number of bits
uint32_t Crc21(uint32_t initialValue, const uint32_t *buf, unsigned int numBits) noexcept
{
	//TODO
	return initialValue;		//temporary!
}

/****************************************************************
 * Bit unstuffing
 ****************************************************************/

// Add 'count' number of bits from 'data' to the unstuffer
// The bits are shifted left into stuffed_bits, and count_stuff is set to the number of bits added
void BitUnstuffer::AddBits(uint32_t data, uint32_t count) noexcept
{
    const uint32_t mask = (1u << count) - 1;
    stuffed_bits = (stuffed_bits << count) | (data & mask);
    count_stuff = count;
}

// Reset state and set the next desired 'count' unstuffed bits to extract
void BitUnstuffer::SetCount(uint32_t count) noexcept
{
    unstuffed_bits = 0;
    count_unstuff = count;
}

// Clear bitstuffing state (used after crc field to avoid bitstuffing ack field)
void BitUnstuffer::ClearState() noexcept
{
    const uint32_t lb = 1u << count_stuff;
    stuffed_bits = (stuffed_bits & (lb - 1)) | lb;
}

// Pull bits from unstuffer (as specified in unstuf_set_count())
// Return 0 if successful, 1 if need more data, -1 or -2 or -3 if there was an unstuffing error
int BitUnstuffer::PullBits() noexcept
{
    const uint32_t sb = stuffed_bits;
    if (usingFixedStuffBits)
    {
    	while (count_unstuff != 0 && count_stuff != 0)
    	{
    		if (bitsUntilFixedStuffBit == 0)
    		{
    			// Check that this is the inverse of the previous bit
    			if (((unstuffed_bits ^ (unstuffed_bits >> 1)) & (1u << (count_stuff - 1))) == 0)
    			{
    				return -3;
    			}
    			--count_stuff;
    		}
    		else
    		{
    			const uint32_t toExtract = min<uint32_t>(count_unstuff, min<uint32_t>(count_stuff, bitsUntilFixedStuffBit));
    			unstuffed_bits = (unstuffed_bits << toExtract)
    							| ((stuffed_bits >> (count_stuff - toExtract)) & ((1u << toExtract) - 1));
    			count_stuff -= toExtract;
    			count_unstuff -= toExtract;
     		}
    	}
    	return unstuffed_bits;
    }

    // Else using variable stuff bits
    const uint32_t edges = sb ^ (sb >> 1);
	const uint32_t e2 = edges | (edges >> 1);
	const uint32_t e4 = e2 | (e2 >> 2);
	const uint32_t rm_bits = ~e4;
	uint32_t cs = count_stuff;
	uint32_t cu = count_unstuff;
	if (cs == 0)
	{
		// Need more data
		return 1;
	}
	while (true)
	{
		uint32_t try_cnt = cs > cu ? cu : cs;
		for (;;)
		{
			const uint32_t try_mask = ((1 << try_cnt) - 1) << (cs + 1 - try_cnt);
			if (likely((rm_bits & try_mask) == 0))
			{
				// No stuff bits in try_cnt bits - copy into unstuffed_bits
				count_unstuff = cu = cu - try_cnt;
				count_stuff = cs = cs - try_cnt;
				unstuffed_bits |= ((sb >> cs) & ((1u << try_cnt) - 1)) << cu;
				if (cu == 0)
				{
					// Extracted desired bits
					return 0;
				}
				break;
			}
			count_stuff = cs = cs - 1;
			if (rm_bits & (1u << (cs + 1)))
			{
				// High bit of try_cnt a stuff bit
				if (unlikely(rm_bits & (1u << cs)))
				{
					// Six consecutive bits - a bitstuff error
					return ((sb >> cs) & 1u) ? -1 : -2;
				}
				break;
			}
			// High bit not a stuff bit - limit try_cnt and retry
			count_unstuff = cu = cu - 1;
			unstuffed_bits |= ((sb >> cs) & 1) << cu;
			try_cnt /= 2;
		}
		if (likely(cs == 0))
		{
			// Need more data
			return 1;
		}
	}
}

// Signal the bit stuffer to expect a stuff bit next and another after every 4 bits subsequently
void BitUnstuffer::UseFixedStuffBits() noexcept
{
	usingFixedStuffBits = true;
	bitsUntilFixedStuffBit = 0;
}

/****************************************************************
 * Bit stuffing
 ****************************************************************/

// State storage for building bit stuffed transmit messages
class BitStuffer
{
public:
	BitStuffer(uint32_t *p_buf) noexcept : prev_stuffed(1), bitpos(0), totalStuffBits(0), buf(p_buf) { }
	void pushraw(uint32_t data, uint32_t count) noexcept;
	void push(uint32_t data, uint32_t count) noexcept;
	void AddFixedStuffBit() noexcept;
	uint32_t finalize() noexcept;
	uint32_t stuff(uint32_t& pb, uint32_t num_bits) noexcept;
	uint32_t GetTotalStuffBits() const noexcept { return totalStuffBits; }
	uint32_t GetTotalBits() const noexcept { return bitpos; }

private:
    uint32_t prev_stuffed;		// the previous 5 (or more) bits we have stored in the buffer, so we can tell whether stuffing is needed when we store more bits
    uint32_t bitpos;			// how many bits we have stored in the buffer already
    uint32_t totalStuffBits;
    uint32_t *buf;				// the buffer we are stuffing into
};

// Push 'count' bits of 'data' into stuffer without performing bit stuffing
void BitStuffer::pushraw(uint32_t data, uint32_t count) noexcept
pre(count <= 32)
{
    const uint32_t wp = bitpos / 32;					// current index into the buffer
    const uint32_t bitavail = 32 - (bitpos % 32);		// number of bits available in the word at the current index, 1 to 32
    if (bitavail == 32)
    {
    	buf[wp] = data << (32 - count);
    }
    else if (bitavail >= count)
    {
    	buf[wp] |= data << (bitavail - count);
    }
    else
    {
    	buf[wp] |= data >> (count - bitavail);
    	buf[wp + 1] = data << (32 - (count - bitavail));
    }
    bitpos += count;
}

// Push 'count' (max 26) bits of 'data' into stuffer
void BitStuffer::push(uint32_t data, uint32_t count) noexcept
pre(count <= 26)										// the stuffer may expand 26 bits to 32
{
    data &= (1 << count) - 1;							// clear the high bits
    uint32_t stuf = (prev_stuffed << count) | data;
    const uint32_t newcount = stuff(stuf, count);
    pushraw(stuf, newcount);
    prev_stuffed = stuf;
}

// Pad final word of stuffer with high bits
uint32_t BitStuffer::finalize() noexcept
{
    const uint32_t words = DIV_ROUND_UP(bitpos, 32);
    const uint32_t extra = words * 32 - bitpos;
    if (extra)
    {
        buf[words - 1] |= (1 << extra) - 1;
    }
    return words;
}

// Add a fixed stuff bit, not included in the count of total stuff bits
void BitStuffer::AddFixedStuffBit() noexcept
{
	prev_stuffed = (prev_stuffed << 1) | ((prev_stuffed & 1u) ^ 1u);
	pushraw(prev_stuffed & 1u, 1);
}

// Stuff 'num_bits' (max 26) bits in 'pb' - upper bits must already be stuffed. Return the count of bits generated.
uint32_t BitStuffer::stuff(uint32_t& pb, uint32_t num_bits) noexcept
pre(count <= 26)
{
    uint32_t b = pb;
    uint32_t count = num_bits;
    for (;;)
    {
        uint32_t try_cnt = num_bits;
        const uint32_t edges = b ^ (b >> 1);
        const uint32_t e2 = edges | (edges >> 1), e4 = e2 | (e2 >> 2);
        const uint32_t add_bits = ~e4;
        for (;;)
        {
            const uint32_t try_mask = ((1 << try_cnt) - 1) << (num_bits - try_cnt);
            if (!(add_bits & try_mask))
            {
                // No stuff bits needed in try_cnt bits
                if (try_cnt >= num_bits) { goto done; }
                num_bits -= try_cnt;
                try_cnt = (num_bits + 1) / 2;
            }
            else
            {
				if (add_bits & (1 << (num_bits - 1)))
				{
					// A stuff bit must be inserted prior to the high bit
					const uint32_t low_mask = (1 << num_bits) - 1, low = b & low_mask;
					const uint32_t high = (b & ~(low_mask >> 1)) << 1;
					b = high ^ low ^ (1 << (num_bits - 1));
					++count;
					++totalStuffBits;
					if (num_bits <= 4) { goto done; }
					num_bits -= 4;
					break;
				}
				// High bit doesn't need stuff bit - accept it, limit try_cnt, retry
				num_bits--;
				try_cnt /= 2;
            }
        }
    }
done:
    pb = b;
    return count;
}

// CanFD2040 members
// Start CAN-FD running
void CanFD2040::Entry(VirtualCanRegisters *p_regs) noexcept
{
	regs = p_regs;
    for (;;)
    {
    	while (!regs->canEnabled) { }

    	pio_setup();										// Set up the PIO and pins
        data_state_go_discard();

    	while (regs->canEnabled) { }

    	// Disable CAN - set output to recessive
    	//TODO
    }
}

// Set up a message ready to be transmitted
void CanFD2040::PopulateTransmitBuffer() noexcept
{
	const uint32_t* txBufferPtr = const_cast<const uint32_t*>(regs->txFifoAddr);
	const CanTxBufferHeader *const txHdr = reinterpret_cast<const CanTxBufferHeader *>(reinterpret_cast<const uint8_t*>(txBufferPtr) + (sizeof(CanTxBufferHeader) + 64) * regs->txFifoGetIndex);

	txId = txHdr->T0.bit.ID;
	txDlc = txHdr->T1.bit.DLC;

	// Push the header through the stuffer
	BitStuffer bs(txMessage);
	if (txHdr->T0.bit.XTD)
	{
		// Extended header
		const uint32_t h1 = ((txId & 0x1ffc0000) >> 16) | 0x03;
		const uint32_t h2 = ((txId & 0x3ffff) << 9) | 0x80 | txDlc;
		bs.push(h1, 19);
		bs.push(h2, 20);
	}
	else
	{
		// Standard header
		uint32_t hdr = ((txId & 0x7ff) << 10) | 0x80 | txDlc;
		bs.push(hdr, 21);
	}

	// Push the data
	const uint8_t *data8 = reinterpret_cast<const uint8_t*>(txHdr) + sizeof(txHdr);
	if (txDlc < 8)
	{
		for (size_t i = 0; i < txDlc; i++)
		{
			bs.push(data8[i], 8);
		}
	}
	else
	{
		const uint16_t *data16 = reinterpret_cast<const uint16_t*>(data8);
		if (txDlc <= 12)
		{
			for (size_t i = 0; i < 2 * (txDlc - 6); i++)
			{
				bs.push(data16[i], 16);
			}
		}
		else
		{
			for (size_t i = 0; i < 8 * (txDlc - 11); i++)
			{
				bs.push(data16[i], 16);
			}
		}
	}

	// Push the stuff count
	// 3-bit Gray code table, shifted left 1 bit with lower parity bit added
	constexpr uint8_t GrayTable[] = { 0b0000, 0b0011, 0b0110, 0b0101, 0b1100, 0b1111, 0b1010, 0b1001 };
	bs.AddFixedStuffBit();
	bs.push(GrayTable[bs.GetTotalStuffBits() & 7], 4);

	// CRC everything stored so far and store the CRC
	if (txDlc > 10)
	{
		txCrc = Crc21(1u << 20, txMessage, bs.GetTotalBits());
		bs.AddFixedStuffBit();
		bs.pushraw((txCrc >> 17) & 0x0f, 4);
	}
	else
	{
		txCrc = Crc17(1u << 16, txMessage, bs.GetTotalBits());
	}

	bs.AddFixedStuffBit();
	bs.pushraw((txCrc >> 13) & 0x0f, 4);
	bs.AddFixedStuffBit();
	bs.pushraw((txCrc >> 9) & 0x0f, 4);
	bs.AddFixedStuffBit();
	bs.pushraw((txCrc >> 5) & 0x0f, 4);
	bs.AddFixedStuffBit();
	bs.pushraw((txCrc >> 1) & 0x0f, 4);
	bs.AddFixedStuffBit();
	bs.pushraw(txCrc & 0x01, 1);

	txStuffedWords = bs.finalize();
}

void CanFD2040::pio_setup() noexcept
{
	constexpr int PIO_FUNC = 6;

    // Configure pio0 clock
    const uint32_t rb = (regs->pioNumber) ? RESETS_RESET_PIO1_BITS : RESETS_RESET_PIO0_BITS;
    rp2040_clear_reset(rb);

    // Setup and sync pio state machine clocks
    const uint32_t div = (256 / PIO_CLOCK_PER_BIT) * SystemCoreClockFreq / regs->bitrate;
    for (unsigned int i = 0; i < 4; i++)
    {
    	pio_hw->sm[i].clkdiv = div << PIO_SM0_CLKDIV_FRAC_LSB;
    }

    // Configure state machines
    pio_sm_setup();

    // Map Rx/Tx gpios
    rp2040_gpio_peripheral(regs->rxPin, PIO_FUNC, 1);
    rp2040_gpio_peripheral(regs->txPin, PIO_FUNC, 0);

    irq_set_exclusive_handler ((unsigned int)((regs->pioNumber) ? PIO1_IRQ_0_IRQn : PIO0_IRQ_0_IRQn), PIO_isr);
}

// Setup PIO "sync" state machine (state machine 0)
void CanFD2040::pio_sync_setup() noexcept
{
    struct pio_sm_hw *sm = &pio_hw->sm[0];
    sm->execctrl = (
        regs->rxPin << PIO_SM0_EXECCTRL_JMP_PIN_LSB
        | (can2040_offset_sync_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_sync_signal_start << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->pinctrl = (
        1 << PIO_SM0_PINCTRL_SET_COUNT_LSB
        | regs->rxPin << PIO_SM0_PINCTRL_SET_BASE_LSB);
    sm->instr = 0xe080; // set pindirs, 0
    sm->pinctrl = 0;
    pio_hw->txf[0] = PIO_CLOCK_PER_BIT / 2 * 8 - 5 - 1;
    sm->instr = 0x80a0; // pull block
    sm->instr = can2040_offset_sync_entry; // jmp sync_entry
}

// Setup PIO "rx" state machine (state machine 1)
void CanFD2040::pio_rx_setup() noexcept
{
    struct pio_sm_hw *sm = &pio_hw->sm[1];
    sm->execctrl = (
        (can2040_offset_shared_rx_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->pinctrl = regs->rxPin << PIO_SM0_PINCTRL_IN_BASE_LSB;
    sm->shiftctrl = 0; // flush fifo on a restart
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS
                     | 8 << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB
                     | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    sm->instr = can2040_offset_shared_rx_read; // jmp shared_rx_read
}

// Setup PIO "match" state machine (state machine 2)
void CanFD2040::pio_match_setup() noexcept
{
    struct pio_sm_hw *sm = &pio_hw->sm[2];
    sm->execctrl = (
          (can2040_offset_match_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->pinctrl = regs->rxPin << PIO_SM0_PINCTRL_IN_BASE_LSB;
    sm->shiftctrl = 0;
    sm->instr = 0xe040; // set y, 0
    sm->instr = 0xa0e2; // mov osr, y
    sm->instr = 0xa02a, // mov x, !y
    sm->instr = can2040_offset_match_load_next; // jmp match_load_next
}

// Setup PIO "tx" state machine (state machine 3)
void CanFD2040::pio_tx_setup() noexcept
{
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->execctrl = regs->rxPin << PIO_SM0_EXECCTRL_JMP_PIN_LSB;
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS
                     | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS);
    sm->pinctrl = (1 << PIO_SM0_PINCTRL_SET_COUNT_LSB
                   | 1 << PIO_SM0_PINCTRL_OUT_COUNT_LSB
                   | regs->txPin << PIO_SM0_PINCTRL_SET_BASE_LSB
                   | regs->txPin << PIO_SM0_PINCTRL_OUT_BASE_LSB);
    sm->instr = 0xe001; // set pins, 1
    sm->instr = 0xe081; // set pindirs, 1
}

// Set PIO "sync" machine to signal "may transmit" (sm irq 0) on 11 idle bits
void CanFD2040::pio_sync_normal_start_signal() noexcept
{
    uint32_t eom_idx = can2040_offset_sync_found_end_of_message;
    pio_hw->instr_mem[eom_idx] = 0xe13a; // set x, 26 [1]
}

// Set PIO "sync" machine to signal "may transmit" (sm irq 0) on 17 idle bits
void CanFD2040::pio_sync_slow_start_signal() noexcept
{
    uint32_t eom_idx = can2040_offset_sync_found_end_of_message;
    pio_hw->instr_mem[eom_idx] = 0xa127; // mov x, osr [1]
}

// Test if PIO "rx" state machine has overflowed its fifos
int CanFD2040::pio_rx_check_stall() noexcept
{
    return pio_hw->fdebug & (1 << (PIO_FDEBUG_RXSTALL_LSB + 1));
}

// Report number of bytes still pending in PIO "rx" fifo queue
int CanFD2040::pio_rx_fifo_level() noexcept
{
    return (pio_hw->flevel & PIO_FLEVEL_RX1_BITS) >> PIO_FLEVEL_RX1_LSB;
}

// Set PIO "match" state machine to raise a "matched" signal on a bit sequence
void CanFD2040::pio_match_check(uint32_t match_key) noexcept
{
    pio_hw->txf[2] = match_key;
}

// Calculate pos+bits identifier for PIO "match" state machine
/*static*/ uint32_t CanFD2040::pio_match_calc_key(uint32_t raw_bits, uint32_t rx_bit_pos) noexcept
{
    return (raw_bits & 0x1fffff) | ((-rx_bit_pos) << 21);
}

// Cancel any pending checks on PIO "match" state machine
void CanFD2040::pio_match_clear() noexcept
{
    pio_match_check(0);
}

// Flush and halt PIO "tx" state machine
void CanFD2040::pio_tx_reset() noexcept
{
    pio_hw->ctrl = ((0x07 << PIO_CTRL_SM_ENABLE_LSB)
                    | (0x08 << PIO_CTRL_SM_RESTART_LSB));
    pio_hw->irq = (1 << 2) | (1<< 3); // clear "matched" and "ack done" signals
    // Clear tx fifo
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->shiftctrl = 0;
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS
                     | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS);
    // Must reset again after clearing fifo
    pio_hw->ctrl = ((0x07 << PIO_CTRL_SM_ENABLE_LSB)
                    | (0x08 << PIO_CTRL_SM_RESTART_LSB));
}

// Queue a message for transmission on PIO "tx" state machine
void CanFD2040::pio_tx_send(uint32_t *data, uint32_t count) noexcept
{
    pio_tx_reset();
    pio_hw->instr_mem[can2040_offset_tx_got_recessive] = 0xa242; // nop [2]
    for (unsigned int i = 0; i < count; i++)
    {
    	pio_hw->txf[3] = data[i];
    }
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->instr = 0xe001; // set pins, 1
    sm->instr = can2040_offset_tx_start; // jmp tx_start
    sm->instr = 0x20c0; // wait 1 irq, 0
    pio_hw->ctrl = 0x0f << PIO_CTRL_SM_ENABLE_LSB;
}

// Set PIO "tx" state machine to inject an ack after a CRC match
void CanFD2040::pio_tx_inject_ack(uint32_t match_key) noexcept
{
    pio_tx_reset();
    pio_hw->instr_mem[can2040_offset_tx_got_recessive] = 0xc023; // irq wait 3
    pio_hw->txf[3] = 0x7fffffff;
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->instr = 0xe001; // set pins, 1
    sm->instr = can2040_offset_tx_start; // jmp tx_start
    sm->instr = 0x20c2; // wait 1 irq, 2
    pio_hw->ctrl = 0x0f << PIO_CTRL_SM_ENABLE_LSB;

    pio_match_check(match_key);
}

// Check if the PIO "tx" state machine stopped due to passive/dominant conflict
int CanFD2040::pio_tx_did_conflict() noexcept
{
    return pio_hw->sm[3].addr == can2040_offset_tx_conflict;
}

// Enable host irq on a "may transmit" signal (sm irq 0)
void CanFD2040::pio_irq_set_maytx() noexcept
{
    pio_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS;
}

// Enable host irq on a "may transmit" or "matched" signal (sm irq 0 or 2)
void CanFD2040::pio_irq_set_maytx_matched() noexcept
{
    pio_hw->inte0 = (PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM2_BITS
                     | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS);
}

// Enable host irq on a "may transmit" or "ack done" signal (sm irq 0 or 3)
void CanFD2040::pio_irq_set_maytx_ackdone() noexcept
{
    pio_hw->inte0 = (PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM3_BITS
                     | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS);
}

// Atomically enable "may transmit" signal (sm irq 0)
void CanFD2040::pio_irq_atomic_set_maytx() noexcept
{
    hw_set_bits(&pio_hw->inte0, PIO_IRQ0_INTE_SM0_BITS);
}

// Disable PIO host irqs (except for normal data read irq)
void CanFD2040::pio_irq_set_none() noexcept
{
    pio_hw->inte0 = PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS;
}

// Setup PIO state machines
void CanFD2040::pio_sm_setup() noexcept
{
    // Reset state machines
    pio_hw->ctrl = PIO_CTRL_SM_RESTART_BITS | PIO_CTRL_CLKDIV_RESTART_BITS;
    pio_hw->fdebug = 0xffffffff;

    // Load pio program
    for (unsigned int i=0; i<ARRAY_SIZE(can2040_program_instructions); i++)
    {
    	pio_hw->instr_mem[i] = can2040_program_instructions[i];
    }

    // Set initial state machine state
    pio_sync_setup();
    pio_rx_setup();
    pio_match_setup();
    pio_tx_setup();

    // Start state machines
    pio_hw->ctrl = 0x07 << PIO_CTRL_SM_ENABLE_LSB;
}

// Report error to calling code (via callback interface)
void CanFD2040::report_callback_error(uint32_t error_code) noexcept
{
    //TODO
}

// Report a received message to calling code (via callback interface)
void CanFD2040::report_callback_rx_msg() noexcept
{
	//TODO
}

// Report a message that was successfully transmitted (via callback interface)
void CanFD2040::report_callback_tx_msg() noexcept
{
	// We don't support the Tx event fifo yet
}

// EOF phase complete - report message (rx or tx) to calling code
void CanFD2040::report_handle_eof() noexcept
{
    if (report_state == ReportState::RS_IDLE)
    {
        // Message already reported or an unexpected EOF
        return;
    }
    if (report_state & ReportState::RS_AWAIT_EOF)
    {
        // Successfully processed a new message - report to calling code
        pio_sync_normal_start_signal();
        if (report_state & ReportState::RS_IS_TX)
        {
            report_callback_tx_msg();
        }
        else
        {
            report_callback_rx_msg();
        }
    }
    report_state = ReportState::RS_IDLE;
    pio_match_clear();
}

// Check if in an rx ack is pending
int CanFD2040::report_is_acking_rx() noexcept
{
    return report_state == (ReportState)(ReportState::RS_IN_MSG | ReportState::RS_AWAIT_EOF);
}

// Parser found a new message start
void CanFD2040::report_note_message_start() noexcept
{
    pio_irq_set_maytx();
}

// Setup for ack injection (if receiving) or ack confirmation (if transmit)
void CanFD2040::report_note_crc_start() noexcept
{
	// Get 19 bits of expected data (lower 15 bits of CRC plus 4 stuff bits)
	uint32_t expectedData = (parse_crc & 0x01)						// bit 0 of CRC
						  | (~parse_crc & 0x02)						// fixed stuff bit = inverse of bit 1
						  | ((parse_crc & 0x1e) << 1)				// bits 1-4 of CRC
						  | ((~parse_crc & 0x20) << 1)				// fixed stuff bit = inverse of bit 5
						  | ((parse_crc & 0x1e0) << 2)				// bits 5-8 of CRC
						  | ((~parse_crc & 0x0200) << 2)			// fixed stuff bit = inverse of bit 9
						  | ((parse_crc & 0x1e00) << 3)				// bits 9-12 of CRC
						  | ((~parse_crc & 0x2000) << 3)			// fixed stuff bit = inverse of bit 13
						  | ((parse_crc & 0x6000) << 4);			// bits 13-14 of CRC
    const uint32_t cs = unstuf.GetStuffCount();
    const uint32_t crcstart_bitpos = raw_bit_count - cs - 1;
    const uint32_t crcend_bitpos = crcstart_bitpos + 19;

    if (tx_check_local_message())
    {
        // This is a self transmit - setup tx eof "matched" signal
        report_state = (ReportState)(ReportState::RS_IN_MSG | ReportState::RS_IS_TX);
        expectedData = (expectedData << 10) | 0x02ff;
        pio_match_check(pio_match_calc_key(expectedData, crcend_bitpos + 10));
    }
    else
    {
		// Inject ack
		report_state = ReportState::RS_IN_MSG;
		expectedData = (expectedData << 1) | 0x01;
		const uint32_t match_key = pio_match_calc_key(expectedData, crcend_bitpos + 1);
	    if (   tx_state == TS_QUEUED
	    	&& !pio_tx_did_conflict()
	        && pio_rx_fifo_level() > 1
		   )
	    {
	        // Rx state is behind - acking wont succeed and may halt active tx
	    }
	    else
	    {
			tx_state = TS_ACKING_RX;
			pio_tx_inject_ack(match_key);
			pio_irq_set_maytx_ackdone();

			// Setup for future rx eof "matched" signal
			expectedData = (expectedData << 8) | 0x7f;
			report_eof_key = pio_match_calc_key(expectedData, crcend_bitpos + 9);
		}
    }
}

// Parser found successful ack
void CanFD2040::report_note_ack_success() noexcept
{
    if (!(report_state & ReportState::RS_IN_MSG))
        // Got rx "ackdone" and "matched" signals already
        return;
    report_state = (ReportState)(report_state | ReportState::RS_AWAIT_EOF);
    if (report_state & ReportState::RS_IS_TX)
        // Enable "matched" irq for fast back-to-back transmit scheduling
        pio_irq_set_maytx_matched();
}

// Parser found successful EOF
void CanFD2040::report_note_eof_success() noexcept
{
    report_handle_eof();
}

// Parser found unexpected data on input
void CanFD2040::report_note_parse_error() noexcept
{
    if (report_state != ReportState::RS_IDLE) {
        report_state = ReportState::RS_IDLE;
        pio_match_clear();
    }
    pio_sync_slow_start_signal();
    pio_irq_set_maytx();
}

// Received PIO rx "ackdone" irq
void CanFD2040::report_line_ackdone() noexcept
{
    if (!(report_state & ReportState::RS_IN_MSG)) {
        // Parser already processed ack and eof bits
        pio_irq_set_maytx();
        return;
    }
    // Setup "matched" irq for fast rx callbacks
    report_state = (ReportState)(ReportState::RS_IN_MSG | ReportState::RS_AWAIT_EOF);
    pio_match_check(report_eof_key);
    pio_irq_set_maytx_matched();
    // Schedule next transmit (so it is ready for next frame line arbitration)
    tx_schedule_transmit();
}

// Received PIO "matched" irq
void CanFD2040::report_line_matched() noexcept
{
    // Implement fast rx callback and/or fast back-to-back tx scheduling
    report_handle_eof();
    pio_irq_set_none();
    tx_schedule_transmit();
}

// Received 10+ passive bits on the line (between 10 and 17 bits)
void CanFD2040::report_line_maytx() noexcept
{
    // Line is idle - may be unexpected EOF, missed ack injection,
    // missed "matched" signal, or can2040_transmit() kick.
    report_handle_eof();
    pio_irq_set_none();
    tx_schedule_transmit();
}

// Transition to the next parsing state
void CanFD2040::data_state_go_next(ParseState state, uint32_t bits) noexcept
{
    parse_state = state;
    unstuf.SetCount(bits);
}

// Transition to the MS_DISCARD state - drop all bits until 6 passive bits
void CanFD2040::data_state_go_discard() noexcept
{
    report_note_parse_error();

    if (pio_rx_check_stall())
    {
        // CPU couldn't keep up for some read data - must reset pio state
        raw_bit_count = 0;
        unstuf.ClearStuffCount();
        pio_sm_setup();
        report_callback_error(0);
    }

    data_state_go_next(MS_DISCARD, 32);
}

// Received six dominant bits on the line
void CanFD2040::data_state_line_error() noexcept
{
    data_state_go_discard();
}

// Received six passive bits on the line
void CanFD2040::data_state_line_passive() noexcept
{
    if (parse_state != MS_DISCARD)
    {
        // Bitstuff error
        data_state_go_discard();
    }
    else
    {
		const uint32_t stuffed_bits = unstuf.GetStuffedBits() >> unstuf.GetStuffCount();
		if (stuffed_bits == 0xffffffff)
		{
			// Counter overflow in "sync" state machine - reset it
			pio_sync_setup();
			unstuf.ClearStuffedBits();
			data_state_go_discard();
		}
		else if (((stuffed_bits + 1) & 0x1ff) == 0)		// look for sof after 9 passive bits (most "PIO sync" will produce)
		{
			data_state_go_next(MS_START, 1);
		}
		else
		{
			data_state_go_discard();
		}
    }
}

// Transition to MS_DATA0 state (if applicable) - await data bits
void CanFD2040::data_state_go_data() noexcept
{
	if (parse_dlc == 0)
    {
        data_state_go_stuff_count();						// no data in packet
    }
	else
	{
		parse_bytesReceived = 0;
		parse_bytesLeft = (parse_dlc <= 8) ? parse_dlc
							: (parse_dlc <= 12) ? (parse_dlc - 6) << 2
								: (parse_dlc - 11) << 4;
		data_state_go_next(MS_DATA, 8 * min<uint32_t>(parse_bytesLeft, 4));
	}
}

// Transition to MS_STUFFCOUNT state
void CanFD2040::data_state_go_stuff_count() noexcept
{
	unstuf.UseFixedStuffBits();								// tell the unstuffer that the next bit and then every 4 bits are forced stuff bits
	data_state_go_next(MS_STUFFCOUNT, (parse_dlc >= 10) ? 10 : 6);	// receive the 4 stuff count bits + the first 2 or 6 CRC bits (and 2 fixed stuff bits)
}

// Handle reception of first bit of header (after start-of-frame (SOF))
void CanFD2040::data_state_update_start(uint32_t data) noexcept
{
	rxTimeStamp = timer_hw->timerawl;						// save time stamp for later
    parse_id = data;										// store the first data bit (MSbit of ID)
    report_note_message_start();
    data_state_go_next(MS_HEADER, 17);
}

// Handle reception of next 20 header bits which for CAN-FD frame with short ID takes us up to and including the DLC bits
void CanFD2040::data_state_update_header(uint32_t data) noexcept
{
    data |= parse_id << 20;									// or in the most significant ID bit
    if ((data & 0x0300) == 0x0300)							// if SRR and ID are both set
    {
        // Extended ID
        parse_id = (data & 0x1ffc00) << 8;					// this is now the top 11 bits of the ID field
        data_state_go_next(MS_EXT_HEADER, 19);
    }
    else if ((data & 0x3e0) == 0x80)						// if r1 and IDE are both clear, FDF is set, r0 and and BRS are clear
    {
		// Short ID
		parse_id = (data >> 10) & 0x7ff;					// save the complete ID
		//TODO filter it
		parse_dlc = data & 0x0F;
		data_state_go_data();
    }
	else
	{
		data_state_go_discard();							// not a message format we can parse
	}
}

// Handle reception of additional 19 bits of "extended header" which takes us to the end of the DLC field
void CanFD2040::data_state_update_ext_header(uint32_t data) noexcept
{
	if ((data & 0x1e0) == 0x80)								// if FDF bit is set, and r1, BRS and r0 are not set
	{
		parse_id |= (data >> 9) & 0x03ffff;					// or-in the bottom 18 bits of the ID
		//TODO filter it
		parse_dlc = data & 0x0F;
		data_state_go_data();
	}
	else
	{
		data_state_go_discard();
	}
}

// Handle reception of data content
void CanFD2040::data_state_update_data(uint32_t data) noexcept
{
	if (parse_bytesLeft < 4)
	{
	    rxMessage[parse_bytesReceived/4] = __builtin_bswap32(data << (32 - (8 * parse_bytesLeft)));
	    data_state_go_stuff_count();
	}
	else
	{
		rxMessage[parse_bytesReceived/4] = __builtin_bswap32(data);
		parse_bytesReceived += 4;
		parse_bytesLeft -= 4;
		if (parse_bytesLeft != 0)
		{
			data_state_go_next(MS_DATA, 8 * min<uint32_t>(parse_bytesLeft, 4));
		}
		else
		{
			data_state_go_stuff_count();
		}
	}
}

// Handle reception of 4 bits of message stuff count
void CanFD2040::data_state_update_stuffCount(uint32_t data) noexcept
{
	uint32_t stuffCount;
	uint32_t crcBits;
	if (parse_dlc >= 10)
	{
		parse_crc = unstuf.GetCrc21();		// set up the expected CRC
		stuffCount = data >> 6;				// get stuff count and parity bit
		crcBits = data & 0x3f;				// get top 6 bits of CRC
	}
	else
	{
		parse_crc = unstuf.GetCrc17();		// set up the expected CRC
		stuffCount = data >> 2;				// get stuff count and parity bit
		crcBits = data & 0x03;				// get top 2 bits of CRC
	}

	// Check stuff count parity and stuff count
	uint32_t parity = stuffCount ^ (stuffCount >> 2);
	parity ^= (parity >> 1);
#if 0	//TODO
	if (   (parity & 1)							//TODO should parity be even or odd?
		|| InverseGrayTable(stuffCount >> 1) != unstuff.GetTotalStuffBits()
	   )
	{
		data_state_go_discard();
	}
	else
#else
	(void)parity;
#endif
	if ((parse_crc >> 15) == crcBits)
	{
		// Transition to MS_CRC state - await another 15 bits of crc + delimiter
		report_note_crc_start();
		data_state_go_next(MS_CRC, 16);						// read 15 CRC bits (plus fixed stuffing bits) plus delimiter
	}
	else
	{
		data_state_go_discard();
	}
}

// Handle reception of 16 bits of message CRC (15 crc bits + crc delimiter)
void CanFD2040::data_state_update_crc(uint32_t data) noexcept
{
#if 0	//TODO don't check CRC
    if ((((parse_crc & 0x7fff) << 1) | 1) != data)
    {
        data_state_go_discard();
    }
    else
#endif
    {
		unstuf.ClearState();
		data_state_go_next(MS_ACK, 2);
    }
}

// Handle reception of 2 bits of ack phase (ack, ack delimiter)
void CanFD2040::data_state_update_ack(uint32_t data) noexcept
{
    if (data != 0x01)
    {
        data_state_go_discard();
        return;
    }
    report_note_ack_success();
    data_state_go_next(MS_EOF0, 4);
}

// Handle reception of first four end-of-frame (EOF) bits
void CanFD2040::data_state_update_eof0(uint32_t data) noexcept
{
    if (data != 0x0f || pio_rx_check_stall())
    {
        data_state_go_discard();
        return;
    }
    unstuf.ClearState();
    data_state_go_next(MS_EOF1, 4);
}

// Handle reception of end-of-frame (EOF) bits 5-7 and first IFS bit
void CanFD2040::data_state_update_eof1(uint32_t data) noexcept
{
    if (data >= 0x0e || (data >= 0x0c && report_is_acking_rx()))
    {
        // Message is considered fully transmitted
        report_note_eof_success();
    }

    if (data == 0x0f)
    {
        data_state_go_next(MS_START, 1);
    }
    else
    {
        data_state_go_discard();
    }
}

// Handle data received while in MS_DISCARD state
void CanFD2040::data_state_update_discard(uint32_t data) noexcept
{
    data_state_go_discard();
}

// Update parsing state after reading the bits of the current field
void CanFD2040::data_state_update(uint32_t data) noexcept
{
	switch (parse_state)
	{
	case MS_START:		data_state_update_start(data); break;
	case MS_HEADER:		data_state_update_header(data); break;
	case MS_EXT_HEADER:	data_state_update_ext_header(data); break;
	case MS_DATA:		data_state_update_data(data); break;
	case MS_STUFFCOUNT:	data_state_update_stuffCount(data); break;
	case MS_CRC:		data_state_update_crc(data); break;
	case MS_ACK:		data_state_update_ack(data); break;
	case MS_EOF0:		data_state_update_eof0(data); break;
	case MS_EOF1:		data_state_update_eof1(data); break;
	case MS_DISCARD:	data_state_update_discard(data); break;
	}
}

/****************************************************************
 * Input processing
 ****************************************************************/

// Process an incoming byte of data from PIO "rx" state machine
void CanFD2040::process_rx(uint32_t rx_byte) noexcept
{
	unstuf.AddBits(rx_byte, 8);
    raw_bit_count += 8;

    // undo bit stuffing
    for (;;)
    {
        int ret = unstuf.PullBits();
        if (likely(ret > 0))
        {
            // Need more data
            break;
        } else if (likely(!ret))
        {
            // Pulled the next field - process it
            data_state_update(unstuf.GetUnstuffedBits());
        } else
        {
            if (ret == -1)
            {
                // 6 consecutive high bits
                data_state_line_passive();
            }
            else
            {
                // 6 consecutive low bits
                data_state_line_error();
            }
        }
    }
}

// Main API irq notification function
void CanFD2040::pio_irq_handler() noexcept
{
    uint32_t ints = pio_hw->ints0;
    while (likely(ints & PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS))
    {
        uint8_t rx_byte = pio_hw->rxf[1];
        process_rx(rx_byte);
        ints = pio_hw->ints0;
        if (likely(!ints))
        {
            return;
        }
    }

    if (ints & PIO_IRQ0_INTE_SM3_BITS)
    {
        // Ack of received message completed successfully
        report_line_ackdone();
    }
    else if (ints & PIO_IRQ0_INTE_SM2_BITS)
    {
        // Transmit message completed successfully
        report_line_matched();
    }
    else if (ints & PIO_IRQ0_INTE_SM0_BITS)
    {
        // Bus is idle, but not all bits may have been flushed yet
        report_line_maytx();
    }
}

/****************************************************************
 * Transmit state tracking
 ****************************************************************/

// Queue the next message for transmission in the PIO
void CanFD2040::tx_schedule_transmit() noexcept
{
    if (tx_state == TS_QUEUED && !pio_tx_did_conflict())
    {
        // Already queued or actively transmitting
        return;
    }
    if (txStuffedWords == 0)
    {
        // No new messages to transmit
        tx_state = TS_IDLE;
        return;
    }
    tx_state = TS_QUEUED;
    pio_tx_send(txMessage, txStuffedWords);
}

// Check if the current parsed message is feedback from current transmit
bool CanFD2040::tx_check_local_message() noexcept
{
    if (tx_state == TS_QUEUED && txStuffedWords != 0 && txCrc == parse_crc && txId == parse_id && txDlc == parse_dlc)		// ideally we would check the data here too
    {
        // Received our own transmission
        tx_state = TS_CONFIRM_TX;
        return true;
    }
    return false;
}

extern VirtualCanRegisters virtualRegs;
static CanFD2040 canFdDevice;

extern "C" [[noreturn]]void Core1Entry() noexcept
{
	canFdDevice.Entry(&virtualRegs);
}

extern "C" void PIO_isr() noexcept
{
	canFdDevice.pio_irq_handler();
}

// End
