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
#include "hardware/structs/pio.h" // pio0_hw
#include "hardware/structs/resets.h" // RESETS_RESET_PIO0_BITS

#include <cstring>

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

/****************************************************************
 * Bit unstuffing
 ****************************************************************/

// Add 'count' number of bits from 'data' to the 'bu' unstuffer
void BitUnstuffer::unstuf_add_bits(uint32_t data, uint32_t count) noexcept
{
    const uint32_t mask = (1 << count) - 1;
    stuffed_bits = (stuffed_bits << count) | (data & mask);
    count_stuff = count;
}

// Reset state and set the next desired 'count' unstuffed bits to extract
void BitUnstuffer::unstuf_set_count(uint32_t count) noexcept
{
    unstuffed_bits = 0;
    count_unstuff = count;
}

// Clear bitstuffing state (used after crc field to avoid bitstuffing ack field)
void BitUnstuffer::unstuf_clear_state() noexcept
{
    const uint32_t lb = 1 << count_stuff;
    stuffed_bits = (stuffed_bits & (lb - 1)) | lb;
}

// Pull bits from unstuffer (as specified in unstuf_set_count() )
int BitUnstuffer::unstuf_pull_bits() noexcept
{
    uint32_t sb = stuffed_bits, edges = sb ^ (sb >> 1);
    uint32_t e2 = edges | (edges >> 1), e4 = e2 | (e2 >> 2), rm_bits = ~e4;
    uint32_t cs = count_stuff, cu = count_unstuff;
    if (!cs)
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
            if (likely(!(rm_bits & try_mask)))
            {
                // No stuff bits in try_cnt bits - copy into unstuffed_bits
                count_unstuff = cu = cu - try_cnt;
                count_stuff = cs = cs - try_cnt;
                unstuffed_bits |= ((sb >> cs) & ((1 << try_cnt) - 1)) << cu;
                if (! cu)
                {
                    // Extracted desired bits
                    return 0;
                }
                break;
            }
            count_stuff = cs = cs - 1;
            if (rm_bits & (1 << (cs + 1)))
            {
                // High bit of try_cnt a stuff bit
                if (unlikely(rm_bits & (1 << cs)))
                {
                    // Six consecutive bits - a bitstuff error
                    return ((sb >> cs) & 1) ? -1 : -2;
                }
                break;
            }
            // High bit not a stuff bit - limit try_cnt and retry
            count_unstuff = cu = cu - 1;
            unstuffed_bits |= ((sb >> cs) & 1) << cu;
            try_cnt /= 2;
        }
        if (likely(!cs))
        {
            // Need more data
            return 1;
        }
    }
}

/****************************************************************
 * Bit stuffing
 ****************************************************************/

// Stuff 'num_bits' bits in '*pb' - upper bits must already be stuffed
static uint32_t bitstuff(uint32_t *pb, uint32_t num_bits) noexcept
{
    uint32_t b = *pb, count = num_bits;
    for (;;) {
        uint32_t try_cnt = num_bits, edges = b ^ (b >> 1);
        uint32_t e2 = edges | (edges >> 1), e4 = e2 | (e2 >> 2), add_bits = ~e4;
        for (;;) {
            uint32_t try_mask = ((1 << try_cnt) - 1) << (num_bits - try_cnt);
            if (!(add_bits & try_mask)) {
                // No stuff bits needed in try_cnt bits
                if (try_cnt >= num_bits)
                    goto done;
                num_bits -= try_cnt;
                try_cnt = (num_bits + 1) / 2;
                continue;
            }
            if (add_bits & (1 << (num_bits - 1))) {
                // A stuff bit must be inserted prior to the high bit
                uint32_t low_mask = (1 << num_bits) - 1, low = b & low_mask;
                uint32_t high = (b & ~(low_mask >> 1)) << 1;
                b = high ^ low ^ (1 << (num_bits - 1));
                count += 1;
                if (num_bits <= 4)
                    goto done;
                num_bits -= 4;
                break;
            }
            // High bit doesn't need stuff bit - accept it, limit try_cnt, retry
            num_bits--;
            try_cnt /= 2;
        }
    }
done:
    *pb = b;
    return count;
}

// State storage for building bit stuffed transmit messages
struct bitstuffer_s {
    uint32_t prev_stuffed, bitpos, *buf;
};

// Push 'count' bits of 'data' into stuffer without performing bit stuffing
static void bs_pushraw(struct bitstuffer_s *bs, uint32_t data, uint32_t count) noexcept
{
    uint32_t bitpos = bs->bitpos;
    uint32_t wp = bitpos / 32, bitused = bitpos % 32, bitavail = 32 - bitused;
    uint32_t *fb = &bs->buf[wp];
    if (bitavail >= count) {
        fb[0] |= data << (bitavail - count);
    } else {
        fb[0] |= data >> (count - bitavail);
        fb[1] |= data << (32 - (count - bitavail));
    }
    bs->bitpos = bitpos + count;
}

// Push 'count' bits of 'data' into stuffer
static void bs_push(struct bitstuffer_s *bs, uint32_t data, uint32_t count) noexcept
{
    data &= (1 << count) - 1;
    uint32_t stuf = (bs->prev_stuffed << count) | data;
    uint32_t newcount = bitstuff(&stuf, count);
    bs_pushraw(bs, stuf, newcount);
    bs->prev_stuffed = stuf;
}

// Pad final word of stuffer with high bits
static uint32_t bs_finalize(struct bitstuffer_s *bs) noexcept
{
    uint32_t bitpos = bs->bitpos;
    uint32_t words = DIV_ROUND_UP(bitpos, 32);
    uint32_t extra = words * 32 - bitpos;
    if (extra)
        bs->buf[words - 1] |= (1 << extra) - 1;
    return words;
}

// Set up a message ready to be queued for transmission. The callback function is set to null.
void CanFD2040TransmitBuffer::Populate(uint32_t p_id, const uint8_t *data, unsigned int numBytes) noexcept
{
	if (numBytes > 64)
	{
		numBytes = 64;				// prevent things going horribly wrong if the caller made a mistake
	}

	// Set up the ID field
    if (p_id & CanFD2040ReceiveBuffer::IdExtraBits::ID_EFF)
    {
        id = p_id & ~0x20000000;
    }
    else
    {
        id = p_id & 0x7ff;			// we don't support RTR
    }

    // Set up the dlc field and compute how many padding btyes we need to add
    size_t padding;
    if (numBytes <= 8)
    {
    	dlc = numBytes;
    	padding = 0;
    }
    else if (numBytes <= 24)
    {
    	padding = 4 - (numBytes & 0x03);
    	dlc = ((numBytes + padding) >> 2) + 6;
    }
    else
    {
    	padding = 16 - (numBytes & 0x0F);
    	dlc = ((numBytes + padding) >> 4) + 11;
    }

    // Calculate crc and stuff bits
    memset(stuffed_data, 0, sizeof(stuffed_data));
    bitstuffer_s bs = { 1, 0, stuffed_data };
    uint32_t locCrc = 0;
    if (p_id & CanFD2040ReceiveBuffer::IdExtraBits::ID_EFF)
    {
        // Extended header
        const uint32_t eid = p_id;
        const uint32_t h1 = ((eid & 0x1ffc0000) >> 11) | 0x60 | ((eid & 0x3e000) >> 13);
        const uint32_t h2 = ((eid & 0x1fff) << 7) | dlc;
        locCrc = crc_bytes(locCrc, h1 >> 4, 2);
        locCrc = crc_bytes(locCrc, ((h1 & 0x0f) << 20) | h2, 3);
        bs_push(&bs, h1, 19);
        bs_push(&bs, h2, 20);
    }
    else
    {
        // Standard header
        uint32_t hdr = ((p_id & 0x7ff) << 7) | dlc;
        locCrc = crc_bytes(locCrc, hdr, 3);
        bs_push(&bs, hdr, 19);
    }

    for (size_t i = 0; i < numBytes; i++)
    {
        locCrc = crc_byte(locCrc, data[i]);
        bs_push(&bs, data[i], 8);
    }
    for (size_t i = 0; i < padding; i++)
    {
        locCrc = crc_byte(locCrc, 0);
        bs_push(&bs, 0, 8);
    }
    crc = locCrc & 0x7fff;
    bs_push(&bs, crc, 15);
    bs_pushraw(&bs, 1, 1);
    stuffed_words = bs_finalize(&bs);

	completionCallback = nullptr;
}

// Set up the callback. Call this after populating the buffer but before queueing it for transmission.
void CanFD2040TransmitBuffer::SetCallback(CanTransmitCallback fn, uint32_t param) noexcept
{
	completionCallback = fn;
	completionParam = param;
}

// CanIdFilter members
CanIdFilter::CanIdFilter(uint32_t p_id, uint32_t p_mask, CanReceiveCallback p_callback) noexcept
	: id(p_id), mask(p_mask), callback(p_callback), receivedMessages(nullptr)
{
}

// Offer a message to the filter. Returns true if the message is accepted and added to the list.
bool CanIdFilter::Offer(const CanFD2040ReceiveBuffer *msg) const noexcept
{
	return (msg->id & mask) == id;
}

// Add a message to the queue
// Caller must already have entered the critical section that guards the buffer list.
void CanIdFilter::AddMessage(CanFD2040ReceiveBuffer *msg) noexcept
{
	msg->next = nullptr;
	CanFD2040ReceiveBuffer **bufp = &receivedMessages;
	while (*bufp != nullptr)
	{
		bufp = &((*bufp)->next);
	}
	*bufp = msg;
}

// Get the message at the head of the receive queue. The caller becomes responsible for releasing the buffer.
// Caller must already have entered the critical section that guards the buffer list.
CanFD2040ReceiveBuffer* CanIdFilter::GetMessage() noexcept
{
	CanFD2040ReceiveBuffer *const buf = receivedMessages;
	if (buf != nullptr)
	{
		receivedMessages = buf->next;
	}
	return buf;
}

// Execute the callback
void CanIdFilter::ExecuteCallback(unsigned int filterNumber) const noexcept
{
	if (callback != nullptr)
	{
		callback(filterNumber);
	}
}

// CanFD2040 members
CanFD2040::CanFD2040(uint8_t p_pio_num, Pin txPin, Pin rxPin) noexcept
	: pio_num(p_pio_num), gpio_rx(rxPin), gpio_tx(txPin),
	  txFreelist(nullptr), rxFreelist(nullptr),
	  filters(nullptr), parse_msg(nullptr), tx_queue(nullptr)
{
    pio_hw = (pio_num) ? pio1_hw : pio0_hw;
}

// Start CAN-FD running
void CanFD2040::Start(uint32_t sys_clock, uint32_t bitrate, unsigned int numTxBuffers, unsigned int numRxBuffers) noexcept
{
	critical_section_init(&txQueueCriticalSection);
	critical_section_init(&rxQueueCriticalSection);
	while (numTxBuffers != 0)
	{
		txFreelist = new CanFD2040TransmitBuffer(txFreelist);
		--numTxBuffers;
	}
	while (numRxBuffers != 0)
	{
		rxFreelist = new CanFD2040ReceiveBuffer(rxFreelist);
		--numRxBuffers;
	}
	pio_setup(sys_clock, bitrate);
    data_state_go_discard();
}

// Add a filter to the end of the current list. There is currently no facility to remove a filter.
void CanFD2040::AddFilter(CanIdFilter *p_filter) noexcept
{
	p_filter->next = nullptr;
	CanIdFilter **fpp = &filters;
	while (*fpp != nullptr)
	{
		fpp = &(*fpp)->next;
	}
	*fpp = p_filter;
}

// Queue a message for transmission. After transmission or failure, the callback will be made if non-null and then the buffer will be released.
void CanFD2040::QueueForTransmit(CanFD2040TransmitBuffer *buf) noexcept
{
	buf->next = nullptr;
	CanFD2040TransmitBuffer **bufp = &tx_queue;
	critical_section_enter_blocking(&txQueueCriticalSection);
	while (*bufp != nullptr)
	{
		bufp = &((*bufp)->next);
	}
	*bufp = buf;
	critical_section_exit(&txQueueCriticalSection);

    // Wakeup if in TS_IDLE state
    pio_irq_atomic_set_maytx();
}

// Get the message at the head of the receive queue for the specified filter number. The caller must release the buffer.
CanFD2040ReceiveBuffer *CanFD2040::GetMessage(unsigned int filterNumber) noexcept
{
	CanIdFilter *filter = filters;
	for (;;)
	{
		if (filter == nullptr)
		{
			return nullptr;
		}
		if (filterNumber == 0)
		{
			break;
		}
		--filterNumber;
		filter = filter->next;
	}

	critical_section_enter_blocking(&rxQueueCriticalSection);
	CanFD2040ReceiveBuffer *ret = filter->GetMessage();
	critical_section_exit(&rxQueueCriticalSection);
	return ret;
}

// Free a receive buffer that was returned by GetMessage
void CanFD2040::ReleaseReceiveBuffer(CanFD2040ReceiveBuffer *buf) noexcept
{
	critical_section_enter_blocking(&rxQueueCriticalSection);
	buf->next = rxFreelist;
	rxFreelist = buf;
	critical_section_exit(&rxQueueCriticalSection);
}

void CanFD2040::pio_setup(uint32_t sys_clock, uint32_t bitrate) noexcept
{
	constexpr int PIO_FUNC = 6;

    // Configure pio0 clock
    uint32_t rb = pio_num ? RESETS_RESET_PIO1_BITS : RESETS_RESET_PIO0_BITS;
    rp2040_clear_reset(rb);

    // Setup and sync pio state machine clocks
    uint32_t div = (256 / PIO_CLOCK_PER_BIT) * sys_clock / bitrate;
    int i;
    for (i=0; i<4; i++)
    {
    	((pio_hw_t*)pio_hw)->sm[i].clkdiv = div << PIO_SM0_CLKDIV_FRAC_LSB;
    }

    // Configure state machines
    pio_sm_setup();

    // Map Rx/Tx gpios
    rp2040_gpio_peripheral(gpio_rx, PIO_FUNC, 1);
    rp2040_gpio_peripheral(gpio_tx, PIO_FUNC, 0);
}

// Setup PIO "sync" state machine (state machine 0)
void CanFD2040::pio_sync_setup() noexcept
{
    struct pio_sm_hw *sm = &((pio_hw_t*)pio_hw)->sm[0];
    sm->execctrl = (
        gpio_rx << PIO_SM0_EXECCTRL_JMP_PIN_LSB
        | (can2040_offset_sync_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_sync_signal_start << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->pinctrl = (
        1 << PIO_SM0_PINCTRL_SET_COUNT_LSB
        | gpio_rx << PIO_SM0_PINCTRL_SET_BASE_LSB);
    sm->instr = 0xe080; // set pindirs, 0
    sm->pinctrl = 0;
    ((pio_hw_t*)pio_hw)->txf[0] = PIO_CLOCK_PER_BIT / 2 * 8 - 5 - 1;
    sm->instr = 0x80a0; // pull block
    sm->instr = can2040_offset_sync_entry; // jmp sync_entry
}

// Setup PIO "rx" state machine (state machine 1)
void CanFD2040::pio_rx_setup() noexcept
{
    struct pio_sm_hw *sm = &((pio_hw_t*)pio_hw)->sm[1];
    sm->execctrl = (
        (can2040_offset_shared_rx_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->pinctrl = gpio_rx << PIO_SM0_PINCTRL_IN_BASE_LSB;
    sm->shiftctrl = 0; // flush fifo on a restart
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS
                     | 8 << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB
                     | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    sm->instr = can2040_offset_shared_rx_read; // jmp shared_rx_read
}

// Setup PIO "match" state machine (state machine 2)
void CanFD2040::pio_match_setup() noexcept
{
     struct pio_sm_hw *sm = &((pio_hw_t*)pio_hw)->sm[2];
    sm->execctrl = (
        (can2040_offset_match_end - 1) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB
        | can2040_offset_shared_rx_read << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    sm->pinctrl = gpio_rx << PIO_SM0_PINCTRL_IN_BASE_LSB;
    sm->shiftctrl = 0;
    sm->instr = 0xe040; // set y, 0
    sm->instr = 0xa0e2; // mov osr, y
    sm->instr = 0xa02a, // mov x, !y
    sm->instr = can2040_offset_match_load_next; // jmp match_load_next
}

// Setup PIO "tx" state machine (state machine 3)
void CanFD2040::pio_tx_setup() noexcept
{
    struct pio_sm_hw *sm = &((pio_hw_t*)pio_hw)->sm[3];
    sm->execctrl = gpio_rx << PIO_SM0_EXECCTRL_JMP_PIN_LSB;
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS
                     | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS);
    sm->pinctrl = (1 << PIO_SM0_PINCTRL_SET_COUNT_LSB
                   | 1 << PIO_SM0_PINCTRL_OUT_COUNT_LSB
                   | gpio_tx << PIO_SM0_PINCTRL_SET_BASE_LSB
                   | gpio_tx << PIO_SM0_PINCTRL_OUT_BASE_LSB);
    sm->instr = 0xe001; // set pins, 1
    sm->instr = 0xe081; // set pindirs, 1
}

// Set PIO "sync" machine to signal "may transmit" (sm irq 0) on 11 idle bits
void CanFD2040::pio_sync_normal_start_signal() noexcept
{
    uint32_t eom_idx = can2040_offset_sync_found_end_of_message;
    ((pio_hw_t*)pio_hw)->instr_mem[eom_idx] = 0xe13a; // set x, 26 [1]
}

// Set PIO "sync" machine to signal "may transmit" (sm irq 0) on 17 idle bits
void CanFD2040::pio_sync_slow_start_signal() noexcept
{
    uint32_t eom_idx = can2040_offset_sync_found_end_of_message;
    ((pio_hw_t*)pio_hw)->instr_mem[eom_idx] = 0xa127; // mov x, osr [1]
}

// Test if PIO "rx" state machine has overflowed its fifos
int CanFD2040::pio_rx_check_stall() noexcept
{
    return ((pio_hw_t*)pio_hw)->fdebug & (1 << (PIO_FDEBUG_RXSTALL_LSB + 1));
}

// Report number of bytes still pending in PIO "rx" fifo queue
int CanFD2040::pio_rx_fifo_level() noexcept
{
    return (((pio_hw_t*)pio_hw)->flevel & PIO_FLEVEL_RX1_BITS) >> PIO_FLEVEL_RX1_LSB;
}

// Set PIO "match" state machine to raise a "matched" signal on a bit sequence
void CanFD2040::pio_match_check(uint32_t match_key) noexcept
{
    ((pio_hw_t*)pio_hw)->txf[2] = match_key;
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
    ((pio_hw_t*)pio_hw)->ctrl = ((0x07 << PIO_CTRL_SM_ENABLE_LSB)
                    | (0x08 << PIO_CTRL_SM_RESTART_LSB));
    ((pio_hw_t*)pio_hw)->irq = (1 << 2) | (1<< 3); // clear "matched" and "ack done" signals
    // Clear tx fifo
    struct pio_sm_hw *sm = &((pio_hw_t*)pio_hw)->sm[3];
    sm->shiftctrl = 0;
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS
                     | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS);
    // Must reset again after clearing fifo
    ((pio_hw_t*)pio_hw)->ctrl = ((0x07 << PIO_CTRL_SM_ENABLE_LSB)
                    | (0x08 << PIO_CTRL_SM_RESTART_LSB));
}

// Queue a message for transmission on PIO "tx" state machine
void CanFD2040::pio_tx_send(uint32_t *data, uint32_t count) noexcept
{
    pio_tx_reset();
    ((pio_hw_t*)pio_hw)->instr_mem[can2040_offset_tx_got_recessive] = 0xa242; // nop [2]
    for (unsigned int i=0; i<count; i++)
    {
    	((pio_hw_t*)pio_hw)->txf[3] = data[i];
    }
    struct pio_sm_hw *sm = &((pio_hw_t*)pio_hw)->sm[3];
    sm->instr = 0xe001; // set pins, 1
    sm->instr = can2040_offset_tx_start; // jmp tx_start
    sm->instr = 0x20c0; // wait 1 irq, 0
    ((pio_hw_t*)pio_hw)->ctrl = 0x0f << PIO_CTRL_SM_ENABLE_LSB;
}

// Set PIO "tx" state machine to inject an ack after a CRC match
void CanFD2040::pio_tx_inject_ack(uint32_t match_key) noexcept
{
    pio_tx_reset();
    ((pio_hw_t*)pio_hw)->instr_mem[can2040_offset_tx_got_recessive] = 0xc023; // irq wait 3
    ((pio_hw_t*)pio_hw)->txf[3] = 0x7fffffff;
    struct pio_sm_hw *sm = &((pio_hw_t*)pio_hw)->sm[3];
    sm->instr = 0xe001; // set pins, 1
    sm->instr = can2040_offset_tx_start; // jmp tx_start
    sm->instr = 0x20c2; // wait 1 irq, 2
    ((pio_hw_t*)pio_hw)->ctrl = 0x0f << PIO_CTRL_SM_ENABLE_LSB;

    pio_match_check(match_key);
}

// Check if the PIO "tx" state machine stopped due to passive/dominant conflict
int CanFD2040::pio_tx_did_conflict() noexcept
{
    return ((pio_hw_t*)pio_hw)->sm[3].addr == can2040_offset_tx_conflict;
}

// Enable host irq on a "may transmit" signal (sm irq 0)
void CanFD2040::pio_irq_set_maytx() noexcept
{
    ((pio_hw_t*)pio_hw)->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS;
}

// Enable host irq on a "may transmit" or "matched" signal (sm irq 0 or 2)
void CanFD2040::pio_irq_set_maytx_matched() noexcept
{
    ((pio_hw_t*)pio_hw)->inte0 = (PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM2_BITS
                     | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS);
}

// Enable host irq on a "may transmit" or "ack done" signal (sm irq 0 or 3)
void CanFD2040::pio_irq_set_maytx_ackdone() noexcept
{
    ((pio_hw_t*)pio_hw)->inte0 = (PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM3_BITS
                     | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS);
}

// Atomically enable "may transmit" signal (sm irq 0)
void CanFD2040::pio_irq_atomic_set_maytx() noexcept
{
    hw_set_bits(&((pio_hw_t*)pio_hw)->inte0, PIO_IRQ0_INTE_SM0_BITS);
}

// Disable PIO host irqs (except for normal data read irq)
void CanFD2040::pio_irq_set_none() noexcept
{
    ((pio_hw_t*)pio_hw)->inte0 = PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS;
}

// Setup PIO state machines
void CanFD2040::pio_sm_setup() noexcept
{
    // Reset state machines
    ((pio_hw_t*)pio_hw)->ctrl = PIO_CTRL_SM_RESTART_BITS | PIO_CTRL_CLKDIV_RESTART_BITS;
    ((pio_hw_t*)pio_hw)->fdebug = 0xffffffff;

    // Load pio program
    for (unsigned int i=0; i<ARRAY_SIZE(can2040_program_instructions); i++)
    {
    	((pio_hw_t*)pio_hw)->instr_mem[i] = can2040_program_instructions[i];
    }

    // Set initial state machine state
    pio_sync_setup();
    pio_rx_setup();
    pio_match_setup();
    pio_tx_setup();

    // Start state machines
    ((pio_hw_t*)pio_hw)->ctrl = 0x07 << PIO_CTRL_SM_ENABLE_LSB;
}

// Report error to calling code (via callback interface)
void CanFD2040::report_callback_error(uint32_t error_code) noexcept
{
    //TODO
}

// Report a received message to calling code (via callback interface)
void CanFD2040::report_callback_rx_msg() noexcept
{
	// Scan the filters to see if there is a matching one
	unsigned int filterNumber = 0;
	for (CanIdFilter *f = filters; f != nullptr; f = f->next)
	{
		if (f->Offer(parse_msg))
		{
			critical_section_enter_blocking(&rxQueueCriticalSection);
			f->AddMessage(parse_msg);
			parse_msg = rxFreelist;
			if (parse_msg != nullptr)
			{
				rxFreelist = parse_msg->next;
			}
			critical_section_exit(&rxQueueCriticalSection);
			f->ExecuteCallback(filterNumber);
			return;
		}
		++filterNumber;
	}

	// If we get here then no filter accepted the message, so re-use the buffer
}

// Report a message that was successfully transmitted (via callback interface)
void CanFD2040::report_callback_tx_msg() noexcept
{
	CanFD2040TransmitBuffer *txb = tx_queue;
	if (txb != nullptr)			// should always be true
	{
		if (txb->completionCallback != nullptr)
		{
			txb->completionCallback(true, txb->completionParam);
		}
		critical_section_enter_blocking(&txQueueCriticalSection);
		tx_queue = txb->next;
		txb->next = txFreelist;
		txFreelist = txb;
		critical_section_exit(&txQueueCriticalSection);
	}
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
    uint32_t cs = unstuf.GetStuffCount();
    uint32_t crcstart_bitpos = raw_bit_count - cs - 1;
    uint32_t last = ((unstuf.GetStuffedBits() >> cs) << 15) | parse_crc;
    uint32_t crc_bitcount = bitstuff(&last, 15 + 1) - 1;
    uint32_t crcend_bitpos = crcstart_bitpos + crc_bitcount;

    bool ret = tx_check_local_message();
    if (ret)
    {
        // This is a self transmit - setup tx eof "matched" signal
        report_state = (ReportState)(ReportState::RS_IN_MSG | ReportState::RS_IS_TX);
        last = (last << 10) | 0x02ff;
        pio_match_check(pio_match_calc_key(last, crcend_bitpos + 10));
        return;
    }

    // Inject ack
    report_state = ReportState::RS_IN_MSG;
    last = (last << 1) | 0x01;
    ret = tx_inject_ack(pio_match_calc_key(last, crcend_bitpos + 1));
    if (ret)
    {
        // Ack couldn't be scheduled (due to lagged parsing state)
        return;
    }
    pio_irq_set_maytx_ackdone();
    // Setup for future rx eof "matched" signal
    last = (last << 8) | 0x7f;
    report_eof_key = pio_match_calc_key(last, crcend_bitpos + 9);
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
    unstuf.unstuf_set_count(bits);
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
        return;
    }

    uint32_t stuffed_bits = unstuf.GetStuffedBits() >> unstuf.GetStuffCount();
    if (stuffed_bits == 0xffffffff)
    {
        // Counter overflow in "sync" state machine - reset it
        pio_sync_setup();
        unstuf.ClearStuffedBits();
        data_state_go_discard();
        return;
    }

    // Look for sof after 9 passive bits (most "PIO sync" will produce)
    if (((stuffed_bits + 1) & 0x1ff) == 0)
    {
        data_state_go_next(MS_START, 1);
        return;
    }

    data_state_go_discard();
}

// Transition to MS_CRC state - await 16 bits of crc
void CanFD2040::data_state_go_crc() noexcept
{
    parse_crc &= 0x7fff;
    report_note_crc_start();
    data_state_go_next(MS_CRC, 16);
}

// Transition to MS_DATA0 state (if applicable) - await data bits
void CanFD2040::data_state_go_data(uint32_t id, uint32_t data) noexcept
{
    if (data & (0x03 << 4)) {
        // Not a supported header
        data_state_go_discard();
        return;
    }
    parse_msg->data32[0] = parse_msg->data32[1] = 0;
    parse_dlc = data & 0x0f;
    if (data & (1 << 6)) {
    	parse_dlc = 0;
        id |= CanFD2040ReceiveBuffer::IdExtraBits::ID_RTR;
    }
    parse_msg->id = id;
    if (parse_dlc)
        data_state_go_next(MS_DATA0, (parse_dlc >= 4) ? 32 : parse_dlc * 8);
    else
        data_state_go_crc();
}

// Handle reception of first bit of header (after start-of-frame (SOF))
void CanFD2040::data_state_update_start(uint32_t data) noexcept
{
#if SUPPORT_CAN_FD
	//TODO save the time stamp for later
#endif
    parse_msg->id = data;
    report_note_message_start();
    data_state_go_next(MS_HEADER, 17);
}

// Handle reception of next 17 header bits which for CAN frame with short ID takes us up to and including the DLC bits
void CanFD2040::data_state_update_header(uint32_t data) noexcept
{
    data |= parse_msg->id << 17;							// or in the most significant ID bit
    if ((data & 0x60) == 0x60) {
        // Extended header
        parse_msg->id = data;								// this is now the first 11 bits of the ID field shifted left 7 bits
        data_state_go_next(MS_EXT_HEADER, 20);
        return;
    }
#if SUPPORT_CAN_FD
    const uint32_t id = (data >> 7) & 0x7ff;
    if (data & 0x10) {										// if FDF bit is set
    	data_state_go_dlc(id, data, 3);						// we need to read 3 more bits to get the complete DLC
    	if (data & (1u << 2)) {								// if BRS bit is set
            data_state_go_discard();
            return;
    	}
    	parse_msg.id = id | CAN2040_ID_FDF;
    	parse_scratch = data << 3;							// save for when we know which CRC algorithm to use
    	data_state_go_next(MS_DLC, 3);
    } else {
    	cd->parse_crc = crc_bytes(0, data, 3);				// CRC the complete header (18 bits)
    	data_state_go_data(id, data);
    }
#else
    parse_crc = crc_bytes(0, data, 3);
    data_state_go_data((data >> 7) & 0x7ff, data);
#endif
}

// Handle reception of additional 20 bits of "extended header"
void CanFD2040::data_state_update_ext_header(uint32_t data) noexcept
{
    uint32_t hdr1 = parse_msg->id;
#if SUPPORT_CAN_FD
    uint32_t id = (((hdr1 << 11) & 0x1ffc0000) | ((hdr1 << 13) & 0x3e000)
                   | (data >> 7) | CAN2040_ID_EFF);
    if (data & 0x20) {										// if FDF bit is set
    	if (data & 0x08) {									// if BRS bit is set
            data_state_go_discard(cd);
            return;
    	}
    	cd->parse_msg.id = id | CAN2040_ID_FDF;
    	cd->parse_scratch = data << 2;						// save for when we know which CRC algorithm to use
    	data_state_go_next(cd, MS_DLC, 2);
    } else {
        uint32_t crc = crc_bytes(0, hdr1 >> 4, 2);
        cd->parse_crc = crc_bytes(crc, ((hdr1 & 0x0f) << 20) | data, 3);
        data_state_go_data(cd, id, data);
    }
#else
    uint32_t crc = crc_bytes(0, hdr1 >> 4, 2);
    parse_crc = crc_bytes(crc, ((hdr1 & 0x0f) << 20) | data, 3);
    uint32_t id = (((hdr1 << 11) & 0x1ffc0000) | ((hdr1 << 13) & 0x3e000)
                   | (data >> 7) | CanFD2040ReceiveBuffer::IdExtraBits::ID_EFF);
    data_state_go_data(id, data);
#endif
}

#if SUPPORT_CAN_FD

// Handle reception of the last 2 bits of the DLC field in a CAN-FD message
void CanFD2040::data_state_update_dlc(uint32_t data) noexcept
{
	data |= parse_scratch;
	parse_dlc = data & 0x0f;
	if (dlc >= 8) {
		crc = qq;
		dwordsLeft = (parse_dlc <= 12) ? parse_dlc - 6 : (parse_dlc - 11) << 2;
		qq;
	} else {
	    parse_msg->data32[0] = parse_msg->data32[1] = 0;
	    if (parse_dlc)
	        data_state_go_next(MS_DATA0, parse_dlc >= 4 ? 32 : parse_dlc * 8);
	    else
	        data_state_go_crc();
	}
}

#endif

// Handle reception of first 1-4 bytes of data content
void CanFD2040::data_state_update_data0(uint32_t data) noexcept
{
    uint32_t bits = parse_dlc >= 4 ? 32 : parse_dlc * 8;
    parse_crc = crc_bytes(parse_crc, data, parse_dlc);
    parse_msg->data32[0] = __builtin_bswap32(data << (32 - bits));
    if (parse_dlc > 4)
        data_state_go_next(MS_DATA1, parse_dlc >= 8 ? 32 : (parse_dlc - 4) * 8);
    else
        data_state_go_crc();
}

// Handle reception of bytes 5-8 of data content
void CanFD2040::data_state_update_data1(uint32_t data) noexcept
{
    uint32_t bits = parse_dlc >= 8 ? 32 : (parse_dlc - 4) * 8;
    parse_crc = crc_bytes(parse_crc, data, parse_dlc - 4);
    parse_msg->data32[1] = __builtin_bswap32(data << (32 - bits));
#if SUPPORT_CAN_FD
    if (parse_msg.id & CAN2040_ID_FDF) {
    qq;
    } else {
    	data_state_go_crc();
    }
#else
    data_state_go_crc();
#endif
}

// Handle reception of 16 bits of message CRC (15 crc bits + crc delimiter)
void CanFD2040::data_state_update_crc(uint32_t data) noexcept
{
    if (((parse_crc << 1) | 1) != data)
    {
        data_state_go_discard();
        return;
    }

    unstuf.unstuf_clear_state();
    data_state_go_next(MS_ACK, 2);
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
    unstuf.unstuf_clear_state();
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
    switch (parse_state) {
    case MS_START: data_state_update_start(data); break;
    case MS_HEADER: data_state_update_header(data); break;
    case MS_EXT_HEADER: data_state_update_ext_header(data); break;
    case MS_DATA0: data_state_update_data0(data); break;
    case MS_DATA1: data_state_update_data1(data); break;
#if SUPPORT_CAN_FD
    case MS_DLC: data_state_update_dlc(data); break;
#endif
    case MS_CRC: data_state_update_crc(data); break;
    case MS_ACK: data_state_update_ack(data); break;
    case MS_EOF0: data_state_update_eof0(data); break;
    case MS_EOF1: data_state_update_eof1(data); break;
    case MS_DISCARD: data_state_update_discard(data); break;
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
    if (tx_queue == nullptr)
    {
        // No new messages to transmit
        tx_state = TS_IDLE;
        return;
    }
    tx_state = TS_QUEUED;
    pio_tx_send(tx_queue->stuffed_data, tx_queue->stuffed_words);
}

// Setup PIO state for ack injection
int CanFD2040::tx_inject_ack(uint32_t match_key) noexcept
{
    if (tx_state == TS_QUEUED && !pio_tx_did_conflict()
        && pio_rx_fifo_level() > 1)
    {
        // Rx state is behind - acking wont succeed and may halt active tx
        return -1;
    }
    tx_state = TS_ACKING_RX;
    pio_tx_inject_ack(match_key);
    return 0;
}

// Check if the current parsed message is feedback from current transmit
bool CanFD2040::tx_check_local_message() noexcept
{
    if (tx_state != TS_QUEUED)
    {
        return false;
    }
    CanFD2040TransmitBuffer *qt = tx_queue;
    if (qt->crc == parse_crc && qt->id == parse_msg->id && qt->dlc == parse_dlc)		// ideally we would check the data here too
    {
        // Received our own transmission
        tx_state = TS_CONFIRM_TX;
        return true;
    }
    return false;
}

// End
