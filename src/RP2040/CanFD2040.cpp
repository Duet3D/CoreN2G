/*
 * CanFD2040.cpp
 *
 *  Created on: 22 Aug 2022
 *      Author: David Crocker
 *
 * This is the low-level driver for partial ISO CAN-FD support on the RP2040.
 * It is derived from Kevin Connor's CAN 2.0 implementation for the RP2040, see https://github.com/KevinOConnor/can2040
 * In particular, the PIO code is from that project and the low-level functions are derived from it.
 *
 * IMPORTANT! Robert Bosch GmbH owns many relevant patents and requires a license fee to be paid for any commercial use of CAN-FD.
 *
 * Software license: GNU GENERAL PUBLIC LICENSE Version 3
 *
 * Features of this driver:
 * - Supports CAN-FD frames up to the maximum 64 bytes long
 * - It has been tested at 1Mbit/second
 *
 * Limitations of this driver
 * - It supports CAN-FD frames only. CAN 2.0 frames seen by this node will be discarded. Supporting CAN 2.0 frames too would
 *   add extra complexity because of the different CRC scheme, and is not needed by RepRapFirmware.
 * - It does not support bit rate switching. CAN-FD frames with the BRS bit set will be discarded.
 * - It dedicates the entire second processor core and one PIO to CAN. It would be possible to
 *   have the second core perform other operations, for example by performing them in the main loop in function Entry,
 *   however interrupt latency on the second core is critical for CAN to function correctly
 *
 * This driver is intended to work with the RP2040 Core 0 processor running FreeRTOS and using the code in file CanDevice RP2040.cpp
 * to communicate with this driver. Most of the communication is via a shared memory block defined in file VirtualCanRegisters.h.
 * When CAN is running, each field in that memory block is only ever written by one of the cores. Additionally, Core 0 uses
 * the inter-core fifo to signal to Core 0 that something important has happened, e.g. a message has been received. That
 * signal can be used to wake up a waiting FreeRTOS task.
 *
 * The shared memory block includes two receive fifos and one transmit fifo. Messages are transmitted strictly in FIFO order.
 * Received messages are filtered by ID and put into one of the two receive fifos or discarded. It would be easy to add
 * additional receive fifos.
 */

#include "CanFD2040.h"

#define PICO_MUTEX_ENABLE_SDK120_COMPATIBILITY	0	// used by mutex.h which is included by multicore.h
#include <hardware/structs/dma.h>					// dma_hw
#include <hardware/structs/resets.h>				// for RESETS_RESET_PIO0_BITS
#include <hardware/regs/dreq.h>
#include <hardware/dma.h>
#include <pico/multicore.h>
#include <RP2040.h>
#include "PIOassignments.h"

#include <cstring>

// Define the DMA channel used by this driver. RP2040 configurations of client projects must avoid using this channel.
constexpr DmaChannel DmacChanCAN = 8;				// which DMA channel we use
constexpr DmaPriority DmacPrioCAN = 1;				// RP2040 has two DMA priorities, 0 and 1

extern "C" void debugPrintf(const char *fmt, ...) noexcept;	// for debugging

extern "C" void PIO_isr() noexcept;					// forward declaration

// The stack for Core 1 lives in the top 2kb of the SCRATCH_X area of RAM. We can use the lower 2kb for time-critical code or data used by Core 1.
// When we build the SDK ourselves, we will be able to reduce the stack size to make more room.
#if 0
#define CORE1_CRITICAL_CODE(_name)				_name
#define CORE1_CRITICAL_MEMBER(_class, _name)	_class::_name
#define CORE1_CRITICAL_DATA_RO(_name)			_name
#define CORE1_CRITICAL_DATA_RW(_name)			_name
#else
#define CORE1_CRITICAL_CODE(_name)				__attribute__((section(".scratch_x." #_name))) _name
#define CORE1_CRITICAL_MEMBER(_class, _name)	__attribute__((section(".scratch_x." #_class "_" #_name))) _class::_name
#define CORE1_CRITICAL_DATA_RO(_name)			__attribute__((section(".scratch_x." #_name))) _name
#define CORE1_CRITICAL_DATA_RW(_name)			__attribute__((section(".scratch_x." #_name))) _name
#endif
// Place critical code and data in RAM
#if 0
#define CRITICAL_CODE(_name)			_name
#define CRITICAL_MEMBER(_class, _name)	_class::_name
#define CRITICAL_DATA_RO(_name)			_name
#define CRITICAL_DATA_RW(_name)			_name
#else
#define CRITICAL_CODE(_name)			__attribute__((section(".time_critical." #_name))) _name
#define CRITICAL_MEMBER(_class, _name)	__attribute__((section(".time_critical." #_class "_" #_name))) _class::_name
#define CRITICAL_DATA_RO(_name)			__attribute__((section(".time_critical." #_name))) _name
#define CRITICAL_DATA_RW(_name)			__attribute__((section(".time_critical." #_name))) _name
#endif

// Macro to align a function on a cache line. The XIP cache has 8 bytes per line, so  starting functions on 8-byte boundaries may be fastest.
// For this to work, the start of the text segment must be 8-byte aligned in the linker script.
#define ALIGNED_FUNCTION	__attribute__((aligned(8)))

/****************************************************************
 * rp2040 and low-level helper functions
 ****************************************************************/

// Helper compiler definitions
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

#pragma GCC optimize ("O3")

// rp2040 helper function to clear a hardware reset bit
static void rp2040_clear_reset(uint32_t reset_bit)
{
    if (resets_hw->reset & reset_bit)
    {
        hw_clear_bits(&resets_hw->reset, reset_bit);
        while (!(resets_hw->reset_done & reset_bit)) { }
    }
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

// 3-bit Gray code table, shifted left 1 bit with lower even parity bit added
constexpr uint8_t CRITICAL_DATA_RO(GrayTable)[] = { 0b0000, 0b0011, 0b0110, 0b0101, 0b1100, 0b1111, 0b1010, 0b1001 };

/****************************************************************
 * CRC calculation
 ****************************************************************/

static constexpr uint32_t CRITICAL_DATA_RO(crc17Table)[] =
{
	0x00000000, 0xB42D8000, 0xDC768000, 0x685B0000, 0x0CC08000, 0xB8ED0000, 0xD0B60000, 0x649B8000,
	0x19810000, 0xADAC8000, 0xC5F78000, 0x71DA0000, 0x15418000, 0xA16C0000, 0xC9370000, 0x7D1A8000,
	0x33020000, 0x872F8000, 0xEF748000, 0x5B590000, 0x3FC28000, 0x8BEF0000, 0xE3B40000, 0x57998000,
	0x2A830000, 0x9EAE8000, 0xF6F58000, 0x42D80000, 0x26438000, 0x926E0000, 0xFA350000, 0x4E188000,
	0x66040000, 0xD2298000, 0xBA728000, 0x0E5F0000, 0x6AC48000, 0xDEE90000, 0xB6B20000, 0x029F8000,
	0x7F850000, 0xCBA88000, 0xA3F38000, 0x17DE0000, 0x73458000, 0xC7680000, 0xAF330000, 0x1B1E8000,
	0x55060000, 0xE12B8000, 0x89708000, 0x3D5D0000, 0x59C68000, 0xEDEB0000, 0x85B00000, 0x319D8000,
	0x4C870000, 0xF8AA8000, 0x90F18000, 0x24DC0000, 0x40478000, 0xF46A0000, 0x9C310000, 0x281C8000,
	0xCC080000, 0x78258000, 0x107E8000, 0xA4530000, 0xC0C88000, 0x74E50000, 0x1CBE0000, 0xA8938000,
	0xD5890000, 0x61A48000, 0x09FF8000, 0xBDD20000, 0xD9498000, 0x6D640000, 0x053F0000, 0xB1128000,
	0xFF0A0000, 0x4B278000, 0x237C8000, 0x97510000, 0xF3CA8000, 0x47E70000, 0x2FBC0000, 0x9B918000,
	0xE68B0000, 0x52A68000, 0x3AFD8000, 0x8ED00000, 0xEA4B8000, 0x5E660000, 0x363D0000, 0x82108000,
	0xAA0C0000, 0x1E218000, 0x767A8000, 0xC2570000, 0xA6CC8000, 0x12E10000, 0x7ABA0000, 0xCE978000,
	0xB38D0000, 0x07A08000, 0x6FFB8000, 0xDBD60000, 0xBF4D8000, 0x0B600000, 0x633B0000, 0xD7168000,
	0x990E0000, 0x2D238000, 0x45788000, 0xF1550000, 0x95CE8000, 0x21E30000, 0x49B80000, 0xFD958000,
	0x808F0000, 0x34A28000, 0x5CF98000, 0xE8D40000, 0x8C4F8000, 0x38620000, 0x50390000, 0xE4148000,
	0x2C3D8000, 0x98100000, 0xF04B0000, 0x44668000, 0x20FD0000, 0x94D08000, 0xFC8B8000, 0x48A60000,
	0x35BC8000, 0x81910000, 0xE9CA0000, 0x5DE78000, 0x397C0000, 0x8D518000, 0xE50A8000, 0x51270000,
	0x1F3F8000, 0xAB120000, 0xC3490000, 0x77648000, 0x13FF0000, 0xA7D28000, 0xCF898000, 0x7BA40000,
	0x06BE8000, 0xB2930000, 0xDAC80000, 0x6EE58000, 0x0A7E0000, 0xBE538000, 0xD6088000, 0x62250000,
	0x4A398000, 0xFE140000, 0x964F0000, 0x22628000, 0x46F90000, 0xF2D48000, 0x9A8F8000, 0x2EA20000,
	0x53B88000, 0xE7950000, 0x8FCE0000, 0x3BE38000, 0x5F780000, 0xEB558000, 0x830E8000, 0x37230000,
	0x793B8000, 0xCD160000, 0xA54D0000, 0x11608000, 0x75FB0000, 0xC1D68000, 0xA98D8000, 0x1DA00000,
	0x60BA8000, 0xD4970000, 0xBCCC0000, 0x08E18000, 0x6C7A0000, 0xD8578000, 0xB00C8000, 0x04210000,
	0xE0358000, 0x54180000, 0x3C430000, 0x886E8000, 0xECF50000, 0x58D88000, 0x30838000, 0x84AE0000,
	0xF9B48000, 0x4D990000, 0x25C20000, 0x91EF8000, 0xF5740000, 0x41598000, 0x29028000, 0x9D2F0000,
	0xD3378000, 0x671A0000, 0x0F410000, 0xBB6C8000, 0xDFF70000, 0x6BDA8000, 0x03818000, 0xB7AC0000,
	0xCAB68000, 0x7E9B0000, 0x16C00000, 0xA2ED8000, 0xC6760000, 0x725B8000, 0x1A008000, 0xAE2D0000,
	0x86318000, 0x321C0000, 0x5A470000, 0xEE6A8000, 0x8AF10000, 0x3EDC8000, 0x56878000, 0xE2AA0000,
	0x9FB08000, 0x2B9D0000, 0x43C60000, 0xF7EB8000, 0x93700000, 0x275D8000, 0x4F068000, 0xFB2B0000,
	0xB5338000, 0x011E0000, 0x69450000, 0xDD688000, 0xB9F30000, 0x0DDE8000, 0x65858000, 0xD1A80000,
	0xACB28000, 0x189F0000, 0x70C40000, 0xC4E98000, 0xA0720000, 0x145F8000, 0x7C048000, 0xC8290000
};

static constexpr uint32_t CRITICAL_DATA_RO(crc21Table)[] =
{
	0x00000000, 0x8144C800, 0x83CD5800, 0x02899000, 0x86DE7800, 0x079AB000, 0x05132000, 0x8457E800,
	0x8CF83800, 0x0DBCF000, 0x0F356000, 0x8E71A800, 0x0A264000, 0x8B628800, 0x89EB1800, 0x08AFD000,
	0x98B4B800, 0x19F07000, 0x1B79E000, 0x9A3D2800, 0x1E6AC000, 0x9F2E0800, 0x9DA79800, 0x1CE35000,
	0x144C8000, 0x95084800, 0x9781D800, 0x16C51000, 0x9292F800, 0x13D63000, 0x115FA000, 0x901B6800,
	0xB02DB800, 0x31697000, 0x33E0E000, 0xB2A42800, 0x36F3C000, 0xB7B70800, 0xB53E9800, 0x347A5000,
	0x3CD58000, 0xBD914800, 0xBF18D800, 0x3E5C1000, 0xBA0BF800, 0x3B4F3000, 0x39C6A000, 0xB8826800,
	0x28990000, 0xA9DDC800, 0xAB545800, 0x2A109000, 0xAE477800, 0x2F03B000, 0x2D8A2000, 0xACCEE800,
	0xA4613800, 0x2525F000, 0x27AC6000, 0xA6E8A800, 0x22BF4000, 0xA3FB8800, 0xA1721800, 0x2036D000,
	0xE11FB800, 0x605B7000, 0x62D2E000, 0xE3962800, 0x67C1C000, 0xE6850800, 0xE40C9800, 0x65485000,
	0x6DE78000, 0xECA34800, 0xEE2AD800, 0x6F6E1000, 0xEB39F800, 0x6A7D3000, 0x68F4A000, 0xE9B06800,
	0x79AB0000, 0xF8EFC800, 0xFA665800, 0x7B229000, 0xFF757800, 0x7E31B000, 0x7CB82000, 0xFDFCE800,
	0xF5533800, 0x7417F000, 0x769E6000, 0xF7DAA800, 0x738D4000, 0xF2C98800, 0xF0401800, 0x7104D000,
	0x51320000, 0xD076C800, 0xD2FF5800, 0x53BB9000, 0xD7EC7800, 0x56A8B000, 0x54212000, 0xD565E800,
	0xDDCA3800, 0x5C8EF000, 0x5E076000, 0xDF43A800, 0x5B144000, 0xDA508800, 0xD8D91800, 0x599DD000,
	0xC986B800, 0x48C27000, 0x4A4BE000, 0xCB0F2800, 0x4F58C000, 0xCE1C0800, 0xCC959800, 0x4DD15000,
	0x457E8000, 0xC43A4800, 0xC6B3D800, 0x47F71000, 0xC3A0F800, 0x42E43000, 0x406DA000, 0xC1296800,
	0x437BB800, 0xC23F7000, 0xC0B6E000, 0x41F22800, 0xC5A5C000, 0x44E10800, 0x46689800, 0xC72C5000,
	0xCF838000, 0x4EC74800, 0x4C4ED800, 0xCD0A1000, 0x495DF800, 0xC8193000, 0xCA90A000, 0x4BD46800,
	0xDBCF0000, 0x5A8BC800, 0x58025800, 0xD9469000, 0x5D117800, 0xDC55B000, 0xDEDC2000, 0x5F98E800,
	0x57373800, 0xD673F000, 0xD4FA6000, 0x55BEA800, 0xD1E94000, 0x50AD8800, 0x52241800, 0xD360D000,
	0xF3560000, 0x7212C800, 0x709B5800, 0xF1DF9000, 0x75887800, 0xF4CCB000, 0xF6452000, 0x7701E800,
	0x7FAE3800, 0xFEEAF000, 0xFC636000, 0x7D27A800, 0xF9704000, 0x78348800, 0x7ABD1800, 0xFBF9D000,
	0x6BE2B800, 0xEAA67000, 0xE82FE000, 0x696B2800, 0xED3CC000, 0x6C780800, 0x6EF19800, 0xEFB55000,
	0xE71A8000, 0x665E4800, 0x64D7D800, 0xE5931000, 0x61C4F800, 0xE0803000, 0xE209A000, 0x634D6800,
	0xA2640000, 0x2320C800, 0x21A95800, 0xA0ED9000, 0x24BA7800, 0xA5FEB000, 0xA7772000, 0x2633E800,
	0x2E9C3800, 0xAFD8F000, 0xAD516000, 0x2C15A800, 0xA8424000, 0x29068800, 0x2B8F1800, 0xAACBD000,
	0x3AD0B800, 0xBB947000, 0xB91DE000, 0x38592800, 0xBC0EC000, 0x3D4A0800, 0x3FC39800, 0xBE875000,
	0xB6288000, 0x376C4800, 0x35E5D800, 0xB4A11000, 0x30F6F800, 0xB1B23000, 0xB33BA000, 0x327F6800,
	0x1249B800, 0x930D7000, 0x9184E000, 0x10C02800, 0x9497C000, 0x15D30800, 0x175A9800, 0x961E5000,
	0x9EB18000, 0x1FF54800, 0x1D7CD800, 0x9C381000, 0x186FF800, 0x992B3000, 0x9BA2A000, 0x1AE66800,
	0x8AFD0000, 0x0BB9C800, 0x09305800, 0x88749000, 0x0C237800, 0x8D67B000, 0x8FEE2000, 0x0EAAE800,
	0x06053800, 0x8741F000, 0x85C86000, 0x048CA800, 0x80DB4000, 0x019F8800, 0x03161800, 0x8252D000
};

// CRC polynomials used, including the MSB
constexpr uint32_t crc17polynomial = 0x0003685B;	// 0b0000'0000'0000'0011'0110'1000'0101'0011 including the leading 1
constexpr uint32_t crc17initialValue = 1u << 31;	// initial value after left shifting
constexpr uint32_t crc21polynomial = 0x00302899; 	// 0b0000'0000'0011'0000'0010'1000'1001'1001;
constexpr uint32_t crc21initialValue = 1u << 31;	// initial value after left shifting

// Calculate CRC17 on a number of bits between 1 and 32 (must not be called with 0 bits!). The bits are left-justified and any lower-order bits are zero.
uint32_t CRITICAL_CODE(Crc17Bits)(uint32_t remainder, uint32_t data, unsigned int numBits) noexcept
{
	remainder ^= data;
	do
	{
		--numBits;
        const uint32_t mask = (remainder & 0x8000'0000) ? 0xFFFFFFFF : 0;
        remainder = (remainder << 1) ^ (mask & (crc17polynomial << (32 - 17)));
	} while (numBits != 0);
	return remainder;
}

// Calculate CRC17 on a number of 32-bit words
uint32_t CRITICAL_CODE(Crc17Words)(const uint32_t *buf, unsigned int numWords) noexcept
{
	uint32_t r17 = crc17initialValue;
	while (numWords != 0)
	{
		--numWords;
		uint32_t data = *buf++;
		r17 = (r17 << 8) ^ crc17Table[(r17 ^ data) >> 24];
		data <<= 8;
		r17 = (r17 << 8) ^ crc17Table[(r17 ^ data) >> 24];
		data <<= 8;
		r17 = (r17 << 8) ^ crc17Table[(r17 ^ data) >> 24];
		data <<= 8;
		r17 = (r17 << 8) ^ crc17Table[(r17 ^ data) >> 24];
	}
	return r17;
}

// Calculate CRC21 on a number of bits between 1 and 32. (must not be called with 0 bits!). The bits are left-justified and any lower-order bits are zero.
uint32_t CRITICAL_CODE(Crc21Bits)(uint32_t remainder, uint32_t data, unsigned int numBits) noexcept
{
	remainder ^= data;
	do
	{
		--numBits;
        const uint32_t mask = (remainder & 0x8000'0000) ? 0xFFFFFFFF : 0;
        remainder = (remainder << 1) ^ (mask & (crc21polynomial << (32 - 21)));
	} while (numBits != 0);
	return remainder;
}

// Calculate CRC21 on a number of words
uint32_t CRITICAL_CODE(Crc21Words)(const uint32_t *buf, unsigned int numWords) noexcept
{
	uint32_t r21 = crc21initialValue;
	while (numWords != 0)
	{
		--numWords;
		uint32_t data = *buf++;
		r21 = (r21 << 8) ^ crc21Table[(r21 ^ data) >> 24];
		data <<= 8;
		r21 = (r21 << 8) ^ crc21Table[(r21 ^ data) >> 24];
		data <<= 8;
		r21 = (r21 << 8) ^ crc21Table[(r21 ^ data) >> 24];
		data <<= 8;
		r21 = (r21 << 8) ^ crc21Table[(r21 ^ data) >> 24];
	}
	return r21;
}

/****************************************************************
 * Bit unstuffing
 ****************************************************************/

// Update the CRCs with between 1 and 32 bits (0 is not allowed). The data is right-justified and any higher bits in it are ignored.
ALIGNED_FUNCTION void CRITICAL_MEMBER(BitUnstuffer, AddCrcBits)(uint32_t data, unsigned int numBits) noexcept
{
	data &= (1u << numBits) - 1;					// clear any higher order bits
	uint32_t leftJustifiedData;
	unsigned int bitsThisTime;
	const unsigned int totalBits = numBits + numBitsLeftOver;
	if (totalBits > 32)
	{
		bitsThisTime = 32;
		leftJustifiedData = bitsLeftOver << (32 - numBitsLeftOver);
		numBitsLeftOver = numBits - 32;
		leftJustifiedData |= (data >> numBitsLeftOver);
		bitsLeftOver = data & ((1u << numBitsLeftOver) - 1);
	}
	else
	{
		bitsThisTime = totalBits & (~7);
		if (bitsThisTime == 0)
		{
			bitsLeftOver = (bitsLeftOver << numBits) | data;
			numBitsLeftOver = totalBits;
			return;
		}
		leftJustifiedData = bitsLeftOver << (32 - numBitsLeftOver);
		leftJustifiedData |= data << (32 - totalBits);		// a few extraneous bits at the bottom don't matter
		numBitsLeftOver = totalBits - bitsThisTime;
		bitsLeftOver = data & ((1u << numBitsLeftOver) - 1);
	}
	uint32_t r17 = crc17;
	uint32_t r21 = crc21;

	// Use table driven byte-at-a-time CRC for the first 8n bits
	while (bitsThisTime >= 8)
	{
		bitsThisTime -= 8;
		r17 = (r17 << 8) ^ crc17Table[(r17 ^ leftJustifiedData) >> 24];
		r21 = (r21 << 8) ^ crc21Table[(r21 ^ leftJustifiedData) >> 24];
		leftJustifiedData <<= 8;
	}
	crc17 = r17;
	crc21 = r21;
}

// Get the CRC17, right justified
uint32_t CRITICAL_MEMBER(BitUnstuffer, GetCrc17)() const noexcept
{
	uint32_t r17 = crc17;
	unsigned int numBits = numBitsLeftOver;
	if (numBits != 0)
	{
		r17 ^= bitsLeftOver << (32 - numBits);
		do
		{
			--numBits;
			// This code compiles the loop body to fewer instructions than using an if-statement does, and saves a conditional jump
			const uint32_t mask17 = (r17 & 0x8000'0000) ? 0xFFFFFFFF : 0;
			r17 = (r17 << 1) ^ (mask17 & (crc17polynomial << (32 - 17)));
		} while (numBits != 0);

	}
	return r17 >> (32 - 17);
}

// Get the CRC21, right justified
uint32_t CRITICAL_MEMBER(BitUnstuffer, GetCrc21)() const noexcept
{
	uint32_t r21 = crc21;
	unsigned int numBits = numBitsLeftOver;
	if (numBits != 0)
	{
		r21 ^= bitsLeftOver << (32 - numBits);
		do
		{
			--numBits;
			// This code compiles the loop body to fewer instructions than using an if-statement does, and saves a conditional jump
			const uint32_t mask21 = (r21 & 0x8000'0000) ? 0xFFFFFFFF : 0;
			r21 = (r21 << 1) ^ (mask21 & (crc21polynomial << (32 - 21)));
		} while (numBits != 0);

	}
	return r21 >> (32 - 21);
}

// Initialise the CRCs with 0 to 32 bits of data
inline void CRITICAL_MEMBER(BitUnstuffer, InitCrc)(uint32_t data, unsigned int numBits) noexcept
{
	crc17 = crc21 = 1u << 31;							// both CRCs start with 1 in the MSB
	numBitsLeftOver = 0;
	bitsLeftOver = 0;
	if (numBits != 0)
	{
		AddCrcBits(data, numBits);
	}
}

// Add 'count' number of bits from 'data' to the unstuffer
// The bits are shifted left into stuffed_bits, and count_stuff is set to the number of bits added
void CRITICAL_MEMBER(BitUnstuffer, AddBits)(uint32_t data, uint32_t count) noexcept
{
    const uint32_t mask = (1u << count) - 1;
    stuffed_bits = (stuffed_bits << count) | (data & mask);
    stuffedBitsAvailable = count;
}

// Reset state and set the next desired 'count' unstuffed bits to extract
void CRITICAL_MEMBER(BitUnstuffer, SetCount)(uint32_t count) noexcept
{
    unstuffed_bits = 0;
    unstuffedBitsWanted = count;
}

// Update the stuffed bits history so that it looks like we had just one recessive bit before any remaining unprocessed bits
void CRITICAL_MEMBER(BitUnstuffer, ClearState)() noexcept
{
	const uint32_t prevBit = 1u << stuffedBitsAvailable;
	stuffed_bits = (stuffed_bits & (prevBit - 1u)) | prevBit;
}

// Pull bits from unstuffer (as specified in unstuf_set_count())
// Return 0 if successful, 1 if need more data, -1 if six recessive bits detected, or -2 if 6 dominant bits detected
int CRITICAL_MEMBER(BitUnstuffer, PullBits)() noexcept
{
    const uint32_t sb = stuffed_bits;
	uint32_t cs = stuffedBitsAvailable;
	uint32_t cu = unstuffedBitsWanted;

	switch (stuffType)
	{
	case StuffingType::none:		// receiving ACK or EOF, so using no stuff bits
	default:
		{
			const uint32_t toExtract = min<uint32_t>(cu, cs);
			unstuffed_bits = (unstuffed_bits << toExtract)
							| ((stuffed_bits >> (cs - toExtract)) & ((1u << toExtract) - 1u));
			cs -= toExtract;
			cu -= toExtract;
			stuffedBitsAvailable = cs;
			unstuffedBitsWanted = cu;
			return (cu == 0) ? 0 : 1;
		}

	case StuffingType::fixed:		// receiving stuff count or CRC, so using fixed stuff bits
		{
			// When using fixed stuff bits we do not update the CRC
			uint32_t bu = bitsUntilFixedStuffBit;
			while (cu != 0 && cs != 0)
			{
				if (bu == 0)
				{
					// Check that this is the inverse of the previous bit
					if (unlikely(((sb ^ (sb >> 1)) & (1u << (cs - 1))) == 0))
					{
						return ((sb >> cs) & 1u) ? -1 : -2;
					}
					--cs;
					bu = 4;
					if (cs == 0)
					{
						break;
					}
				}
				const uint32_t toExtract = min<uint32_t>(cu, min<uint32_t>(cs, bu));
				unstuffed_bits = (unstuffed_bits << toExtract)
								| ((stuffed_bits >> (cs - toExtract)) & ((1u << toExtract) - 1u));
				cs -= toExtract;
				cu -= toExtract;
				bu -= toExtract;
			}
			stuffedBitsAvailable = cs;
			unstuffedBitsWanted = cu;
			bitsUntilFixedStuffBit = bu;
			return (cu == 0) ? 0 : 1;
		}

	case StuffingType::dynamic:		// receiving header or data, so using variable stuff bits. In CAN-FD the stuff bits are included in the CRC.
		{
			if (cs == 0)
			{
				// Need more data
				return 1;
			}

			const uint32_t edges = sb ^ (sb >> 1);
			const uint32_t e2 = edges | (edges >> 1);
			const uint32_t e4 = e2 | (e2 >> 2);
			const uint32_t rm_bits = ~e4;

			while (true)
			{
				uint32_t try_cnt = cs > cu ? cu : cs;
				for (;;)
				{
					// Try_cnt may be zero here, but we must not call the CRC functions with 0 bits of data
					if (likely(try_cnt != 0))
					{
						const uint32_t try_mask = ((1 << try_cnt) - 1) << (cs + 1 - try_cnt);
						if (likely((rm_bits & try_mask) == 0))
						{
							// No stuff bits in try_cnt bits - copy into unstuffed_bits
							unstuffedBitsWanted = cu = cu - try_cnt;
							stuffedBitsAvailable = cs = cs - try_cnt;
							unstuffed_bits |= ((sb >> cs) & ((1u << try_cnt) - 1)) << cu;
							AddCrcBits(sb >> cs, try_cnt);
							if (cu == 0)
							{
								// Extracted desired bits
								return 0;
							}
							break;
						}
					}

					// There must be at least 1 more bit available
					stuffedBitsAvailable = cs = cs - 1;
					AddCrcBits(sb >> cs, 1);
					if (rm_bits & (1u << (cs + 1)))
					{
						// High bit of try_cnt a stuff bit
						++totalStuffBits;
						if (unlikely(rm_bits & (1u << cs)))
						{
							// Six consecutive bits - a bitstuff error
							return ((sb >> cs) & 1u) ? -1 : -2;
						}
						break;
					}
					// High bit not a stuff bit. Copy 1 bit over, limit try_cnt, and retry
					unstuffedBitsWanted = cu = cu - 1;
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
	}
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
	uint32_t GetTotalStuffBits() const noexcept { return totalStuffBits; }
	uint32_t GetTotalBits() const noexcept { return bitpos; }

private:
    uint32_t prev_stuffed;		// the previous 5 (or more) bits we have stored in the buffer, so we can tell whether stuffing is needed when we store more bits
    uint32_t bitpos;			// how many bits we have stored in the buffer already
    uint32_t totalStuffBits;
    uint32_t lastData;			// the last bit stored, used when adding fixed stuff bits
    uint32_t *buf;				// the buffer we are stuffing into
};

// Push 'count' bits of 'data' into stuffer without performing bit stuffing
void BitStuffer::pushraw(uint32_t data, uint32_t count) noexcept
pre(count <= 32)
{
	lastData = data;									// assume we are never asked to push 0 bits
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
pre(count != 0; count <= 26)							// pushraw needs at least 1 bit, and the stuffer may expand 26 bits to 32
{
    data &= (1u << count) - 1;							// clear the high bits
    uint32_t stuf = (prev_stuffed << count) | data;

    // This is a slow algorithm, however the one from can2040.c got the bit stuffing wrong sometimes
    uint32_t newcount = count;
    uint32_t edges = (stuf >> 1) ^ stuf;
    do
    {
    	if ((edges & (0x0f << count)) == 0)
    	{
    		// Need to insert a stuff bit at position 'count'
    		const uint32_t mask = (1u << (count + 1)) - 1;		// a mask for the lower (count+1) bits
    		stuf = (  ((stuf << 1) & ~mask)				// the bits from 'count' upwards, shifted left to make room for the stuff bit
    				| (stuf & mask)						// the bits from 0 to 'count' inclusive
				   )
    			   ^ (1u << count);						// invert the bit at 'count' to give the stuff bit
    		++totalStuffBits;
    		++newcount;
    		if (count <= 4) break;
    		edges = (stuf >> 1) ^ stuf;					// update edges because we inserted a stuff bit
    		count -= 4;									// we can now accept another 4 bits of any polarity without stuffing
    	}
    	else
    	{
    		// The bit at 'count' does not need stuffing
    		--count;
    	}
    } while (count != 0);
    pushraw(stuf, newcount);
    prev_stuffed = stuf;
}

// Pad final word of stuffer with high bits
uint32_t BitStuffer::finalize() noexcept
{
	const uint32_t words = DIV_ROUND_UP(bitpos, 32);
	const uint32_t extra = words * 32 - bitpos;
	buf[words - 1] |= (1u << extra) - 1u;
	return words;
}

// Add a fixed stuff bit, not included in the count of total stuff bits
void BitStuffer::AddFixedStuffBit() noexcept
{
	pushraw((~lastData) & 1u, 1);
}

// CanFD2040 members
// Start CAN-FD running
void CanFD2040::Entry(VirtualCanRegisters *p_regs) noexcept
{
	regs = p_regs;
	dma_channel_claim(DmacChanCAN);
    for (;;)
    {
    	// Disable CAN - set output to recessive
    	pinMode(regs->txPin, OUTPUT_HIGH);

    	// Wait for the signal to enable CAN
    	while (!regs->canEnabled) { }

    	// Clear all pending interrupts
    	for (volatile bool& irq : rxInterruptPending)
    	{
    		irq = false;
    	}
		txFifoNotFullInterruptPending = false;

		// Flag the transmitter as idle and no message prepared
		txStuffedWords = 0;
		tx_state = TS_IDLE;

		// Set up the PIO hardware address
		pio_hw = (CanPioNumber) ? pio1_hw : pio0_hw;

		// Set up the transmit DMA controller as far as possible
		const uint32_t trigSrc = (CanPioNumber) ? DREQ_PIO1_TX3 : DREQ_PIO0_TX3;
		dmaControlWord = (trigSrc << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB)
						| (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB)
						| DMA_CH0_CTRL_TRIG_INCR_READ_BITS
						| ((uint32_t)DmacPrioCAN << DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_LSB);
		dma_channel_hw_t *const dmach_hw = &dma_hw->ch[DmacChanCAN];
		dmach_hw->write_addr = reinterpret_cast<uint32_t>(&pio_hw->txf[3]);
		dmach_hw->al1_ctrl = dmaControlWord;

    	pio_setup();										// set up the PIO and pins (pio_hw has already been set up)
        data_state_go_discard();
        __enable_irq();

    	while (regs->canEnabled)
    	{
    		// If we were asked to clear the error counts, do it
    		if (regs->clearErrorCounts)
    		{
    			regs->clearErrorCounts = false;
    			regs->errors.Clear();
    		}

    		// If we have no pending transmission populate the transmit buffer from the next element in the transmit fifo
    		if (txStuffedWords == 0)
    		{
    			TryPopulateTransmitBuffer();
    		}

    		// If there are any pending interrupt flags, send them to the other processor
    		// This scheme avoids race conditions with the ISR and also avoids having to disable interrupts
   			if ((sio_hw->fifo_st & SIO_FIFO_ST_RDY_BITS) != 0)
   			{
   				uint32_t pendingIrqs = 0;
   				for (unsigned int i = 0; i < NumCanRxFifos; ++i)
   				{
   					if (rxInterruptPending[i])
   					{
   						rxInterruptPending[i] = false;
   						pendingIrqs |= VirtualCanRegisters::recdFifo0 << i;
   					}
   				}
   				if (txFifoNotFullInterruptPending)
   				{
   					txFifoNotFullInterruptPending = false;
   					pendingIrqs |= VirtualCanRegisters::txFifoNotFull;
   				}
   				if (pendingIrqs != 0)
   				{
					sio_hw->fifo_wr = pendingIrqs;
					__sev();
   				}
   			}
    	}
    	__disable_irq();
    }
}

// Set up a message ready to be transmitted
void CanFD2040::TryPopulateTransmitBuffer() noexcept
{
	VirtualCanRegisters::TxFifo& fifo = regs->txFifo;
	uint32_t getIndex = fifo.getIndex;
	if (getIndex != fifo.putIndex)
	{
		const volatile CanTxBuffer *const txBuf = &regs->txFifo.buffers[getIndex];
		txId = txBuf->T0.bit.ID;
		txDlc = txBuf->T1.bit.DLC;

		// Push the header through the stuffer
		BitStuffer bs(txMessage);
		if (txBuf->T0.bit.XTD)
		{
			// Extended header. The maximum number of bits we can push in a single operation is 26 to allow for up to 6 stuff bits.
			const uint32_t h1 = ((txId & 0x1ffc0000) >> 16) | 0x03;
			const uint32_t h2 = txId & 0x3ffff;
			const uint32_t h3 = 0x80 | txDlc;
			bs.push(h1, 14);				// SOF + 11 bits of ID + RRS + ID
			bs.push(h2, 18);				// 18 bits of ID
			bs.push(h3, 9);					// RRS + 8 bits starting at FDF
		}
		else
		{
			// Standard header
			const uint32_t hdr = ((txId & 0x7ff) << 10) | 0x80 | txDlc;
			bs.push(hdr, 22);				// SOF + 11 bits of ID + RRS + IDE + 8 bytes starting at FDF
		}

		// Push the data
		if (txDlc < 8)
		{
			for (size_t i = 0; i < txDlc; i++)
			{
				bs.push(txBuf->data8[i], 8);
			}
		}
		else
		{
			const size_t hwordCount = (txDlc <= 12) ? 2 * (txDlc - 6) : 8 * (txDlc - 11);
			for (size_t i = 0; i < hwordCount; i++)
			{
				bs.push((uint32_t)__builtin_bswap16(txBuf->data16[i]), 16);
			}
		}

		// Push the stuff count. The CRC does not include the fixed stuff bits, so get the CRC up to now first.
		const size_t numWords = bs.GetTotalBits() / 32;

		const unsigned int numBits = bs.GetTotalBits() % 32;
		uint32_t tempCrc;
		if (txDlc > 10)
		{
			tempCrc = Crc21Words(txMessage, numWords);
			if (numBits != 0)
			{
				tempCrc = Crc21Bits(tempCrc, txMessage[numWords], numBits);
			}
		}
		else
		{
			tempCrc = Crc17Words(txMessage, numWords);
			if (numBits != 0)
			{
				tempCrc = Crc17Bits(tempCrc, txMessage[numWords], numBits);
			}
		}

		const uint8_t encodedStuffBits = GrayTable[bs.GetTotalStuffBits() & 7];
		bs.AddFixedStuffBit();
		bs.pushraw(encodedStuffBits, 4);

		// CRC everything stored so far and store the CRC
		if (txDlc > 10)
		{
			txCrc = Crc21Bits(tempCrc, (uint32_t)encodedStuffBits << (32 - 4), 4) >> (32 - 21);
			bs.AddFixedStuffBit();
			bs.pushraw((txCrc >> 17) & 0x0f, 4);
		}
		else
		{
			txCrc = Crc17Bits(tempCrc, (uint32_t)encodedStuffBits << (32 - 4), 4) >> (32 - 17);
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
		bs.pushraw(((txCrc & 0x01) << 1) | 0x01, 2);		// final CRC bit and CRC delimiter

		txStuffedWords = bs.finalize();

		// Wakeup if in TS_IDLE state
		pio_irq_atomic_set_maytx();

		// Now that we have copied the message to our local buffer, we can free up the fifo slot
		++getIndex;
		if (getIndex == fifo.size)
		{
			getIndex = 0;
		}
		fifo.getIndex = getIndex;

		if (regs->txFifoNotFullInterruptEnabled)
		{
			txFifoNotFullInterruptPending = true;
		}
	}
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
int CRITICAL_MEMBER(CanFD2040, pio_rx_check_stall)() noexcept
{
    return pio_hw->fdebug & (1 << (PIO_FDEBUG_RXSTALL_LSB + 1));
}

// Report number of bytes still pending in PIO "rx" fifo queue
int CRITICAL_MEMBER(CanFD2040, pio_rx_fifo_level)() noexcept
{
    return (pio_hw->flevel & PIO_FLEVEL_RX1_BITS) >> PIO_FLEVEL_RX1_LSB;
}

// Set PIO "match" state machine to raise a "matched" signal on a bit sequence
void CRITICAL_MEMBER(CanFD2040, pio_match_check)(uint32_t match_key) noexcept
{
    pio_hw->txf[2] = match_key;
}

// Calculate pos+bits identifier for PIO "match" state machine
/*static*/ uint32_t CRITICAL_MEMBER(CanFD2040, pio_match_calc_key)(uint32_t raw_bits, uint32_t rx_bit_pos) noexcept
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
    pio_hw->ctrl = ((0x07 << PIO_CTRL_SM_ENABLE_LSB)				// disable the state machine
                    | (0x08 << PIO_CTRL_SM_RESTART_LSB));
    struct pio_sm_hw *sm = &pio_hw->sm[3];
    sm->instr = 0xe001;												// set output recessive in case we aborted a transmission on a dominant bit
    pio_hw->irq = (1u << 2) | (1u << 3);							// clear "matched" and "ack done" signals

    dma_channel_hw_t *const dmach_hw = &dma_hw->ch[DmacChanCAN];
    dmach_hw->al1_ctrl = dmaControlWord;							// pause DMA to the FIFO to stop it being refilled
    dma_hw->abort = 1u << DmacChanCAN;								// abort any pending transfers (can occur if a transmission was aborted due to a bus conflict)
    while (dma_hw->abort & (1u << DmacChanCAN)) { }					// wait for in-flight transfers to complete

    // Clear tx fifo
    sm->shiftctrl = 0;												// changing the value of PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS flushes the fifo
    sm->shiftctrl = (PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS
                     | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS);
    // Must reset again after clearing fifo
    pio_hw->ctrl = ((0x07 << PIO_CTRL_SM_ENABLE_LSB)
                    | (0x08 << PIO_CTRL_SM_RESTART_LSB));			// enable and restart the state machine
}

// Queue a message for transmission on PIO "tx" state machine
void CanFD2040::pio_tx_send(uint32_t *data, uint32_t count) noexcept
{
    pio_tx_reset();													// this also disables DMA
    pio_hw->instr_mem[can2040_offset_tx_got_recessive] = 0xa242;	// nop [2]

    if (count <= 8)
    {
    	// The entire message will fit in the transmit fifo
		for (unsigned int i = 0; i < count; i++)
		{
			pio_hw->txf[3] = data[i];
		}
    }
    else
    {
    	// The message is bigger than the fifo so send it using DMA
    	pio_hw->txf[3] = data[0];									// put the first word in the fifo in case the DMA is slow to start
        dma_channel_hw_t *const dmach_hw = &dma_hw->ch[DmacChanCAN];
    	dmach_hw->read_addr = reinterpret_cast<uint32_t>(data + 1);
    	dmach_hw->transfer_count = count - 1;
		dmach_hw->ctrl_trig = dmaControlWord | DMA_CH0_CTRL_TRIG_EN_BITS;
    }

    pio_sm_hw *const sm = &pio_hw->sm[3];
    sm->instr = 0xe001; // set pins, 1
    sm->instr = can2040_offset_tx_start; // jmp tx_start
    sm->instr = 0x20c0; // wait 1 irq, 0
    pio_hw->ctrl = 0x0f << PIO_CTRL_SM_ENABLE_LSB;
}

// Set PIO "tx" state machine to inject an ack after a CRC match
void CRITICAL_MEMBER(CanFD2040, pio_tx_inject_ack)(uint32_t match_key) noexcept
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
void CRITICAL_MEMBER(CanFD2040, pio_irq_set_maytx)() noexcept
{
    pio_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS;
}

// Enable host irq on a "may transmit" or "matched" signal (sm irq 0 or 2)
void CRITICAL_MEMBER(CanFD2040, pio_irq_set_maytx_matched)() noexcept
{
    pio_hw->inte0 = (PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM2_BITS
                     | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS);
}

// Enable host irq on a "may transmit" or "ack done" signal (sm irq 0 or 3)
void CRITICAL_MEMBER(CanFD2040, pio_irq_set_maytx_ackdone)() noexcept
{
    pio_hw->inte0 = (PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM3_BITS
                     | PIO_IRQ0_INTE_SM1_RXNEMPTY_BITS);
}

// Atomically enable "may transmit" signal (sm irq 0)
void CRITICAL_MEMBER(CanFD2040, pio_irq_atomic_set_maytx)() noexcept
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

void CanFD2040::pio_setup() noexcept
{
    // Configure PIO clock
    const uint32_t rb = (CanPioNumber) ? RESETS_RESET_PIO1_BITS : RESETS_RESET_PIO0_BITS;
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
	const GpioPinFunction pioFunc = (CanPioNumber) ? GpioPinFunction::Pio1 : GpioPinFunction::Pio0;
	SetPinFunction(regs->rxPin, pioFunc);
	SetPinFunction(regs->txPin, pioFunc);

    const IRQn_Type irqn = (CanPioNumber) ? PIO1_IRQ_0_IRQn : PIO0_IRQ_0_IRQn;
	NVIC_DisableIRQ(irqn);
	NVIC_ClearPendingIRQ(irqn);
	NVIC_SetPriority(irqn, 1);
    irq_set_exclusive_handler ((unsigned int)irqn, PIO_isr);
	NVIC_EnableIRQ(irqn);
}

// Report error to calling code (via callback interface)
void CanFD2040::report_callback_error(uint32_t error_code) noexcept
{
    //TODO
}

// Report a received message to calling code (via callback interface)
void CanFD2040::report_callback_rx_msg() noexcept
{
	if (rxFifoNumber < NumCanRxFifos)
	{
		VirtualCanRegisters::RxFifo& fifo = regs->rxFifos[rxFifoNumber];
		unsigned int putPointer = fifo.putIndex;
		CanRxBuffer& rxBuffer = const_cast<CanRxBuffer&>(fifo.buffers[putPointer]);
		rxBuffer.R0.bit.ID = parse_id;
		rxBuffer.R1.bit.DLC = parse_dlc;
		++putPointer;
		if (putPointer == fifo.size)
		{
			putPointer = 0;
		}
		fifo.putIndex = putPointer;
		rxInterruptPending[rxFifoNumber] = true;
	}
}

// Report a message that was successfully transmitted
void CanFD2040::report_callback_tx_msg() noexcept
{
	regs->cancelTransmission = false;					// in case we were just asked to cancel the transmission that we have just completed
	txStuffedWords = 0;									// this allows the main loop to populate the buffer with a new message
}

// EOF phase complete - report message (rx or tx) to calling code
ALIGNED_FUNCTION void CRITICAL_MEMBER(CanFD2040, report_handle_eof)() noexcept
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
int CRITICAL_MEMBER(CanFD2040, report_is_acking_rx)() noexcept
{
    return report_state == (ReportState)(ReportState::RS_IN_MSG | ReportState::RS_AWAIT_EOF);
}

// Parser found a new message start
void CRITICAL_MEMBER(CanFD2040, report_note_message_start)() noexcept
{
    pio_irq_set_maytx();
}

// Setup for ack injection (if receiving) or ack confirmation (if transmit)
void CRITICAL_MEMBER(CanFD2040, report_note_crc_start)() noexcept
{
	// Get 21 (more is OK) bits of expected data (lower 16 bits of CRC plus 4 fixed stuff bits plus CRC delimiter)
#if 1
	// GCC generates more efficient code for this than for either of the following
	uint32_t expectedData =   ((parse_crc & 0xe000) << 4)
							| ((parse_crc & 0x1e00) << 3)
							| ((parse_crc & 0x01e0) << 2)
							| ((parse_crc & 0x001e) << 1)
							|  (parse_crc & 0x0001);										// insert gaps for stuff bits
	expectedData = (expectedData << 1)														// make room for CRC delimiter
					| ((~expectedData) & ((1u << 17) | (1u << 12) | (1u << 7) | (1u << 2)))	// insert stuff bits
					| 0x0001;																// add CRC delimiter
#elif 1
	uint32_t expectedData = (parse_crc & 0x1e000);											// bits 13-15 of CRC
	expectedData = (expectedData << 1) | (~parse_crc & 0x2000) | (parse_crc & 0x1e00);		// fixed stuff bit (inverse of bit 13) and bits 9-12
	expectedData = (expectedData << 1) | (~parse_crc & 0x0200) | (parse_crc & 0x01e0);		// fixed stuff bit (inverse of bit 9) and bits 5-8
	expectedData = (expectedData << 1) | (~parse_crc & 0x0020) | (parse_crc & 0x001e);		// fixed stuff bit (inverse of bit 5) and bits 1-4
	expectedData = (expectedData << 1) | (~parse_crc & 0x0002) | (parse_crc & 0x0001);		// fixed stuff bit (inverse of bit 1) and bit 0
	expectedData = (expectedData << 1) | 0x0001;
#else
	uint32_t expectedData = (parse_crc & 0x01)						// bit 0 of CRC
						  | (~parse_crc & 0x02)						// fixed stuff bit = inverse of bit 1
						  | ((parse_crc & 0x1e) << 1)				// bits 1-4 of CRC
						  | ((~parse_crc & 0x20) << 1)				// fixed stuff bit = inverse of bit 5
						  | ((parse_crc & 0x1e0) << 2)				// bits 5-8 of CRC
						  | ((~parse_crc & 0x0200) << 2)			// fixed stuff bit = inverse of bit 9
						  | ((parse_crc & 0x1e00) << 3)				// bits 9-12 of CRC
						  | ((~parse_crc & 0x2000) << 3)			// fixed stuff bit = inverse of bit 13
						  | ((parse_crc & 0xe000) << 4);			// bits 13-15 of CRC
	expectedData = (expectedData << 1) | 0x0001;
#endif

    const uint32_t cs = unstuf.GetStuffCount();
    const uint32_t crcstart_bitpos = raw_bit_count - cs - 1;
    const uint32_t crcend_bitpos = crcstart_bitpos + 19;

    if (tx_check_local_message())
    {
        // This is a self transmit - setup tx eof "matched" signal
        report_state = (ReportState)(ReportState::RS_IN_MSG | ReportState::RS_IS_TX);
        pio_match_check(pio_match_calc_key((expectedData << 9) | 0x00ff, crcend_bitpos + 10));
    }
    else if (rxFifoNumber < NumCanRxFifos)
    {
		// Inject ack
		report_state = ReportState::RS_IN_MSG;
		const uint32_t match_key = pio_match_calc_key(expectedData, crcend_bitpos + 1);
	    if (   tx_state == TS_QUEUED
	    	&& !pio_tx_did_conflict()
	        && pio_rx_fifo_level() > 1
		   )
	    {
	        // Rx state is behind - acking wont succeed and may halt active tx
	    	++regs->errors.tooLateToAck;
	    }
	    else
	    {
			tx_state = TS_ACKING_RX;
			pio_tx_inject_ack(match_key);
			pio_irq_set_maytx_ackdone();

			// Setup for future rx eof "matched" signal
			report_eof_key = pio_match_calc_key((expectedData << 8) | 0x7f, crcend_bitpos + 9);
		}
    }
    else
    {
		// We are not acking this message because it isn't meant for us. Setup for future rx eof "matched" signal, assuming that an ack bit is seen.
		report_eof_key = pio_match_calc_key((expectedData << 8) | 0x7f, crcend_bitpos + 9);
    }
}

// Parser found successful ack
void CRITICAL_MEMBER(CanFD2040, report_note_ack_success)() noexcept
{
    if (!(report_state & ReportState::RS_IN_MSG))
    {
        // Got rx "ackdone" and "matched" signals already
    }
    else
    {
		report_state = (ReportState)(report_state | ReportState::RS_AWAIT_EOF);
		if (report_state & ReportState::RS_IS_TX)
		{
			// Enable "matched" irq for fast back-to-back transmit scheduling
			pio_irq_set_maytx_matched();
		}
    }
}

// Parser found successful EOF
void CRITICAL_MEMBER(CanFD2040, report_note_eof_success)() noexcept
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
void CRITICAL_MEMBER(CanFD2040, report_line_ackdone)() noexcept
{
    if (!(report_state & ReportState::RS_IN_MSG))
    {
        // Parser already processed ack and eof bits
        pio_irq_set_maytx();
    }
    else
    {
		// Setup "matched" irq for fast rx callbacks
		report_state = (ReportState)(ReportState::RS_IN_MSG | ReportState::RS_AWAIT_EOF);
		pio_match_check(report_eof_key);
		pio_irq_set_maytx_matched();
		// Schedule next transmit (so it is ready for next frame line arbitration)
		tx_schedule_transmit();
    }
}

// Received PIO "matched" irq
void CRITICAL_MEMBER(CanFD2040, report_line_matched)() noexcept
{
    // Implement fast rx callback and/or fast back-to-back tx scheduling
    report_handle_eof();
    pio_irq_set_none();
    tx_schedule_transmit();
}

// Received 10+ passive bits on the line (between 10 and 17 bits)
void CRITICAL_MEMBER(CanFD2040, report_line_maytx)() noexcept
{
    // Line is idle - may be unexpected EOF, missed ack injection,
    // missed "matched" signal, or can2040_transmit() kick.
    report_handle_eof();
    pio_irq_set_none();
    tx_schedule_transmit();
}

// Transition to the next parsing state
void CRITICAL_MEMBER(CanFD2040, data_state_go_next)(ParseState state, uint32_t bits) noexcept
{
    parse_state = state;
    unstuf.SetCount(bits);
}

// Transition to the MS_DISCARD state - drop all bits until 6 passive bits
void CanFD2040::data_state_go_discard() noexcept
{
    report_note_parse_error();
    unstuf.UseDynamicStuffing();

	// I'm not sure that this is correct. It should really be a test to see if the PIO reader has
	// ever stalled, not if it is stalled now. Without that the two bit counts can get out of sync.
	// For now the simple thing to do seems to always reset things, this seems to work well for me.
	// TODO: Investigate if a better way to do this is needed
    //if (pio_rx_check_stall())
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
ALIGNED_FUNCTION void CRITICAL_MEMBER(CanFD2040, data_state_line_passive)() noexcept
{
    if (parse_state != MS_DISCARD)
    {
        // Bitstuff error
        data_state_go_discard();
    }
    else
    {
    	unstuf.UseDynamicStuffing();
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
void CRITICAL_MEMBER(CanFD2040, data_state_go_data)() noexcept
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
void CRITICAL_MEMBER(CanFD2040, data_state_go_stuff_count)() noexcept
{
	unstuf.UseFixedStuffing();								// tell the unstuffer that the next bit and then every 4 bits are forced stuff bits
	data_state_go_next(MS_STUFFCOUNT, (parse_dlc > 10) ? 10 : 6);	// receive the 4 stuff count bits + the first 2 or 6 CRC bits (and 2 fixed stuff bits)
}

// Handle reception of first bit of header (after start-of-frame (SOF))
void CRITICAL_MEMBER(CanFD2040, data_state_update_start)(uint32_t data) noexcept
{
	rxTimeStamp = timer_hw->timerawl;						// save time stamp for later
	data &= 0x01;
    parse_id = data;										// store the first data bit (MSbit of ID)
    report_note_message_start();
    unstuf.UseDynamicStuffing();							// we're already using dynamic stuffing but we need to reset the count of stuff bits
    unstuf.InitCrc(data, 2);								// add the SOF and the first header bit to the CRCs
    data_state_go_next(MS_HEADER, 20);
}

// If there is space in receive fifo 'whichFifo', set rxMessage and FifoNumber accordingly, else leave them alone and flag the overflow
void CanFD2040::SetupToReceive(unsigned int whichFifo, bool extendedId) noexcept
{
	if (whichFifo < NumCanRxFifos)
	{
		VirtualCanRegisters::RxFifo& fifo = regs->rxFifos[whichFifo];
		const uint32_t putIndex = fifo.putIndex;
		uint32_t nextPutIndex = putIndex + 1;
		if (nextPutIndex == fifo.size)
		{
			nextPutIndex = 0;
		}
		if (nextPutIndex != fifo.getIndex)
		{
			CanRxBuffer& rxBuffer = const_cast<CanRxBuffer&>(fifo.buffers[putIndex]);
			rxBuffer.R0.bit.XTD = extendedId;
			rxBuffer.R1.bit.RXTS = rxTimeStamp;
			rxMessage = rxBuffer.data;
			rxFifoNumber = whichFifo;
		}
		else
		{
			++regs->errors.rxFifoOverlow[whichFifo];
		}
	}
}

// Handle reception of next 20 header bits which for CAN-FD frame with short ID takes us up to and including the DLC bits
ALIGNED_FUNCTION void CRITICAL_MEMBER(CanFD2040, data_state_update_header)(uint32_t data) noexcept
{
    data |= parse_id << 20;									// or in the most significant ID bit
    if ((data & 0x0300) == 0x0300)							// if SRR and ID are both set
    {
        // Extended ID
        parse_id = ((data & 0x1ffc00) << 8) | ((data & 0x00ff) << 10);	// this is now the top 19 bits of the ID field
        data_state_go_next(MS_EXT_HEADER, 19);
    }
    else if ((data & 0x3e0) == 0x80)						// if r1 and IDE are both clear, FDF is set, r0 and and BRS are clear
    {
		// Short ID
		parse_id = (data >> 10) & 0x7ff;					// save the complete ID
		parse_dlc = data & 0x0F;

		// See if we are interested in this message
		rxFifoNumber = NumCanRxFifos;						// set an invalid fifo number
		rxMessage = rxDummyMessage;
		for (unsigned int filterNumber = 0; filterNumber < regs->numShortFilterElements; ++filterNumber)
		{
			const CanStandardMessageFilterElement& f = const_cast<const CanStandardMessageFilterElement&>(regs->shortFiltersAddr[filterNumber]);
			if (f.Matches(parse_id))
			{
				SetupToReceive(f.whichBuffer, false);
				break;
			}
		}

		data_state_go_data();
    }
	else
	{
		++regs->errors.wrongMessageType;
		data_state_go_discard();							// not a message format we can parse
	}
}

// Handle reception of additional 19 bits of "extended header" which takes us to the end of the DLC field
ALIGNED_FUNCTION void CRITICAL_MEMBER(CanFD2040, data_state_update_ext_header)(uint32_t data) noexcept
{
	if ((data & 0x1e0) == 0x80)								// if FDF bit is set, and r1, BRS and r0 are not set
	{
		parse_id |= (data >> 9) & 0x03ffff;					// or-in the bottom 18 bits of the ID
		parse_dlc = data & 0x0F;

		// See if we are interested in this message
		rxFifoNumber = NumCanRxFifos;						// set an invalid fifo number
		rxMessage = rxDummyMessage;
		for (unsigned int filterNumber = 0; filterNumber < regs->numExtendedFilterElements; ++filterNumber)
		{
			const CanExtendedMessageFilterElement& f = const_cast<const CanExtendedMessageFilterElement&>(regs->extendedFiltersAddr[filterNumber]);
			if (f.Matches(parse_id))
			{
				SetupToReceive(f.whichBuffer, true);
				break;
			}
		}

		data_state_go_data();
	}
	else
	{
		++regs->errors.wrongMessageType;
		data_state_go_discard();
	}
}

// Handle reception of data content
ALIGNED_FUNCTION void CRITICAL_MEMBER(CanFD2040, data_state_update_data)(uint32_t data) noexcept
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

// Handle reception of 4 bits of message stuff count and 2 or 6 bytes of CRC
ALIGNED_FUNCTION void CRITICAL_MEMBER(CanFD2040, data_state_update_stuffCount)(uint32_t data) noexcept
{
	uint32_t stuffCount;
	uint32_t crcBits;
	if (parse_dlc > 10)
	{
		stuffCount = data >> 6;									// get stuff count and parity bit
		unstuf.AddCrcBits(stuffCount, 4);
		parse_crc = unstuf.GetCrc21();							// set up the expected CRC
		crcBits = data & 0x3f;									// get top 6 bits of CRC
	}
	else
	{
		stuffCount = data >> 2;									// get stuff count and parity bit
		unstuf.AddCrcBits(stuffCount, 4);
		parse_crc = unstuf.GetCrc17();							// set up the expected CRC
		crcBits = data & 0x03;									// get top 2 bits of CRC
	}

	// Check stuff count parity (should be even) and stuff count
	if (unlikely((stuffCount & 0x0f) != GrayTable[unstuf.GetTotalStuffBits() & 0x07]))
	{
		// Check whether the parity is wrong so we can report a parity error separately
		uint32_t parity = stuffCount ^ (stuffCount >> 2);
		parity ^= (parity >> 1);
		if (parity & 1u)
		{
			++regs->errors.stuffCountParity;
		}
		else
		{
			++regs->errors.wrongStuffCount;
		}
		data_state_go_discard();
	}
	else if (likely((parse_crc >> 15) == crcBits))
	{
		// Transition to MS_CRC state - await another 15 bits of crc + delimiter
		report_note_crc_start();
		data_state_go_next(MS_CRC, 16);							// read 15 more CRC bits (plus fixed stuffing bits) plus delimiter
	}
	else
	{
		++regs->errors.wrongCrc;
		data_state_go_discard();
	}
}

// Handle reception of 16 bits of message CRC (15 crc bits + crc delimiter)
ALIGNED_FUNCTION void CRITICAL_MEMBER(CanFD2040, data_state_update_crc)(uint32_t data) noexcept
{
	if (likely((((parse_crc & 0x7fff) << 1) | 1u) == (data & 0xffff)))
    {
		unstuf.UseNoStuffing();
		data_state_go_next(MS_ACK, 2);
    }
	else if (data & 0x01)
    {
    	++regs->errors.wrongCrc;
        data_state_go_discard();
    }
	else
	{
		++regs->errors.missingCrcDelimiter;
        data_state_go_discard();
	}
}

// Handle reception of 2 bits of ack phase (ack, ack delimiter)
ALIGNED_FUNCTION void CRITICAL_MEMBER(CanFD2040, data_state_update_ack)(uint32_t data) noexcept
{
	if (data != 0x01)
    {
		++regs->errors.noAck;
        data_state_go_discard();
    }
	else
	{
		report_note_ack_success();
		data_state_go_next(MS_EOF0, 4);
	}
}

// Handle reception of first four end-of-frame (EOF) bits
ALIGNED_FUNCTION void CRITICAL_MEMBER(CanFD2040, data_state_update_eof0)(uint32_t data) noexcept
{
    if (data != 0x0f || pio_rx_check_stall())
    {
    	++regs->errors.missingEofBit1;
        data_state_go_discard();
    }
    else
    {
    	unstuf.ClearState();
    	unstuf.UseDynamicStuffing();
    	data_state_go_next(MS_EOF1, 4);
    }
}

// Handle reception of end-of-frame (EOF) bits 5-7 and first IFS bit
ALIGNED_FUNCTION void CRITICAL_MEMBER(CanFD2040, data_state_update_eof1)(uint32_t data) noexcept
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
    	++regs->errors.missingEofBit2;
        data_state_go_discard();
    }
}

// Handle data received while in MS_DISCARD state
void CanFD2040::data_state_update_discard(uint32_t data) noexcept
{
    data_state_go_discard();
}

// Update parsing state after reading the bits of the current field
ALIGNED_FUNCTION void CRITICAL_MEMBER(CanFD2040, data_state_update)(uint32_t data) noexcept
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
void CRITICAL_MEMBER(CanFD2040, process_rx)(uint32_t rx_byte) noexcept
{
	unstuf.AddBits(rx_byte, 8);
    raw_bit_count += 8;

    // undo bit stuffing
    for (;;)
    {
        const int ret = unstuf.PullBits();
        if (likely(ret > 0))
        {
            // Need more data
            break;
        } else if (likely(ret == 0))
        {
            // Pulled the next field - process it
            data_state_update(unstuf.GetUnstuffedBits());
        } else if (ret == -1)
		{
			// 6 consecutive high bits
			data_state_line_passive();
		} else
		{
			// 6 consecutive low bits, or incorrect fixed stuff bit
			++regs->errors.badStuffing;
			data_state_line_error();
		}
    }
}

// Main API irq notification function
void CRITICAL_MEMBER(CanFD2040, pio_irq_handler)() noexcept
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
    }
    else if (txStuffedWords == 0)
    {
        // No new messages to transmit
        tx_state = TS_IDLE;
    }
    else if (regs->cancelTransmission)
    {
    	txStuffedWords = 0;						// release the buffer so that the main loop can fill it with the next message
    	regs->cancelTransmission = false;
        tx_state = TS_IDLE;
    }
    else
    {
		tx_state = TS_QUEUED;
		pio_tx_send(txMessage, txStuffedWords);
    }
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

static CanFD2040 CORE1_CRITICAL_DATA_RW(canFdDevice);

extern "C" [[noreturn]]void Core1Entry() noexcept
{
	canFdDevice.Entry(&virtualRegs);
}

extern "C" ALIGNED_FUNCTION void CRITICAL_CODE(PIO_isr)() noexcept
{
	canFdDevice.pio_irq_handler();
}

// End
