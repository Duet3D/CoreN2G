/*
 * CanFD2040.h
 *
 *  Created on: 22 Aug 2022
 *      Author: David
 *
 * This is the header file for the low-level driver for partial ISO CAN-FD support on the RP2040.
 * It is derived from Kevin Connor's CAN 2.0 implementation for the RP2040, see https://github.com/KevinOConnor/can2040
 * In particular, the PIO code is from that project and the low-level functions are derived from it.
 *
 * IMPORTANT! Robert Bosch GmbH owns many relevant patents and requires a license fee to be paid for any commercial use of CAN-FD.
 *
 * Software license: GNU GENERAL PUBLIC LICENSE Version 3
 */

#ifndef SRC_CANFD2040_H_
#define SRC_CANFD2040_H_

#include <CoreIO.h>
#undef from				// 'from' is used as an identifier in the Pico SDK

#include "VirtualCanRegisters.h"

extern "C" {
#include <pico.h>
#include <pico/critical_section.h>
#include <hardware/structs/pio.h>				// for pio_hw_t
}

// Class to de-stuff received bits. It also maintains the CRCs because for CAN-FD the stuffing bits are included in the CRC.
class BitUnstuffer
{
public:
	void AddBits(uint32_t data, uint32_t count) noexcept;
	void SetCount(uint32_t count) noexcept;
	void ClearState() noexcept;

	int PullBits() noexcept;
	uint32_t GetStuffedBits() const noexcept { return stuffed_bits; }
	uint32_t GetUnstuffedBits() const noexcept { return unstuffed_bits; }
	uint32_t GetStuffCount() const noexcept { return stuffedBitsAvailable; }
	void ClearStuffedBits() noexcept { stuffed_bits = 0; }
	void ClearStuffCount() noexcept { stuffedBitsAvailable = 0; }

	void UseFixedStuffing() noexcept { stuffType = StuffingType::fixed; bitsUntilFixedStuffBit = 0; }
	void UseDynamicStuffing() noexcept { stuffType = StuffingType::dynamic; totalStuffBits = 0; }
	void UseNoStuffing() noexcept { stuffType = StuffingType::none; }

	uint32_t GetTotalStuffBits() const noexcept { return totalStuffBits; }	// return the number of dynamic stuff bits that were counted

	// CRC functions
	void InitCrc(uint32_t data, unsigned int numBits) noexcept;

	// Add between 1 and 32 bits (0 is not allowed) to both CRCs
	void AddCrcBits(uint32_t data, unsigned int numBits) noexcept;

	// Finalise and get the CRC17, right justified
	uint32_t GetCrc17() const noexcept;

	// Finalise and get the CRC21, right justified
	uint32_t GetCrc21() const noexcept;

private:
	enum class StuffingType { dynamic = 0, fixed, none };

	uint32_t stuffed_bits;						// the last 5 (or more) bits received
	unsigned int stuffedBitsAvailable;			// how many unprocessed bits we have in stuffed_bits
	uint32_t unstuffed_bits;					// bits we have destuffed, but we don't have as many as asked for
	unsigned int unstuffedBitsWanted;			// how may more destuffed bits we want
	uint32_t crc17;								// the accumulated CRC17
	uint32_t crc21;								// the accumulated CRC21
	unsigned int totalStuffBits;				// the total number of stuffing bits we removed
	unsigned int bitsUntilFixedStuffBit;		// how many bits we need to process before we expect a fixed stuffing bit
	unsigned int numBitsLeftOver;
	uint32_t bitsLeftOver;
	StuffingType stuffType;						// what sort of bit stuffing we are decoding
};

class CanFD2040
{
public:
	// Start CAN-FD running
	[[noreturn]] void Entry(VirtualCanRegisters *p_regs) noexcept;
	void pio_irq_handler() noexcept;

private:
	// Low level PIO control functions
	void pio_setup() noexcept;
	void pio_sm_setup() noexcept;
	void pio_sync_setup() noexcept;
	void pio_rx_setup() noexcept;
	void pio_match_setup() noexcept;
	void pio_tx_setup() noexcept;
	void pio_sync_normal_start_signal() noexcept;
	void pio_sync_slow_start_signal() noexcept;
	int pio_rx_check_stall() noexcept;
	int pio_rx_fifo_level() noexcept;
	void pio_match_check(uint32_t match_key) noexcept;
	static uint32_t pio_match_calc_key(uint32_t raw_bits, uint32_t rx_bit_pos) noexcept;
	void pio_match_clear() noexcept;
	void pio_tx_reset() noexcept;
	void pio_tx_send(uint32_t *data, uint32_t count) noexcept;
	void pio_tx_inject_ack(uint32_t match_key) noexcept;
	int pio_tx_did_conflict() noexcept;
	void pio_irq_set_maytx() noexcept;
	void pio_irq_set_maytx_matched() noexcept;
	void pio_irq_set_maytx_ackdone() noexcept;
	void pio_irq_atomic_set_maytx() noexcept;
	void pio_irq_set_none() noexcept;

	// Transmit functions
	void tx_schedule_transmit() noexcept;
	bool tx_check_local_message() noexcept;

	void report_callback_error(uint32_t error_code) noexcept;
	void report_callback_rx_msg() noexcept;
	void report_callback_tx_msg() noexcept;
	void report_handle_eof() noexcept;
	int report_is_acking_rx() noexcept;
	void report_note_message_start() noexcept;
	void report_note_crc_start() noexcept;
	void report_note_ack_success() noexcept;
	void report_note_eof_success() noexcept;
	void report_note_parse_error() noexcept;
	void report_line_ackdone() noexcept;
	void report_line_matched() noexcept;
	void report_line_maytx() noexcept;

	// Parsing states (stored in cd->parse_state)
	enum ParseState : uint32_t { MS_START, MS_HEADER, MS_EXT_HEADER, MS_DATA, MS_STUFFCOUNT, MS_CRC, MS_ACK, MS_EOF0, MS_EOF1, MS_DISCARD };

	// Parsing functions
	void data_state_go_next(ParseState state, uint32_t bits) noexcept;
	void data_state_go_discard() noexcept;
	void data_state_line_error() noexcept;
	void data_state_line_passive() noexcept;
	void data_state_go_data() noexcept;
	void data_state_go_stuff_count() noexcept;
	void data_state_go_crc() noexcept;
	void data_state_update_start(uint32_t data) noexcept;
	void data_state_update_header(uint32_t data) noexcept;
	void data_state_update_ext_header(uint32_t data) noexcept;
	void data_state_update_data(uint32_t data) noexcept;
	void data_state_update_stuffCount(uint32_t data) noexcept;
	void data_state_update_crc(uint32_t data) noexcept;
	void data_state_update_ack(uint32_t data) noexcept;
	void data_state_update_eof0(uint32_t data) noexcept;
	void data_state_update_eof1(uint32_t data) noexcept;
	void data_state_update_discard(uint32_t data) noexcept;
	void data_state_update(uint32_t data) noexcept;

	void process_rx(uint32_t rx_byte) noexcept;

	void TryPopulateTransmitBuffer() noexcept;
	void SetupToReceive(unsigned int whichFifo, bool extendedId) noexcept;

	// Setup
	VirtualCanRegisters *regs;
	pio_hw_t *pio_hw;

	// Bit unstuffing
	BitUnstuffer unstuf;
	uint32_t raw_bit_count;

	// Input data state
	uint32_t rxTimeStamp;
	uint32_t parse_id;
	uint32_t parse_state;
	uint32_t parse_crc;
    uint32_t parse_dlc;
    uint32_t parse_bytesReceived;
	uint32_t parse_bytesLeft;				// how many bytes of data are left to receive
	uint32_t *rxMessage;
	unsigned int rxFifoNumber;

	// Reporting
	// Report state flags (stored in report_state)
	enum ReportState : uint32_t { RS_IDLE = 0, RS_IS_TX = 1, RS_IN_MSG = 2, RS_AWAIT_EOF = 4, };

	ReportState report_state;
	uint32_t report_eof_key;

	// Transmit states (stored in tx_state)
	enum TxState : uint32_t { TS_IDLE = 0, TS_QUEUED = 1, TS_ACKING_RX = 2, TS_CONFIRM_TX = 3 };

	uint32_t dmaControlWord;				// the control word for the DMAC without the Enable bit
	uint32_t txId;
	uint32_t txDlc;
	uint32_t txCrc;
	volatile uint32_t txStuffedWords;
	TxState tx_state;

	// Pending interrupts, each in a separate word to avoid race conditions
	volatile bool rxInterruptPending[NumCanRxFifos];									// one interrupt per receive fifo
	volatile bool txFifoNotFullInterruptPending;

	// Transmit buffer
	static constexpr size_t MaxTxMessageBits = ((41 + (64 * 8)) * 5)/4					// max frame length before the stuff count and CRC, including SOF
												+ 5										// fixed stuff bit and stuff count
												+ 28;									// fixed stuff bits, 21-bit CRC and CRC delimiter
	static constexpr size_t MaxTxMessageDwords = (MaxTxMessageBits + 31)/32;			// this comes out at 23
	uint32_t txMessage[MaxTxMessageDwords];

	// Dummy message buffer, to receive messages that we don't want to store in the receive fifos
	uint32_t rxDummyMessage[64/sizeof(uint32_t)];
};

#endif /* SRC_CANFD2040_H_ */
