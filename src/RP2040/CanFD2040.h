/*
 * CanFD2040.h
 *
 *  Created on: 22 Aug 2022
 *      Author: David
 */

#ifndef SRC_CANFD2040_H_
#define SRC_CANFD2040_H_

#include <CoreIO.h>
#undef from				// 'from' is used as an identifier in the Pico SDK

extern "C" {
#include <pico.h>
#include <pico/critical_section.h>
}

#define SUPPORT_CAN_FD	0

// Class to hold an incoming or complete received message
class CanFD2040ReceiveBuffer
{
public:
	friend class CanFD2040;
	friend class CanIdFilter;

	CanFD2040ReceiveBuffer(CanFD2040ReceiveBuffer *p_next) noexcept : next(p_next) { }

	// Flags that we add to the top bits of the ID field. CAN IDs are 29 bits long so bits 29 to 31 are available.
	// We don't use a bit to flag CAN_FD messages because this driver doesn't support plain CAN messages
	enum IdExtraBits : uint32_t
	{
	    ID_RTR = 1ul << 30,
	    ID_EFF = 1ul << 31,
	};

	uint32_t GetId() const noexcept { return id; }
	uint32_t GetTimeStamp() const noexcept { return timeStamp; }
	const uint32_t *GetData32() const noexcept { return data32; }
	const uint8_t *GetData8() const noexcept { return data; }
	unsigned int GetLength() const noexcept { return numDataBytes; }

private:
	CanFD2040ReceiveBuffer *next;
    uint32_t id;
    uint32_t timeStamp;
    unsigned int numDataBytes;
    union
	{
        uint8_t data[64];
        uint32_t data32[16];
    };
};

// Class to hold a message queued or to be queued for transmission
typedef void (*CanTransmitCallback)(bool success, uint32_t param);

class CanFD2040TransmitBuffer
{
public:
	friend class CanFD2040;

	CanFD2040TransmitBuffer(CanFD2040TransmitBuffer *p_next) noexcept : next(p_next) { }

	// Set up a message ready to be queued for transmission. The callback function is set to null.
	void Populate(uint32_t p_id, const uint8_t *data, unsigned int numBytes) noexcept;

	// Set up the callback. Call this after populating the buffer but before queueing it for transmission.
	void SetCallback(CanTransmitCallback fn, uint32_t param) noexcept;

private:
	CanFD2040TransmitBuffer *next;
	CanTransmitCallback completionCallback;
	uint32_t completionParam;
	uint32_t id;
	uint32_t dlc;
	uint32_t crc;
	uint32_t stuffed_words;				// the number of dwords to send after stuffing, including the stuff count and CRC
	uint32_t stuffed_data[24];			//TODO check the maximum possible number after stuffing including the CRC
};

// Class to represent a receive filter. Each filter includes a mask to apply, and ID to match, a queue of received messages that matched this filter, and a callback function.
class CanIdFilter;

typedef void (*CanReceiveCallback)(unsigned int filterNumber);

class CanIdFilter
{
public:
	friend class CanFD2040;

	CanIdFilter(uint32_t p_id, uint32_t p_mask, CanReceiveCallback p_callback) noexcept;

	// Offer a message to the filter. Returns true if the message matches the filter .
	bool Offer(const CanFD2040ReceiveBuffer *msg) const noexcept;

	// Add a message to the queue
	void AddMessage(CanFD2040ReceiveBuffer *msg) noexcept;

	// Get the message at the head of the receive queue. The caller must release the buffer.
	CanFD2040ReceiveBuffer *GetMessage() noexcept;

	// Execute the callback
	void ExecuteCallback(unsigned int filterNumber) const noexcept;

private:
	CanIdFilter *next;
	uint32_t id;
	uint32_t mask;
	uint32_t fifoNumber;
	CanReceiveCallback callback;
	CanFD2040ReceiveBuffer *receivedMessages;
};

// Class to de-stuff received bits. It also maintains the CRCs because for CAN-FD the stuffing bits are included in the CRC.
class BitUnstuffer
{
public:
	void unstuf_add_bits(uint32_t data, uint32_t count) noexcept;
	void unstuf_set_count(uint32_t count) noexcept;
	void unstuf_clear_state() noexcept;
	int unstuf_pull_bits() noexcept;
	uint32_t GetStuffedBits() const noexcept { return stuffed_bits; }
	uint32_t GetStuffCount() const noexcept { return count_stuff; }
	void ClearStuffedBits() noexcept { stuffed_bits = 0; }
	void ClearStuffCount() noexcept { count_stuff = 0; }

private:
	uint32_t stuffed_bits;
	unsigned int count_stuff;
	uint32_t unstuffed_bits;
	unsigned int count_unstuff;
	uint32_t crc17;
	uint32_t crc21;
	unsigned int totalStuffBits;
};

class CanFD2040
{
public:
	CanFD2040(uint8_t p_pio_num, Pin txPin, Pin rxPin) noexcept;

	// Start CAN-FD running
	void Start(uint32_t sys_clock, uint32_t bitrate, unsigned int numTxBuffers, unsigned int numRxBuffers) noexcept;

	// Add a filter to the end of the current list. There is currently no facility to remove a filter.
	void AddFilter(CanIdFilter *p_filter) noexcept;

	// Queue a message for transmission. At the end the callback will be made if non-null and then the b8ffer will be released.
	void QueueForTransmit(CanFD2040TransmitBuffer *buf) noexcept;

	// Get the message at the head of the receive queue for the specified filter number. The caller must release the buffer.
	CanFD2040ReceiveBuffer *GetMessage(unsigned int filterNumber) noexcept;

	// Free a receive buffer that was returned by GetMessage
	void ReleaseReceiveBuffer(CanFD2040ReceiveBuffer *buf) noexcept;

private:
	// Low level PIO control functions
	void pio_setup(uint32_t sys_clock, uint32_t bitrate) noexcept;
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
	int tx_inject_ack(uint32_t match_key) noexcept;
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
	enum ParseState : uint32_t { MS_START, MS_HEADER, MS_EXT_HEADER, MS_DATA0, MS_DATA1, MS_CRC, MS_ACK, MS_EOF0, MS_EOF1, MS_DISCARD };

	// Parsing functions
	void data_state_go_next(ParseState state, uint32_t bits) noexcept;
	void data_state_go_discard() noexcept;
	void data_state_line_error() noexcept;
	void data_state_line_passive() noexcept;
	void data_state_go_crc() noexcept;
	void data_state_go_data(uint32_t id, uint32_t data) noexcept;
	void data_state_update_start(uint32_t data) noexcept;
	void data_state_update_header(uint32_t data) noexcept;
	void data_state_update_ext_header(uint32_t data) noexcept;
#if SUPPORT_CAN_FD
	void data_state_update_dlc(uint32_t data) noexcept;
#endif
	void data_state_update_data0(uint32_t data) noexcept;
	void data_state_update_data1(uint32_t data) noexcept;
	void data_state_update_crc(uint32_t data) noexcept;
	void data_state_update_ack(uint32_t data) noexcept;
	void data_state_update_eof0(uint32_t data) noexcept;
	void data_state_update_eof1(uint32_t data) noexcept;
	void data_state_update_discard(uint32_t data) noexcept;
	void data_state_update(uint32_t data) noexcept;

	// Setup
	uint8_t pio_num;
	Pin gpio_rx, gpio_tx;
	void *pio_hw;

	// Buffer freelists
	CanFD2040TransmitBuffer *txFreelist;
	CanFD2040ReceiveBuffer *rxFreelist;

	// Filters
	CanIdFilter *filters;					// linked list of filters for incoming messages

	// Bit unstuffing
	BitUnstuffer unstuf;
	uint32_t raw_bit_count;

	// Input data state
	uint32_t parse_state;
	uint32_t parse_crc;
    uint32_t parse_dlc;
	uint32_t parse_dwords_left;				// how many 32-bit words of data are left to receive
	CanFD2040ReceiveBuffer *parse_msg;		// the message currently being received

	// Reporting
	// Report state flags (stored in report_state)
	enum ReportState : uint32_t { RS_IDLE = 0, RS_IS_TX = 1, RS_IN_MSG = 2, RS_AWAIT_EOF = 4, };

	ReportState report_state;
	uint32_t report_eof_key;

	// Transmit states (stored in tx_state)
	enum TxState : uint32_t { TS_IDLE = 0, TS_QUEUED = 1, TS_ACKING_RX = 2, TS_CONFIRM_TX = 3 };

	TxState tx_state;
	CanFD2040TransmitBuffer *tx_queue;		// list of messages waiting to be sent

	// Critical sections
	critical_section_t txQueueCriticalSection;
	critical_section_t rxQueueCriticalSection;
};

#endif /* SRC_CANFD2040_H_ */
