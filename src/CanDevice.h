/*
 * CanDevice.h
 *
 *  Created on: 2 Sep 2020
 *      Author: David
 */

#ifndef SRC_CANDEVICE_H_
#define SRC_CANDEVICE_H_

#include <CoreIO.h>

#if SUPPORT_CAN

# include <CanId.h>
# include "CanFilterElements.h"
# include <General/Bitmap.h>

# ifdef RTOS
#  include <RTOSIface/RTOSIface.h>
# endif

#if RP2040
# include <hardware/structs/timer.h>
# include <RP2040/VirtualCanRegisters.h>
#endif

#if RP2040
constexpr unsigned int MaxTxBuffers = 0;			// maximum number of dedicated transmit buffers supported by this driver
constexpr unsigned int MaxRxBuffers = 0;			// maximum number of dedicated receive buffers supported by this driver
#else
constexpr unsigned int MaxTxBuffers = 6;			// maximum number of dedicated transmit buffers supported by this driver
constexpr unsigned int MaxRxBuffers = 4;			// maximum number of dedicated receive buffers supported by this driver
#endif

static_assert(MaxTxBuffers <= 31);					// the hardware allows up to 32 if there is no transmit FIFO but our code only supports up to 31 + a FIFO
static_assert(MaxRxBuffers <= 30);					// the hardware allows up to 64 but our code only supports up to 30 + the FIFOs

# if SAME70
constexpr unsigned int NumCanDevices = 2;			// this driver supports both CAN devices on the SAME70
typedef Mcan Can;
#else
constexpr unsigned int NumCanDevices = 1;			// on other MCUs we only support one CAN device
# endif

class CanMessageBuffer;
class CanTiming;
class CanRxBufferHeader;
class CanTxBufferHeader;

class CanDevice
{
public:
	enum class RxBufferNumber : uint32_t
	{
		fifo0 = 0, fifo1,
#if !RP2040
		buffer0, buffer1, buffer2, buffer3,
#endif
		none = 0xFFFF
	};

	enum class TxBufferNumber : uint32_t
	{
		fifo = 0,
#if !RP2040
		buffer0, buffer1, buffer2, buffer3, buffer4, buffer5,
#endif
	};

	// Struct used to pass configuration constants, with default values
	struct Config
	{
		unsigned int dataSize = 64;											// must be one of: 8, 12, 16, 20, 24, 32, 48, 64
#if RP2040
		unsigned int numTxBuffers = 0;
#else
		unsigned int numTxBuffers = 2;
#endif
		unsigned int txFifoSize = 4;
		unsigned int numRxBuffers = 0;
		unsigned int rxFifo0Size = 16;
		unsigned int rxFifo1Size = 16;
		unsigned int numShortFilterElements = 0;
		unsigned int numExtendedFilterElements = 3;
#if RP2040
		unsigned int txEventFifoSize = 0;
#else
		unsigned int txEventFifoSize = 16;
#endif

		static constexpr size_t StandardFilterElementSize = sizeof(CanStandardMessageFilterElement)/sizeof(uint32_t);				// one word each
		static constexpr size_t ExtendedFilterElementSize = sizeof(CanExtendedMessageFilterElement)/sizeof(uint32_t);				// two words

		// Test whether the data size is supported by the CAN hardware
		constexpr bool ValidDataSize() const noexcept
		{
#if RP2040
			return dataSize == 64;
#else
			return dataSize >= 8
				&& (   (dataSize <= 24 && (dataSize & 3) == 0)
					|| (dataSize <= 64 && (dataSize & 15) == 0)
				   );
#endif
		}

		// Test whether this is a valid CAN configuration. Use this in a static_assert to check that a specified configuration is valid.
		constexpr bool IsValid() const noexcept
		{
			return ValidDataSize()
#if RP2040
				&& numTxBuffers == 0										// our RP2040 code doesn't support dedicated Tx buffers
				&& txEventFifoSize == 0										// our RP2040 code doesn't support the transmit event FIFO
				&& numRxBuffers == 0;										// our RP2040 code doesn't support dedicates receive buffers
#else
				&& numTxBuffers + txFifoSize <= 32							// maximum total Tx buffers supported is 32
				&& numTxBuffers <= MaxTxBuffers								// our code only allows 31 buffers + the FIFO
				&& rxFifo0Size <= 64										// max 64 entries per receive FIFO
				&& rxFifo1Size <= 64										// max 64 entries per receive FIFO
				&& txEventFifoSize <= 32									// max 32 entries in transmit event FIFO
				&& numRxBuffers <= MaxRxBuffers;							// the peripheral supports up to 64 buffers but our code only allows 30 buffers + the two FIFOs
#endif
		}

		// Return the number of words of memory occupied by the 11-bit filters
		// We round this up to the next multiple of 8 bytes to reduce the chance of the Tx and Rx buffers crossing cache lines
		constexpr size_t GetStandardFiltersMemSize() const noexcept
		{
			return ((numShortFilterElements * StandardFilterElementSize) + 1u) & (~1u);
		}

		// Return the number of words of memory occupied by the 29-bit filters
		constexpr size_t GetExtendedFiltersMemSize() const noexcept
		{
			return numExtendedFilterElements * ExtendedFilterElementSize;
		}

		// Return the number of words of memory occupied by each transmit buffer
		constexpr size_t GetTxBufferSize() const noexcept
		{
			return (dataSize >> 2) + 2;										// each transmit buffer has a 2-word header
		}

		// Return the number of words of memory occupied by each receive buffer
		constexpr size_t GetRxBufferSize() const noexcept
		{
			return (dataSize >> 2) + 2;										// each receive buffer has a 2-word header
		}

		// Return the number of words of memory occupied by the transmit event FIFO
		constexpr size_t GetTxEventFifoMemSize() const noexcept
		{
			constexpr size_t TxEventEntrySize = 2;							// each transmit event entry is 2 words
			return txEventFifoSize * TxEventEntrySize;
		}

		// Return the total amount of buffer memory needed in 32-bit words. Must be constexpr so we can allocate memory statically in the correct segment.
		constexpr size_t GetMemorySize() const noexcept
		{
			return
#if RP2040
				// The RP2040 implementation wastes one slot in each FIFO and has no dedicated buffers
				  (txFifoSize + 1) * GetTxBufferSize()
				+ (rxFifo0Size + rxFifo1Size + 2) * GetRxBufferSize()
#else
				  (numTxBuffers + txFifoSize) * GetTxBufferSize()
				+ (numRxBuffers + rxFifo0Size + rxFifo1Size) * GetRxBufferSize()
				+ GetTxEventFifoMemSize()
#endif
				+ GetStandardFiltersMemSize()
				+ GetExtendedFiltersMemSize();
		}
	};

	// Struct to represent CAN statistics
	struct CanStats
	{
		unsigned int messagesQueuedForSending;
		unsigned int messagesReceived;
		unsigned int messagesLost;									// count of received messages lost because the receive FIFO was full
		unsigned int protocolErrors;
		unsigned int busOffCount;									// count of the number of times we have reset due to bus off

		void Clear() noexcept;
	};

	// Type of the callback function called when a transmi event with a nonzero message marker occurs
	typedef void (*TxEventCallbackFunction)(uint8_t marker, CanId id, uint16_t timeStamp) noexcept;

	// Initialise one of the CAN interfaces and return a pointer to the corresponding device. Returns null if device is already in use or device number is out of range.
	// IMPORTANT: the CanDevice stores a copy of the p_config reference. The Config structure that is refers to must remain available and unchanged while the CanDevice is being used!
	static CanDevice *Init(
#if RP2040
							Pin p_txPin, Pin p_rxPin,
#else
							unsigned int p_whichCan, unsigned int p_whichPort,
#endif
							const Config& p_config, uint32_t *memStart, const CanTiming& timing, TxEventCallbackFunction p_txCallback) noexcept;

#if !RP2040
	// Set the extended ID mask. May only be used while the interface is disabled. Only needed when using dedicated buffers.
	void SetExtendedIdMask(uint32_t mask) noexcept;
#endif

	// Free the device
	void DeInit() noexcept;

	// Enable the device
	void Enable() noexcept;

	// Disable the device
	void Disable() noexcept;

	// Wait for a transmit buffer to become free, with timeout. Return true if it's free.
	bool IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept;

#if 0	// not currently used
	// Return the number of messages waiting to be sent in the transmit FIFO
	unsigned int NumTxMessagesPending(TxBufferNumber whichBuffer) noexcept;
#endif

	// Queue a message for sending via a buffer or FIFO. If the buffer isn't free, cancel the previous message (or oldest message in the fifo) and send it anyway.
	// Returns the ID of the message that was cancelled, or 0 if we didn't cancel a message.
	uint32_t SendMessage(TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept;

	// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
	bool ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *null buffer) noexcept;

	// Check whether a message is available, returning true if it is
	bool IsMessageAvailable(RxBufferNumber whichBuffer) noexcept;

	// Disable a short ID filter element
	void DisableShortFilterElement(unsigned int index) noexcept;

	// Set a short ID field filter element
	// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
	void SetShortFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
		pre(index < NumShortFilterElements);

	// Disable an extended ID filter element
	void DisableExtendedFilterElement(unsigned int index) noexcept;

	// Set an extended ID field filter element
	// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
	void SetExtendedFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
		pre(index < NumShortFilterElements);

	void GetLocalCanTiming(CanTiming& timing) noexcept;

	void SetLocalCanTiming(const CanTiming& timing) noexcept;

	void GetAndClearStats(CanDevice::CanStats& dst) noexcept;

	uint16_t ReadTimeStampCounter() noexcept
	{
#if RP2040
		return timer_hw->timerawl;									// read lower 32 bits of the hardware timer, which we also use for CAN time stamping
#elif SAME70
		return hw->MCAN_TSCV;
#else
		return hw->TSCV.reg;
#endif
	}

#if !SAME70 && !RP2040
	uint16_t GetTimeStampPeriod() noexcept
	{
		return bitPeriod;
	}
#endif

#if !RP2040
	void PollTxEventFifo(TxEventCallbackFunction p_txCallback) noexcept;
#endif

#if RP2040
	void GetAndClearErrorCounts(CanErrorCounts& errs) noexcept;
#else
	uint32_t GetErrorRegister() const noexcept;
#endif

#ifdef RTOS
	void Interrupt() noexcept;
#endif

	// Configuration constants. Need to be public because they are used to size static data in CanDevice.cpp
	static constexpr size_t Can0DataSize = 64;

private:
#if !RP2040
	struct TxEvent;
#endif

	void DoHardwareInit() noexcept;
#if !RP2040
	void UpdateLocalCanTiming(const CanTiming& timing) noexcept;
#endif
	uint32_t GetRxBufferSize() const noexcept;
	uint32_t GetTxBufferSize() const noexcept;
	CanRxBufferHeader *GetRxFifo0Buffer(uint32_t index) const noexcept;
	CanRxBufferHeader *GetRxFifo1Buffer(uint32_t index) const noexcept;
#if !RP2040
	CanRxBufferHeader *GetRxBuffer(uint32_t index) const noexcept;
#endif
	CanTxBufferHeader *GetTxBuffer(uint32_t index) const noexcept;
#if !RP2040
	TxEvent *GetTxEvent(uint32_t index) const noexcept;
#endif

	void CopyMessageForTransmit(CanMessageBuffer *buffer, volatile CanTxBufferHeader *f) noexcept;
	void CopyReceivedMessage(CanMessageBuffer *null buffer, const volatile CanRxBufferHeader *f) noexcept;

#if RP2040
	VirtualCanRegisters registers;								// virtual register set used to pass info between cores
	bool inUse = false;
#else
	Can *hw = nullptr;											// address of the CAN peripheral we are using
	unsigned int whichCan;										// which CAN device we are
	unsigned int whichPort;										// which CAN port number we use, 0 or 1

	uint32_t nbtp;												//!< The NBTP register that gives the required normal bit timing
	uint32_t dbtp;												//!< The DBTP register that gives the required bit timing when we use BRS
#endif

	uint32_t statusMask;

	const Config *config;										//!< Configuration parameters
	volatile uint32_t *rx0Fifo;									//!< Receive message fifo start
	volatile uint32_t *rx1Fifo;									//!< Receive message fifo start
	volatile uint32_t *rxBuffers;								//!< Receive direct buffers start
	uint32_t *txBuffers;										//!< Transmit direct buffers start (the Tx fifo buffers follow them)
#if !RP2040
	TxEvent *txEventFifo;										//!< Transfer event fifo
#endif
	CanStandardMessageFilterElement *rxStdFilter;				//!< Standard filter List
	CanExtendedMessageFilterElement *rxExtFilter;				//!< Extended filter List

	CanStats stats;												//!< Statistics gathered

#if !RP2040	// we don't emulate the transmit event fifo on the RP2040
	TxEventCallbackFunction txCallback;							// function that gets called by the ISR when a transmit event for a message with a nonzero marker occurs
#endif

# ifdef RTOS
	// The following are all declared volatile because we care about when they are written
	volatile TaskHandle txTaskWaiting[MaxTxBuffers + 1];		// tasks waiting for each Tx buffer to become free, first entry is for the Tx FIFO
	volatile TaskHandle rxTaskWaiting[MaxRxBuffers + 2];		// tasks waiting for each Rx buffer to receive a message, first 2 entries are for the fifos
	std::atomic<uint32_t> rxBuffersWaiting;						// which buffers tasks are waiting on
# endif

#if !SAME70 && !RP2040
	uint16_t bitPeriod;											// how many clocks in a CAN normal bit
#endif

#if !RP2040
	bool useFDMode;
#endif
};

#endif	// SUPPORT_CAN

#endif /* SRC_CANDEVICE_H_ */
