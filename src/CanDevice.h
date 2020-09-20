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

# include <General/Bitmap.h>

# ifdef RTOS
#  include <RTOSIface/RTOSIface.h>
# endif

# if SAME5x
#  include <hri_can_e54.h>
# elif SAMC21
#  include <hri_can_c21.h>
#endif

class CanMessageBuffer;
class CanTiming;

// Return the highest element of a constexpr array
// This function has to be declared out-of-class so that we can refer to it in constant expressions
static inline constexpr unsigned int MaxElement(const unsigned int arr[], size_t sz) noexcept
{
	return (sz == 1) ? arr[0] : max<unsigned int>(arr[0], MaxElement(arr + 1, sz - 1));
}

class CanDevice
{
public:
	enum class RxBufferNumber : uint32_t
	{
		fifo0 = 0, fifo1,
		buffer0, buffer1, buffer2, buffer3,
	};

	enum class TxBufferNumber : uint32_t
	{
		fifo = 0,
		buffer0, buffer1, buffer2, buffer3,
	};

	// Initialise one of the CAN interfaces and return a pointer to the corresponding device. Returns null if device is already in use or device number is out of range.
	static CanDevice *Init(unsigned int whichCan, unsigned int whichPort, bool useFDMode, const CanTiming& timing) noexcept;

	// Free the device
	void DeInit() noexcept;

	// Enable the device
	void Enable() noexcept;

	// Disable the device
	void Disable() noexcept;

	// Wait for a transmit buffer to become free, with timeout. Return true if it's free.
	bool IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
		pre(whichBuffer >= -2; whichBuffer < NumTxBuffers);

	// Queue a message for sending via a buffer or FIFO. If the buffer isn't free, cancel the previous message (or oldest message in the fifo) and send it anyway.
	void SendMessage(TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
		pre(whichBuffer >= -2; whichBuffer < NumTxBuffers);

	// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
	bool ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
		pre(whichBuffer >= -2; whichBuffer < NumRxBuffers);

	// Check whether a message is available, returning true if it is
	bool IsMessageAvailable(RxBufferNumber whichBuffer, uint32_t timeout) noexcept;
		pre(whichBuffer >= -2; whichBuffer < NumRxBuffers);

	// Set a short ID field filter element. To disable the filter element, use a zero mask parameter.
	// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
	void SetShortFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
		pre(index < NumShortFilterElements; whichBuffer >= -2; whichBuffer < NumRxBuffers);

	// Set an extended ID field filter element. To disable the filter element, use a zero mask parameter.
	// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
	void SetExtendedFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
		pre(index < NumShortFilterElements; whichBuffer >= -2; whichBuffer < NumRxBuffers);

	void GetLocalCanTiming(CanTiming& timing) noexcept;

	void SetLocalCanTiming(const CanTiming& timing) noexcept;

	void Interrupt() noexcept;

	// Configuration constants. Need to be public because they are used to size static data in CanDevice.cpp
	static constexpr size_t Can0DataSize = 64;

# if SAME70
	static constexpr unsigned int NumCanDevices = 2;
	static constexpr unsigned int NumTxBuffers[NumCanDevices] = { 6, 6 };
	static constexpr unsigned int TxFifoSize[NumCanDevices] = { 4, 4 };
	static constexpr unsigned int NumRxBuffers[NumCanDevices] = { 4, 4 };
	static constexpr unsigned int RxFifo0Size[NumCanDevices] = { 16, 16 };
	static constexpr unsigned int RxFifo1Size[NumCanDevices] = { 16, 16 };
	static constexpr unsigned int NumShortFilterElements[NumCanDevices] = { 0, 0 };
	static constexpr unsigned int NumExtendedFilterElements[NumCanDevices] = { 3, 3 };
	static constexpr unsigned int TxEventFifoSize[NumCanDevices] = { 2, 2 };
# elif SAME5x
	static constexpr unsigned int NumCanDevices = 1;
	static constexpr unsigned int NumTxBuffers[NumCanDevices] = { 0 };
	static constexpr unsigned int TxFifoSize[NumCanDevices] = { 4 };
	static constexpr unsigned int NumRxBuffers[NumCanDevices] = { 0 };
	static constexpr unsigned int RxFifo0Size[NumCanDevices] = { 16 };
	static constexpr unsigned int RxFifo1Size[NumCanDevices] = { 0 };
	static constexpr unsigned int NumShortFilterElements[NumCanDevices] = { 0 };
	static constexpr unsigned int NumExtendedFilterElements[NumCanDevices] = { 2 };
	static constexpr unsigned int TxEventFifoSize[NumCanDevices] = { 2 };
#elif SAMC21
	static constexpr unsigned int NumCanDevices = 1;
	static constexpr unsigned int NumTxBuffers[NumCanDevices] = { 0 };
	static constexpr unsigned int TxFifoSize[NumCanDevices] = { 16 };
	static constexpr unsigned int NumRxBuffers[NumCanDevices] = { 0 };
	static constexpr unsigned int RxFifo0Size[NumCanDevices] = { 16 };
	static constexpr unsigned int RxFifo1Size[NumCanDevices] = { 0 };
	static constexpr unsigned int NumShortFilterElements[NumCanDevices] = { 0 };
	static constexpr unsigned int NumExtendedFilterElements[NumCanDevices] = { 2 };
	static constexpr unsigned int TxEventFifoSize[NumCanDevices] = { 2 };
#endif

	static_assert(NumTxBuffers[0] + TxFifoSize[0] <= 32);		// maximum total Tx buffers supported is 32
	static_assert(NumTxBuffers[0] + 1 <= 32);					// our code only allows 31 buffers + the FIFO
	static_assert(NumRxBuffers[0] + 2 <= 32);					// the peripheral supports up to 64 buffers but our code only allows 30 buffers + the two FIFOs
	static_assert(RxFifo0Size[0] <= 64);						// max 64 entries per receive FIFO
	static_assert(RxFifo1Size[0] <= 64);						// max 64 entries per receive FIFO

#if SAME70
	static constexpr size_t Can1DataSize = 64;
	static_assert(NumTxBuffers[1] + TxFifoSize[1] <= 32);		// maximum total Tx buffers supported is 32
	static_assert(NumTxBuffers[1] + 1 <= 32);					// our code only allows 31 buffers + the FIFO
	static_assert(NumRxBuffers[1] + 2 <= 32);					// the peripheral supports up to 64 buffers but our code only allows 30 buffers + the two FIFOs
	static_assert(RxFifo0Size[1] <= 64);						// max 64 entries per receive FIFO
	static_assert(RxFifo1Size[1] <= 64);						// max 64 entries per receive FIFO
#endif

	struct CanTxEventEntry;
	struct CanStandardMessageFilterElement;
	struct CanExtendedMessageFilterElement;
	struct CanContext;

private:
	static constexpr unsigned int qq = 4;		//TEMPORARY!

	static CanDevice devices[NumCanDevices];
	static const CanContext CanContexts[CanDevice::NumCanDevices];

	CanDevice() noexcept { }
	void DoHardwareInit() noexcept;
	void UpdateLocalCanTiming(const CanTiming& timing) noexcept;

	Can *hw;													// address of the CAN peripheral we are using
	const CanContext *context;									// pointer to a struct the points to the buffers and filter elements

# ifdef RTOS
	TaskHandle txTaskWaiting[MaxElement(NumTxBuffers, ARRAY_SIZE(NumTxBuffers)) + 1];	// tasks waiting for each Tx buffer to become free, first entry is for the Tx FIFO
	TaskHandle rxTaskWaiting[MaxElement(NumRxBuffers, ARRAY_SIZE(NumRxBuffers)) + 2];	// tasks waiting for each Rx buffer to receive a message, first 2 entries are for the fifos
	Bitmap<uint32_t> rxBuffersWaiting;							// which Rx FIFOs and buffers tasks are waiting on
	Bitmap<uint32_t> txBuffersWaiting;							// which Tx FIFOs and buffers tasks are waiting on
# endif

	unsigned int whichCan;										// which CAN device we are
	unsigned int whichPort;										// which CAN port number we use, 0 or 1
	unsigned int messagesLost;									// count of received messages lost because the receive FIFO was full
	unsigned int busOffCount;									// count of the number of times we have reset due to bus off
	bool useFDMode;
};

#endif

#endif /* SRC_CANDEVICE_H_ */
