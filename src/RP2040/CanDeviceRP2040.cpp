/*
 * CanDevice.cpp
 *
 *  Created on: 2 Sep 2020
 *      Author: David
 */

#include "CanDevice.h"

#if SUPPORT_CAN

#include <Cache.h>
#include <CanSettings.h>
#include <CanMessageBuffer.h>
#include <General/Bitmap.h>
#include <cstring>

#undef from
#define PICO_MUTEX_ENABLE_SDK120_COMPATIBILITY	0	// used by mutex.h which is included by multicore.h
#include <pico/multicore.h>
#include <hardware/irq.h>

/**@}*/
/**
 * \brief CAN receive FIFO element.
 */
struct CanDevice::RxBufferHeader
{
	union
	{
		struct
		{
			uint32_t ID : 29; /*!< Identifier */
			uint32_t RTR : 1; /*!< Remote Transmission Request */
			uint32_t XTD : 1; /*!< Extended Identifier */
			uint32_t ESI : 1; /*!< Error State Indicator */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} R0;
	union
	{
		struct
		{
			uint32_t RXTS : 16; /*!< Rx Timestamp */
			uint32_t DLC : 4;   /*!< Data Length Code */
			uint32_t BRS : 1;   /*!< Bit Rate Switch */
			uint32_t FDF : 1;   /*!< FD Format */
			uint32_t : 2;       /*!< Reserved */
			uint32_t FIDX : 7;  /*!< Filter Index */
			uint32_t ANMF : 1;  /*!< Accepted Non-matching Frame */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} R1;

	const volatile uint32_t *GetDataPointer() const volatile { return (volatile uint32_t*)this + (sizeof(*this)/sizeof(uint32_t)); }
};

/**
 * \brief CAN transmit FIFO element.
 */
struct CanDevice::TxBufferHeader
{
	union
	{
		struct
		{
			uint32_t ID : 29; /*!< Identifier */
			uint32_t RTR : 1; /*!< Remote Transmission Request */
			uint32_t XTD : 1; /*!< Extended Identifier */
			uint32_t ESI : 1; /*!< Error State Indicator */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} T0;
	union
	{
		struct
		{
			uint32_t : 16;    /*!< Reserved */
			uint32_t DLC : 4; /*!< Data Length Code */
			uint32_t BRS : 1; /*!< Bit Rate Switch */
			uint32_t FDF : 1; /*!< FD Format */
			uint32_t : 1;     /*!< Reserved */
			uint32_t EFCbit : 1; /*!< Event FIFO Control */
			uint32_t MM : 8;  /*!< Message Marker */
		} bit;
		uint32_t val; /*!< Type used for register access */
	} T1;

	volatile uint32_t *GetDataPointer() volatile { return (volatile uint32_t*)this + (sizeof(*this)/sizeof(uint32_t)); }
};

static CanDevice devices[NumCanDevices];

inline uint32_t CanDevice::GetRxBufferSize() const noexcept { return sizeof(RxBufferHeader)/sizeof(uint32_t) + (64 >> 2); }
inline uint32_t CanDevice::GetTxBufferSize() const noexcept { return sizeof(TxBufferHeader)/sizeof(uint32_t) + (64 >> 2); }
inline CanDevice::RxBufferHeader *CanDevice::GetRxFifo0Buffer(uint32_t index) const noexcept { return (RxBufferHeader*)(rx0Fifo + (index * GetRxBufferSize())); }
inline CanDevice::RxBufferHeader *CanDevice::GetRxFifo1Buffer(uint32_t index) const noexcept { return (RxBufferHeader*)(rx1Fifo + (index * GetRxBufferSize())); }
inline CanDevice::RxBufferHeader *CanDevice::GetRxBuffer(uint32_t index) const noexcept { return (RxBufferHeader*)(rxBuffers + (index * GetRxBufferSize())); }
inline CanDevice::TxBufferHeader *CanDevice::GetTxBuffer(uint32_t index) const noexcept { return (TxBufferHeader*)(txBuffers + (index * GetTxBufferSize())); }

extern "C" void CAN_Handler() noexcept;

/**
 * \brief CAN standard message ID filter element structure.
 *
 *  Common element structure for standard message ID filter element.
 */
struct CanDevice::StandardMessageFilterElement
{
	uint32_t id: 11,
			 whichBuffer : 3,
			 mask : 11,
			 enabled : 1;
};

static_assert(sizeof(CanDevice::StandardMessageFilterElement) == CanDevice::Config::StandardFilterElementSize * sizeof(uint32_t));

/**
 * \brief CAN extended message ID filter element structure.
 *
 *  Common element structure for extended message ID filter element.
 */
struct CanDevice::ExtendedMessageFilterElement
{
	uint32_t id : 29,
			 whichBuffer : 3,
			 mask : 29,
			 enabled : 1;
};

static_assert(sizeof(CanDevice::ExtendedMessageFilterElement) == CanDevice::Config::ExtendedFilterElementSize * sizeof(uint32_t));

// Virtual registers, shared between the two cores
uint32_t regError = 0;											// error register. CAN or's bits into it, main proc clears it

// The following configuration registers are written by the main processor while CAN is disabled, and never changed while CAN is enabled
unsigned int rxFifo0Size;										// number of entries in fifo 0
unsigned int rxFifo1Size;										// number of entries in fifo 1
volatile void *rxFifo0Addr;										// fifo 0 start address
volatile void *rxFifo1Addr;										// fifo 1 start address
volatile void *rxBuffersAddr;									// dedicated receive buffers start address
unsigned int txFifoSize;										// number of entries in transmit fifo
volatile void *txFifoAddr;										// transmit fifo address
unsigned int numShortFilterElements;
unsigned int numExtendedFilterElements;
CanDevice::StandardMessageFilterElement *shortFiltersAddr;		// start address of short filter elements
CanDevice::ExtendedMessageFilterElement *extendedFiltersAddr;	// start address of short filter elements

// The following are written by CAN
uint32_t rxBuffersReceivedMap;									// bitmap of dedicated receive buffers containing received messages
unsigned int rxFifo0PutIndex;
unsigned int rxFifo1PutIndex;
unsigned int txFifoGetIndex;

// The following are written by main processor
unsigned int rxFifo0GetIndex;
unsigned int rxFifo1GetIndex;
unsigned int txFifoPutIndex;

// Bit assignments in the message received via the inter-processor fifo
constexpr uint32_t recdFifo0 = 0x01;
constexpr uint32_t recdFifo1 = 0x02;
constexpr uint32_t recdBuff = 0x04;
constexpr uint32_t txDone = 0x08;
constexpr uint32_t rxOvfFifo0 = 0x10;
constexpr uint32_t rxOvfFifo1 = 0x20;

bool canEnabled;												// CAN enable/disable flag, set by main proc to enable CAN after writing other registers, cleared by main proc to disable CAN

// Initialise a CAN device and return a pointer to it
/*static*/ CanDevice* CanDevice::Init(unsigned int p_whichCan, unsigned int p_whichPort, const Config& p_config, uint32_t *memStart, const CanTiming &timing, TxEventCallbackFunction p_txCallback) noexcept
{
	if (   p_whichCan >= NumCanDevices									// device number out of range
		|| p_whichPort != 0												// CAN instance number out of range
	   )
	{
		return nullptr;
	}

	CanDevice& dev = devices[0];
	if (dev.inUse)														// device instance already in use
	{
		return nullptr;
	}

	// Set up device number, peripheral number, hardware address etc.
	dev.inUse = true;
	dev.config = &p_config;

	// Set up pointers to the individual parts of the buffer memory
	memset(memStart, 0, p_config.GetMemorySize());						// clear out filters, transmit pending flags etc.

	dev.rxStdFilter = (StandardMessageFilterElement*)memStart;
	memStart += p_config.GetStandardFiltersMemSize();
	dev.rxExtFilter = (ExtendedMessageFilterElement*)memStart;
	memStart += p_config.GetExtendedFiltersMemSize();
	dev.rx0Fifo = memStart;
	memStart += p_config.rxFifo0Size * p_config.GetRxBufferSize();
	dev.rx1Fifo = memStart;
	memStart += p_config.rxFifo1Size * p_config.GetRxBufferSize();
	dev.rxBuffers = memStart;
	memStart += p_config.numRxBuffers * p_config.GetRxBufferSize();
	memStart += p_config.GetTxEventFifoMemSize();
	dev.txBuffers = memStart;

	dev.messagesQueuedForSending = dev.messagesReceived = dev.messagesLost = dev.busOffCount = 0;
#ifdef RTOS
	for (volatile TaskHandle& h : dev.txTaskWaiting) { h = nullptr; }
	for (volatile TaskHandle& h : dev.rxTaskWaiting) { h = nullptr; }
	dev.rxBuffersWaiting = 0;
#endif

	dev.DoHardwareInit();
	return &dev;
}

// Do the low level hardware initialisation
void CanDevice::DoHardwareInit() noexcept
{
	Disable();

	rxFifo0Size = config->rxFifo0Size;										// number of entries
	rxFifo0Addr = rx0Fifo;													// address
	rxFifo1Size = config->rxFifo1Size;										// number of entries
	rxFifo1Addr = rx1Fifo;													// address
	rxBuffersAddr = rxBuffers;												// dedicated buffers start address
	txFifoSize = config->txFifoSize;
	txFifoAddr = txBuffers;													// address of transmit fifo - we have no dedicated Tx buffers
	numShortFilterElements = config->numShortFilterElements;				// number of short filter elements
	shortFiltersAddr = rxStdFilter;											// short filter start address
	numExtendedFilterElements = config->numExtendedFilterElements;			// number of extended filter elements
	extendedFiltersAddr = rxExtFilter;										// extended filter start address

	multicore_fifo_drain();

#ifdef RTOS
	const IRQn_Type irqn = SIO_IRQ_PROC0_IRQn;
	NVIC_DisableIRQ(irqn);
	NVIC_ClearPendingIRQ(irqn);
	irq_set_exclusive_handler(irqn, CAN_Handler);
	NVIC_EnableIRQ(irqn);
#endif
	// Leave the device disabled. Client must call Enable() to enable it after setting up the receive filters.
}


// Stop and free this device and the CAN port it uses
void CanDevice::DeInit() noexcept
{
	if (inUse)
	{
		Disable();
#ifdef RTOS
		NVIC_DisableIRQ(SIO_IRQ_PROC0_IRQn);
#endif
		inUse = false;														// free the device
	}
}

// Enable this device
void CanDevice::Enable() noexcept
{
	canEnabled = true;
}

// Disable this device
void CanDevice::Disable() noexcept
{
	canEnabled = false;
#endif
}

uint32_t CanDevice::GetErrorRegister() const noexcept
{
	return regError;
}

// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
#if 1
	delay(timeout);
	return false;		//transmission disabled for now
#else
#ifndef RTOS
	const uint32_t start = millis();
#endif

	bool bufferFree;
	if (whichBuffer == TxBufferNumber::fifo)
	{
#ifdef RTOS
		bufferFree = !txFifoFull;
		if (!bufferFree && timeout != 0)
		{
			const unsigned int bufferIndex = txFifoPutIndex;
			const uint32_t trigMask = (uint32_t)1 << bufferIndex;

			txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();

			{
				AtomicCriticalSectionLocker lock;
				txFifoNotFullInterruptEnabled = true;
			}

			bufferFree = !txFifoFull;
			// In the following, when we call TaskBase::Take() the Move task sometimes gets woken up early by by the DDA ring
			// Therefore we loop calling Take() until either the call times out or the buffer is free
			while (!bufferFree)
			{
				const bool timedOut = !TaskBase::Take(timeout);
				bufferFree = !txFifoFull;
				if (timedOut)
				{
					break;
				}
			}
			txTaskWaiting[(unsigned int)whichBuffer] = nullptr;

			{
				AtomicCriticalSectionLocker lock;
				txFifoNotFullInterruptEnabled = false;
			}
		}
#else
		do
		{
			bufferFree = !txFifoFull;
		} while (!bufferFree && millis() - start < timeout);
#endif
	}
	return bufferFree;
#endif
}

#if 0	// not currently used

// Return the number of messages waiting to be sent in the transmit FIFO
unsigned int CanDevice::NumTxMessagesPending(TxBufferNumber whichBuffer) noexcept
{
	return READBITS(hw, TXBC, TFQS) - READBITS(hw, TXFQS, TFFL);
}

#endif

// Queue a message for sending via a buffer or FIFO. If the buffer isn't free, cancel the previous message (or oldest message in the fifo) and send it anyway.
// On return the caller must free or re-use the buffer.
uint32_t CanDevice::SendMessage(TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
#if 1
	return 0;	// transmission disabled for now
#else
	uint32_t cancelledId = 0;
	const bool bufferFree = IsSpaceAvailable(whichBuffer, timeout);
	const uint32_t bufferIndex = txFifoPutIndex;
	if (!bufferFree)
	{
		// Retrieve details of the packet we are about to cancel
		cancelledId = GetTxBuffer(bufferIndex)->T0.bit.ID;
		// Cancel transmission of the oldest packet
		cancelTransmission = true;
		do
		{
			delay(1);
		}
		while (txFifoFull);
	}

	CopyMessageForTransmit(buffer, GetTxBuffer(bufferIndex));
	++txFifoPutIndex;
	return cancelledId;
#endif
}

void CanDevice::CopyReceivedMessage(CanMessageBuffer *null buffer, const volatile RxBufferHeader *f) noexcept
{
	// The CAN has written the message directly to memory, so we must invalidate the cache before we read it
	Cache::InvalidateAfterDMAReceive(f, sizeof(RxBufferHeader) + 64);						// flush the header and up to 64 bytes of data

	if (buffer != nullptr)
	{
		buffer->extId = f->R0.bit.XTD;
		buffer->id.SetReceivedId((buffer->extId) ? f->R0.bit.ID : f->R0.bit.ID >> 18);			// a standard identifier is stored into ID[28:18]
		buffer->remote = f->R0.bit.RTR;

		const volatile uint32_t *data = f->GetDataPointer();
		buffer->timeStamp = f->R1.bit.RXTS;
		const uint8_t dlc = f->R1.bit.DLC;
		static constexpr uint8_t dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

		switch (dlc)
		{
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			buffer->msg.raw32[1] = data[1];
			// no break
		case 0:
		case 1:
		case 2:
		case 3:
			buffer->msg.raw32[0] = data[0];
			buffer->dataLength = dlc;
			break;

		case 15:		// 64 bytes
			buffer->msg.raw32[12] = data[12];
			buffer->msg.raw32[13] = data[13];
			buffer->msg.raw32[14] = data[14];
			buffer->msg.raw32[15] = data[15];
			// no break
		case 14:		// 48 bytes
			buffer->msg.raw32[8] = data[8];
			buffer->msg.raw32[9] = data[9];
			buffer->msg.raw32[10] = data[10];
			buffer->msg.raw32[11] = data[11];
			// no break
		case 13:		// 32 bytes
			buffer->msg.raw32[6] = data[6];
			buffer->msg.raw32[7] = data[7];
			// no break
		case 12:		// 24 bytes
			buffer->msg.raw32[5] = data[5];
			// no break
		case 11:		// 20 bytes
			buffer->msg.raw32[4] = data[4];
			// no break
		case 10:		// 16 bytes
			buffer->msg.raw32[3] = data[3];
			// no break
		case 9:			// 12 bytes
			buffer->msg.raw32[0] = data[0];
			buffer->msg.raw32[1] = data[1];
			buffer->msg.raw32[2] = data[2];
			buffer->dataLength = dlc2len[dlc];
		}
  }

	++messagesReceived;
}

// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
bool CanDevice::ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *null buffer) noexcept
{
#ifndef RTOS
	const uint32_t start = millis();
#endif

	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		{
			// Check for a received message and wait if necessary
#ifdef RTOS
			if (rxFifo0GetIndex == rxFifo0PutIndex)
			{
				if (timeout == 0)
				{
					return false;
				}
				TaskBase::ClearNotifyCount();
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				const bool success = (rxFifo0GetIndex != rxFifo0PutIndex) || (TaskBase::Take(timeout), rxFifo0GetIndex != rxFifo0PutIndex);
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while (rxFifo0GetIndex == rxFifo0PutIndex)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			// Process the received message into the buffer
			CopyReceivedMessage(buffer, GetRxFifo0Buffer(rxFifo1GetIndex));

			// Tell the hardware that we have taken the message
			++rxFifo1GetIndex;
		}
		return true;

	case RxBufferNumber::fifo1:
		// Check for a received message and wait if necessary
		{
#ifdef RTOS
			if (rxFifo1GetIndex == rxFifo1PutIndex)
			{
				if (timeout == 0)
				{
					return false;
				}
				TaskBase::ClearNotifyCount();
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				const bool success = (rxFifo1GetIndex != rxFifo1PutIndex) || (TaskBase::Take(timeout), rxFifo1GetIndex != rxFifo1PutIndex);
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while (rxFifo1GetIndex == rxFifo1PutIndex)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			// Process the received message into the buffer
			const uint32_t getIndex = rxFifo1GetIndex;
			CopyReceivedMessage(buffer, GetRxFifo1Buffer(getIndex));

			// Tell the hardware that we have taken the message
			++rxFifo1GetIndex;
		}
		return true;

	default:
		if ((uint32_t)whichBuffer < (uint32_t)RxBufferNumber::buffer0 + config->numRxBuffers)
		{
			// Check for a received message and wait if necessary
			// We assume that not more than 32 dedicated receive buffers have been configured, so we only need to look at the NDAT1 register
			const uint32_t bufferNumber = (unsigned int)whichBuffer - (unsigned int)RxBufferNumber::buffer0;
			const uint32_t ndatMask = (uint32_t)1 << bufferNumber;
#ifdef RTOS
			if (rxBuffersReceivedMap == 0)
			{
				if (timeout == 0)
				{
					return false;
				}
				TaskBase::ClearNotifyCount();
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				rxBuffersWaiting |= ndatMask;
				const bool success = (rxBuffersReceivedMap & ndatMask) != 0 || (TaskBase::Take(timeout), (rxBuffersReceivedMap & ndatMask) != 0);
				rxBuffersWaiting &= ~ndatMask;
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while (rxBuffersReceivedMap & ndatMask) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			// Process the received message into the buffer
			CopyReceivedMessage(buffer, GetRxBuffer(bufferNumber));

			// Tell the hardware that we have taken the message
			//TODO critical section
			hw_clear_bits(&rxBuffersReceivedMap, ndatMask);
			return true;
		}
		return false;
	}
}

bool CanDevice::IsMessageAvailable(RxBufferNumber whichBuffer) noexcept
{
	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		return rxFifo0GetIndex != rxFifo0PutIndex;
	case RxBufferNumber::fifo1:
		return rxFifo1GetIndex != rxFifo1PutIndex;
	default:
		// We assume that not more than 32 dedicated receive buffers have been configured, so we only need to look at the NDAT1 register
		return (rxBuffersReceivedMap & ((uint32_t)1 << ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0))) != 0;
	}
}

// Disable a short ID filter element
void CanDevice::DisableShortFilterElement(unsigned int index) noexcept
{
	if (index < config->numShortFilterElements)
	{
		rxStdFilter[index].enabled = false;
	}
}

// Set a short ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetShortFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numShortFilterElements)
	{
		rxStdFilter[index].enabled = false;
		rxStdFilter[index].id = id;
		rxStdFilter[index].mask = mask;
		rxStdFilter[index].whichBuffer = (uint32_t)whichBuffer;
		rxStdFilter[index].enabled = true;
	}
}

// Disable an extended ID filter element
void CanDevice::DisableExtendedFilterElement(unsigned int index) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
		rxExtFilter[index].enabled = false;									// disable filter
	}
}

// Set an extended ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetExtendedFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
		rxExtFilter[index].enabled = false;
		rxExtFilter[index].id = id;
		rxExtFilter[index].mask = mask;
		rxExtFilter[index].whichBuffer = (uint32_t)whichBuffer;
		rxExtFilter[index].enabled = true;
	}
}

void CanDevice::GetLocalCanTiming(CanTiming &timing) noexcept
{
#if 1
	timing.SetDefaults_1Mb();
#else
	const uint32_t localNbtp = hw->REG(NBTP);
	const uint32_t tseg1 = (localNbtp & CAN_(NBTP_NTSEG1_Msk)) >> CAN_(NBTP_NTSEG1_Pos);
	const uint32_t tseg2 = (localNbtp & CAN_(NBTP_NTSEG2_Msk)) >> CAN_(NBTP_NTSEG2_Pos);
	const uint32_t jw = (localNbtp & CAN_(NBTP_NSJW_Msk)) >> CAN_(NBTP_NSJW_Pos);
	const uint32_t brp = (localNbtp & CAN_(NBTP_NBRP_Msk)) >> CAN_(NBTP_NBRP_Pos);
	timing.period = (tseg1 + tseg2 + 3) * (brp + 1);
	timing.tseg1 = (tseg1 + 1) * (brp + 1);
	timing.jumpWidth = (jw + 1) * (brp + 1);
#endif
}

void CanDevice::SetLocalCanTiming(const CanTiming &timing) noexcept
{
#if 0
	UpdateLocalCanTiming(timing);				// set up nbtp and dbtp variables
	Disable();
	hw->REG(NBTP) = nbtp;
	hw->REG(DBTP) = dbtp;
	Enable();
#endif
}

#if 0
void CanDevice::UpdateLocalCanTiming(const CanTiming &timing) noexcept
{
	// Sort out the bit timing
	uint32_t period = timing.period;
	uint32_t tseg1 = timing.tseg1;
	uint32_t jumpWidth = timing.jumpWidth;
	uint32_t prescaler = 1;						// 48MHz main clock
	uint32_t tseg2;

	for (;;)
	{
		tseg2 = period - tseg1 - 1;
		if (tseg1 <= 32 && tseg2 <= 16 && jumpWidth <= 16)
		{
			break;
		}

		// Currently we always use a prescaler that is a power of 2, but we could be more general
		prescaler <<= 1;
		period >>= 1;
		tseg1 >>= 1;
		jumpWidth >>= 1;
	}

#if !SAME70
	bitPeriod = period * prescaler;				// the actual CAN normal bit period, in 48MHz clocks
#endif

	nbtp = ((tseg1 - 1) << CAN_(NBTP_NTSEG1_Pos))
		| ((tseg2 - 1) << CAN_(NBTP_NTSEG2_Pos))
		| ((jumpWidth - 1) << CAN_(NBTP_NSJW_Pos))
		| ((prescaler - 1) << CAN_(NBTP_NBRP_Pos));

	// The fast data rate defaults to the same timing
	dbtp = ((tseg1 - 1) << CAN_(DBTP_DTSEG1_Pos))
		| ((tseg2 - 1) << CAN_(DBTP_DTSEG2_Pos))
		| ((jumpWidth - 1) << CAN_(DBTP_DSJW_Pos))
		| ((prescaler - 1) << CAN_(DBTP_DBRP_Pos));
}
#endif

void CanDevice::GetAndClearStats(unsigned int& rMessagesQueuedForSending, unsigned int& rMessagesReceived, unsigned int& rMessagesLost, unsigned int& rBusOffCount) noexcept
{
	AtomicCriticalSectionLocker lock;

	rMessagesQueuedForSending = messagesQueuedForSending;
	rMessagesReceived = messagesReceived;
	rMessagesLost = messagesLost;
	rBusOffCount = busOffCount;
	messagesQueuedForSending = messagesReceived = messagesLost = busOffCount = 0;
}

#ifdef RTOS

void CanDevice::Interrupt() noexcept
{
	while (multicore_fifo_rvalid())
	{
		const uint32_t ir = sio_hw->fifo_rd;

		// Test whether messages have been received into fifo 0
		constexpr unsigned int rxFifo0WaitingIndex = (unsigned int)RxBufferNumber::fifo0;
		if (ir & recdFifo0)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo0WaitingIndex]);
		}

		// Test whether messages have been received into fifo 1
		constexpr unsigned int rxFifo1WaitingIndex = (unsigned int)RxBufferNumber::fifo1;
		if (ir & recdFifo1)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo1WaitingIndex]);
		}

		// Test whether messages have been received into receive buffers
		if (ir & recdBuff)
		{
			// Check which receive buffers have new messages
			uint32_t newData;
			while (((newData = rxBuffersReceivedMap) & rxBuffersWaiting) != 0)
			{
				const unsigned int rxBufferNumber = LowestSetBit(newData);
				rxBuffersWaiting &= ~((uint32_t)1 << rxBufferNumber);
				const unsigned int waitingIndex = rxBufferNumber + (unsigned int)RxBufferNumber::buffer0;
				if (waitingIndex < ARRAY_SIZE(rxTaskWaiting))
				{
					TaskBase::GiveFromISR(rxTaskWaiting[waitingIndex]);
				}
			}
		}

		// Test whether any messages have been transmitted
		if (ir & txDone)
		{
			TaskBase::GiveFromISR(txTaskWaiting[(unsigned int)TxBufferNumber::fifo]);
		}

		// Test for messages lost due to receive fifo full
		if (ir & (rxOvfFifo0 | rxOvfFifo1))
		{
			++messagesLost;
		}
	}
}

// Interrupt handlers

void CAN_Handler() noexcept
{
	devices[0].Interrupt();
}

#endif

// End

