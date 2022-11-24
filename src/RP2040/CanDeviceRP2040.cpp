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
#include "VirtualCanRegisters.h"
#include <cstring>

#undef from
#define PICO_MUTEX_ENABLE_SDK120_COMPATIBILITY	0	// used by mutex.h which is included by multicore.h
#include <pico/multicore.h>
#include <hardware/irq.h>

extern "C" void debugPrintf(const char *fmt, ...) noexcept;

static CanDevice devices[NumCanDevices];

inline uint32_t CanDevice::GetRxBufferSize() const noexcept { return sizeof(CanRxBufferHeader)/sizeof(uint32_t) + (64 >> 2); }
inline uint32_t CanDevice::GetTxBufferSize() const noexcept { return sizeof(CanTxBufferHeader)/sizeof(uint32_t) + (64 >> 2); }
inline CanRxBufferHeader *CanDevice::GetRxFifo0Buffer(uint32_t index) const noexcept { return (CanRxBufferHeader*)(rx0Fifo + (index * GetRxBufferSize())); }
inline CanRxBufferHeader *CanDevice::GetRxFifo1Buffer(uint32_t index) const noexcept { return (CanRxBufferHeader*)(rx1Fifo + (index * GetRxBufferSize())); }
inline CanTxBufferHeader *CanDevice::GetTxBuffer(uint32_t index) const noexcept { return (CanTxBufferHeader*)(txBuffers + (index * GetTxBufferSize())); }

// Virtual registers, shared between the two cores
VirtualCanRegisters virtualRegs;

static bool core1Initialised = false;

extern "C" void CAN_Handler() noexcept;
extern "C" void Core1Entry() noexcept;

// Initialise a CAN device and return a pointer to it
/*static*/ CanDevice* CanDevice::Init(Pin p_txPin, Pin p_rxPin, const Config& p_config, uint32_t *memStart, const CanTiming &timing, TxEventCallbackFunction p_txCallback) noexcept
{
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

	dev.rxStdFilter = (CanStandardMessageFilterElement*)memStart;
	memStart += p_config.GetStandardFiltersMemSize();
	dev.rxExtFilter = (CanExtendedMessageFilterElement*)memStart;
	memStart += p_config.GetExtendedFiltersMemSize();
	dev.rx0Fifo = memStart;
	memStart += (p_config.rxFifo0Size + 1) * p_config.GetRxBufferSize();
	dev.rx1Fifo = memStart;
	memStart += (p_config.rxFifo1Size + 1) * p_config.GetRxBufferSize();
	dev.rxBuffers = memStart;
	memStart += p_config.numRxBuffers * p_config.GetRxBufferSize();
	memStart += p_config.GetTxEventFifoMemSize();
	dev.txBuffers = memStart;

	dev.messagesQueuedForSending = dev.messagesReceived = dev.messagesLost = dev.busOffCount = 0;

	for (unsigned int i = 0; i < p_config.numShortFilterElements; ++i)
	{
		dev.rxStdFilter[i].enabled = false;
	}
	for (unsigned int i = 0; i < p_config.numExtendedFilterElements; ++i)
	{
		dev.rxExtFilter[i].enabled = false;
	}

#ifdef RTOS
	for (volatile TaskHandle& h : dev.txTaskWaiting) { h = nullptr; }
	for (volatile TaskHandle& h : dev.rxTaskWaiting) { h = nullptr; }
	dev.rxBuffersWaiting = 0;
#endif

	virtualRegs.bitrate = 1000000;
	virtualRegs.txPin = p_txPin;
	virtualRegs.rxPin = p_rxPin;

	dev.DoHardwareInit();
	return &dev;
}

// Do the low level hardware initialisation
void CanDevice::DoHardwareInit() noexcept
{
	Disable();			// this also clears some of the virtual registers

	virtualRegs.rxFifos[0].size = config->rxFifo0Size + 1;								// number of entries
	virtualRegs.rxFifos[0].buffers = reinterpret_cast<volatile CanRxBuffer*>(rx0Fifo);	// address
	virtualRegs.rxFifos[1].size = config->rxFifo1Size + 1;								// number of entries
	virtualRegs.rxFifos[1].buffers = reinterpret_cast<volatile CanRxBuffer*>(rx1Fifo);	// address
	virtualRegs.txFifo.size = config->txFifoSize + 1;									// number of entries
	virtualRegs.txFifo.buffers = reinterpret_cast<CanTxBuffer*>(txBuffers);				// address of transmit fifo - we have no dedicated Tx buffers
	virtualRegs.numShortFilterElements = config->numShortFilterElements;				// number of short filter elements
	virtualRegs.shortFiltersAddr = rxStdFilter;											// short filter start address
	virtualRegs.numExtendedFilterElements = config->numExtendedFilterElements;			// number of extended filter elements
	virtualRegs.extendedFiltersAddr = rxExtFilter;										// extended filter start address

	if (!core1Initialised)
	{
		multicore_launch_core1(Core1Entry);
		core1Initialised = true;
	}

	multicore_fifo_drain();

#ifdef RTOS
	const IRQn_Type irqn = SIO_IRQ_PROC0_IRQn;
	NVIC_DisableIRQ(irqn);
	NVIC_ClearPendingIRQ(irqn);
	irq_set_exclusive_handler(irqn, CAN_Handler);
	NVIC_SetPriority(irqn, 3);
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
	virtualRegs.canEnabled = true;
}

// Disable this device
void CanDevice::Disable() noexcept
{
	virtualRegs.Init();
}

// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
#ifndef RTOS
	const uint32_t start = millis();
#endif

	bool bufferFree;
	const unsigned int bufferIndex = virtualRegs.txFifo.putIndex;
	unsigned int nextTxFifoPutIndex = bufferIndex + 1;
	if (nextTxFifoPutIndex == virtualRegs.txFifo.size)
	{
		nextTxFifoPutIndex = 0;
	}

#ifdef RTOS
	bufferFree = nextTxFifoPutIndex != virtualRegs.txFifo.getIndex;
	if (!bufferFree && timeout != 0)
	{
		txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();
		virtualRegs.txFifoNotFullInterruptEnabled = true;

		bufferFree = nextTxFifoPutIndex != virtualRegs.txFifo.getIndex;
		// In the following, when we call TaskBase::Take() the Move task sometimes gets woken up early by by the DDA ring
		// Therefore we loop calling Take() until either the call times out or the buffer is free
		while (!bufferFree)
		{
			const bool timedOut = !TaskBase::Take(timeout);
			bufferFree = nextTxFifoPutIndex != virtualRegs.txFifo.getIndex;
			if (timedOut)
			{
				break;
			}
		}
		txTaskWaiting[(unsigned int)whichBuffer] = nullptr;
		virtualRegs.txFifoNotFullInterruptEnabled = false;
	}
#else
	do
	{
		bufferFree = nextTxFifoPutIndex != txFifoGetIndex;
	} while (!bufferFree && millis() - start < timeout);
#endif
	return bufferFree;
}

#if 0	// not currently used

// Return the number of messages waiting to be sent in the transmit FIFO
unsigned int CanDevice::NumTxMessagesPending(TxBufferNumber whichBuffer) noexcept
{
	return READBITS(hw, TXBC, TFQS) - READBITS(hw, TXFQS, TFFL);
}

#endif

void CanDevice::CopyMessageForTransmit(CanMessageBuffer *buffer, volatile CanTxBufferHeader *f) noexcept
{
	if (buffer->extId)
	{
		f->T0.val = buffer->id.GetWholeId();
		f->T0.bit.XTD = 1;
	}
	else
	{
		/* A standard identifier is stored into ID[28:18] */
		f->T0.val = buffer->id.GetWholeId() << 18;
		f->T0.bit.XTD = 0;
	}

	f->T0.bit.RTR = buffer->remote;

	f->T1.bit.MM = buffer->marker;
	f->T1.bit.EFCbit = buffer->reportInFifo;
	uint32_t dataLength = buffer->dataLength;
	volatile uint32_t *dataPtr = ((CanTxBuffer*)f)->data32;
	if (dataLength <= 8)
	{
		f->T1.bit.DLC = dataLength;
		dataPtr[0] = buffer->msg.raw32[0];
		dataPtr[1] = buffer->msg.raw32[1];
	}
	else
	{
		while (dataLength & 3)
		{
			buffer->msg.raw[dataLength++] = 0;				// pad length to a multiple of 4 bytes, setting any additional bytes we send to zero in case the message ends with a string
		}

		if (dataLength <= 24)
		{
			// DLC values 9, 10, 11, 12 code for lengths 12, 16, 20, 24
			uint8_t dlc = (dataLength >> 2) + 6;
			f->T1.bit.DLC = dlc;
			const uint32_t *p = buffer->msg.raw32;
			do
			{
				*dataPtr++ = *p++;
				--dlc;
			} while (dlc != 6);								// copy 3, 4, 5 or 6 words
		}
		else
		{
			// DLC values 13, 14, 15 code for lengths 32, 48, 64
			while (dataLength & 12)
			{
				buffer->msg.raw32[dataLength >> 2] = 0;		// pad length to a multiple of 16 bytes, setting any additional bytes we send to zero in case the message ends with a string
				dataLength += 4;
			}

			uint8_t dlc = (dataLength >> 4) + 11;
			f->T1.bit.DLC = dlc;
			const uint32_t *p = buffer->msg.raw32;
			do
			{
				*dataPtr++ = *p++;
				*dataPtr++ = *p++;
				*dataPtr++ = *p++;
				*dataPtr++ = *p++;
				--dlc;
			} while (dlc != 11);
		}
	}

	f->T1.bit.FDF = buffer->fdMode;
	f->T1.bit.BRS = buffer->useBrs;

	++messagesQueuedForSending;
}

// Queue a message for sending via a buffer or FIFO. If the buffer isn't free, cancel the previous message (or oldest message in the fifo) and send it anyway.
// On return the caller must free or re-use the buffer.
uint32_t CanDevice::SendMessage(TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	uint32_t cancelledId = 0;
	const bool bufferFree = IsSpaceAvailable(whichBuffer, timeout);
	const unsigned int bufferIndex = virtualRegs.txFifo.putIndex;
	unsigned int nextTxFifoPutIndex = bufferIndex + 1;
	if (nextTxFifoPutIndex == virtualRegs.txFifo.size)
	{
		nextTxFifoPutIndex = 0;
	}

	if (!bufferFree)
	{
		// Retrieve details of the packet we are about to cancel
		unsigned int cancelledIndex = nextTxFifoPutIndex + 1;
		if (cancelledIndex == virtualRegs.txFifo.size)
		{
			cancelledIndex = 0;
		}
		cancelledId = GetTxBuffer(cancelledIndex)->T0.bit.ID;

		// Cancel transmission of the oldest packet
		virtualRegs.cancelTransmission = true;
		do
		{
			delay(1);
		}
		while (nextTxFifoPutIndex == virtualRegs.txFifo.getIndex);
	}

	CopyMessageForTransmit(buffer, GetTxBuffer(bufferIndex));
	virtualRegs.txFifo.putIndex = nextTxFifoPutIndex;
	return cancelledId;
}

void CanDevice::CopyReceivedMessage(CanMessageBuffer *null buffer, const volatile CanRxBufferHeader *f) noexcept
{
	// The CAN has written the message directly to memory, so we must invalidate the cache before we read it
	Cache::InvalidateAfterDMAReceive(f, sizeof(CanRxBuffer));					// flush the header data

	if (buffer != nullptr)
	{
		buffer->extId = f->R0.bit.XTD;
		buffer->id.SetReceivedId(f->R0.bit.ID);
		buffer->remote = false;

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
//		debugPrintf("Rx typ %u src %u dst %u len %u\n", (unsigned int)buffer->id.MsgType(), buffer->id.Src(), buffer->id.Dst(), buffer->dataLength);
//		delay(100);
	}

	++messagesReceived;
}

// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
bool CanDevice::ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *null buffer) noexcept
{
#ifndef RTOS
	const uint32_t start = millis();
#endif

	if ((unsigned int)whichBuffer < NumCanRxFifos)
	{
		// Check for a received message and wait if necessary
		VirtualCanRegisters::RxFifo& fifo = virtualRegs.rxFifos[(unsigned int)whichBuffer];
		unsigned int getIndex = fifo.getIndex;
#ifdef RTOS
		if (getIndex == fifo.putIndex)
		{
			if (timeout == 0)
			{
				return false;
			}
			TaskBase::ClearCurrentTaskNotifyCount();
			const unsigned int waitingIndex = (unsigned int)whichBuffer;
			rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
			const bool success = (getIndex != fifo.putIndex) || (TaskBase::Take(timeout), getIndex != fifo.putIndex);
			rxTaskWaiting[waitingIndex] = nullptr;
			if (!success)
			{
				return false;
			}
		}
#else
		while (getIndex == fifo.putIndex)
		{
			if (millis() - start >= timeout)
			{
				return false;
			}
		}
#endif
		// Process the received message into the buffer
		CopyReceivedMessage(buffer, &fifo.buffers[fifo.getIndex]);

		// Tell the hardware that we have taken the message
		++getIndex;
		if (getIndex == fifo.size)
		{
			getIndex = 0;
		}
		fifo.getIndex = getIndex;
		return true;
	}

	return false;
}

bool CanDevice::IsMessageAvailable(RxBufferNumber whichBuffer) noexcept
{
	if ((unsigned int)whichBuffer < NumCanRxFifos)
	{
		// Check for a received message and wait if necessary
		const VirtualCanRegisters::RxFifo& fifo = virtualRegs.rxFifos[(unsigned int)whichBuffer];
		return fifo.getIndex != fifo.putIndex;
	}

	return false;
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

void CanDevice::GetAndClearErrorCounts(CanErrorCounts& errs) noexcept
{
	errs = virtualRegs.errors;
	virtualRegs.clearErrorCounts = true;
}

#ifdef RTOS

void CanDevice::Interrupt() noexcept
{
	while ((sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS) != 0)
	{
		const uint32_t ir = sio_hw->fifo_rd;

		// Test whether messages have been received into fifo 0
		constexpr unsigned int rxFifo0WaitingIndex = (unsigned int)RxBufferNumber::fifo0;
		if (ir & VirtualCanRegisters::recdFifo0)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo0WaitingIndex]);
		}

		// Test whether messages have been received into fifo 1
		constexpr unsigned int rxFifo1WaitingIndex = (unsigned int)RxBufferNumber::fifo1;
		if (ir & VirtualCanRegisters::recdFifo1)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo1WaitingIndex]);
		}

		// Test whether any messages have been transmitted
		if (ir & VirtualCanRegisters::txFifoNotFull)
		{
			TaskBase::GiveFromISR(txTaskWaiting[(unsigned int)TxBufferNumber::fifo]);
		}
	}
}

// Interrupt handlers

void CAN_Handler() noexcept
{
	devices[0].Interrupt();
}

#endif	// RTOS

#endif	// SUPPORT_CAN

// End

