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

CanDevice CanDevice::devices[NumCanDevices];

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
	dev.txCallback = p_txCallback;

#if 0
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
	dev.txEventFifo = (TxEvent*)memStart;
	memStart += p_config.GetTxEventFifoMemSize();
	dev.txBuffers = memStart;

	dev.useFDMode = (p_config.dataSize > 8);							// assume we want standard CAN if the max data size is 8
#endif
	dev.messagesQueuedForSending = dev.messagesReceived = dev.messagesLost = dev.busOffCount = 0;
#if 0
#ifdef RTOS
	for (volatile TaskHandle& h : dev.txTaskWaiting) { h = nullptr; }
	for (volatile TaskHandle& h : dev.rxTaskWaiting) { h = nullptr; }
	dev.rxBuffersWaiting = 0;
#endif

	dev.UpdateLocalCanTiming(timing);									// sets NBTP and DBTP

	dev.DoHardwareInit();
#endif
	return &dev;
}

#if 0
// get bits 2..15 of an address
static inline uint32_t Bits2to15(const volatile void *addr) noexcept
{
	return reinterpret_cast<uint32_t>(addr) & 0x0000FFFC;
}

// Do the low level hardware initialisation
void CanDevice::DoHardwareInit() noexcept
{
	Disable();

	if (useFDMode)
	{
		hw->REG(CCCR) |= CAN_(CCCR_FDOE) | CAN_(CCCR_BRSE);
	}
	else
	{
		hw->REG(CCCR) &= ~(CAN_(CCCR_FDOE) | CAN_(CCCR_BRSE));
	}

	hw->REG(CCCR) |= CAN_(CCCR_TXP);										// enable transmit pause

#if SAME5x || SAMC21
	hw->MRCFG.reg = CAN_MRCFG_QOS_MEDIUM;
#endif
	hw->REG(TDCR) = 0;														// use just the measured transceiver delay
	hw->REG(NBTP) = nbtp;
	hw->REG(DBTP) = dbtp;
	hw->REG(RXF0C) = 														// configure receive FIFO 0
		  (0 << CAN_(RXF0C_F0OM_Pos))										// blocking mode not overwrite mode
		| CAN_(RXF0C_F0WM)(0)												// no watermark interrupt
		| CAN_(RXF0C_F0S)(config->rxFifo0Size)								// number of entries
		| Bits2to15(rx0Fifo);												// address - don't use CAN_(RXF0C_F0SA) here, it is defined strangely on the SAME70
	hw->REG(RXF1C) = 														// configure receive FIFO 1
		  (0 << CAN_(RXF1C_F1OM_Pos))										// blocking mode not overwrite mode
		| CAN_(RXF1C_F1WM)(0)												// no watermark interrupt
		| CAN_(RXF1C_F1S)(config->rxFifo1Size)								// number of entries
		| Bits2to15(rx1Fifo);												// address - don't use CAN_(RXF0C_F1SA) here, it is defined strangely on the SAME70
	hw->REG(RXBC) = Bits2to15(rxBuffers);									// dedicated buffers start address - don't use CAN_(RXBC_RBSA) here, it is defined strangely on the SAME70

	const uint32_t dataSizeCode = (config->dataSize <= 24) ? (config->dataSize >> 2) - 2 : (config->dataSize >> 4) + 3;
	hw->REG(RXESC) = CAN_(RXESC_F0DS)(dataSizeCode)							// receive fifo 0 data size
					| CAN_(RXESC_F1DS)(dataSizeCode)						// receive fifo 1 data size
					| CAN_(RXESC_RBDS)(dataSizeCode);						// receive buffer data size
	hw->REG(TXESC) = CAN_(TXESC_TBDS)(dataSizeCode);						// transmit buffer data size
	hw->REG(TXBC) = 														// configure transmit buffers
		  (0 << CAN_(TXBC_TFQM_Pos))										// FIFO not queue
		| CAN_(TXBC_TFQS)(config->txFifoSize)								// number of Tx fifo entries
		| CAN_(TXBC_NDTB)(config->numTxBuffers)								// number of dedicated Tx buffers
		| Bits2to15(txBuffers);												// address - don't use CAN_(TXBC_TBSA) here, it is defined strangely on the SAME70
	hw->REG(TXEFC) =  														// configure Tx event fifo
		  CAN_(TXEFC_EFWM)(0)												// no watermark interrupt
		| CAN_(TXEFC_EFS)(config->txEventFifoSize)							// event FIFO size
		| Bits2to15(txEventFifo);											// address - don't use CAN_(TXEFC_EFSA) here, it is defined strangely on the SAME70
	hw->REG(GFC) =
#if SAME70
		  MCAN_GFC_ANFE(2)													// reject non-matching frames extended
		| MCAN_GFC_ANFS(2)													// reject non-matching frames standard
#else
		  CAN_(GFC_ANFS_REJECT)
		| CAN_(GFC_ANFE_REJECT)
#endif
		| CAN_(GFC_RRFS)
		| CAN_(GFC_RRFE);
	hw->REG(SIDFC) = CAN_(SIDFC_LSS)(config->numShortFilterElements)		// number of short filter elements
					| Bits2to15(rxStdFilter);								// short filter start address - don't use CAN_(SIDFC_FLSSA) here, it is defined strangely on the SAME70
	hw->REG(XIDFC) = CAN_(XIDFC_LSE)(config->numExtendedFilterElements)		// number of extended filter elements
					| Bits2to15(rxExtFilter);								// extended filter start address - don't use CAN_(SIDFC_FLESA) here, it is defined strangely on the SAME70
	hw->REG(XIDAM) = 0x1FFFFFFF;

	hw->REG(IR) = 0xFFFFFFFF;												// clear all interrupt sources

	// Set up the timestamp counter
#if SAME70
	// The datasheet says that when using CAN-FD, the external timestamp counter must be used, which is TC0. So TC0 must be the step clock lower 16 bits on SAME70 boards.
	hw->MCAN_TSCC = MCAN_TSCC_TSS_EXT_TIMESTAMP;
#else
	hw->TSCC.reg = CAN_TSCC_TSS_INC | CAN_TSCC_TCP(0);						// run timestamp counter at CAN bit speed
#endif

#ifdef RTOS
	const IRQn irqn = IRQnsByPort[whichPort];
	NVIC_DisableIRQ(irqn);
	NVIC_ClearPendingIRQ(irqn);

	hw->REG(ILS) = 0;														// all interrupt sources assigned to interrupt line 0 for now
	statusMask =         CAN_(IR_RF0N)  | CAN_(IR_RF1N)  | CAN_(IR_DRX)  | CAN_(IR_TC)  | CAN_(IR_BO)  | CAN_(IR_RF0L)  | CAN_(IR_RF1L);
	uint32_t intEnable = CAN_(IE_RF0NE) | CAN_(IE_RF1NE) | CAN_(IE_DRXE) | CAN_(IE_TCE) | CAN_(IE_BOE) | CAN_(IE_RF0LE) | CAN_(IE_RF1LE);
	if (txCallback != nullptr)
	{
		intEnable |= CAN_(IE_TEFNE);
		statusMask |= CAN_(IR_TEFN);
	}
	hw->REG(IE) = intEnable;												// enable the interrupt sources that we want
	hw->REG(ILE) = CAN_(ILE_EINT0);											// enable interrupt line 0

	NVIC_EnableIRQ(irqn);
#else
	hw->REG(IE) = 0;														// disable all interrupt sources
	hw->REG(ILE) = 0;
#endif
	// Leave the device disabled. Client must call Enable() to enable it after setting up the receive filters.
}

#endif

// Set the extended ID mask. May only be used while the interface is disabled.
void CanDevice::SetExtendedIdMask(uint32_t mask) noexcept
{
#if 0
	hw->REG(XIDAM) = mask;
#endif
}

// Stop and free this device and the CAN port it uses
void CanDevice::DeInit() noexcept
{
	if (inUse)
	{
		Disable();
#if 0
		NVIC_DisableIRQ(IRQnsByPort[whichPort]);
#endif
		inUse = false;														// free the device
	}
}

// Enable this device
void CanDevice::Enable() noexcept
{
#if 0
	hw->REG(CCCR) &= ~CAN_(CCCR_INIT);
	while ((hw->REG(CCCR) & CAN_(CCCR_INIT)) != 0) { }
#endif
}

// Disable this device
void CanDevice::Disable() noexcept
{
#if 0
	hw->REG(CCCR) |= CAN_(CCCR_INIT);
	while ((hw->REG(CCCR) & CAN_(CCCR_INIT)) == 0) { }
	hw->REG(CCCR) |= CAN_(CCCR_CCE);
#endif
}

// Drain the Tx event fifo. Can use this instead of supplying a Tx event callback in Init() if we don't expect many events.
void CanDevice::PollTxEventFifo(TxEventCallbackFunction p_txCallback) noexcept
{
#if 0
	uint32_t txefs;
	while (((txefs = hw->REG(TXEFS)) & CAN_(TXEFS_EFFL_Msk)) != 0)
	{
		const uint32_t index = (txefs & CAN_(TXEFS_EFGI_Msk)) >> CAN_(TXEFS_EFGI_Pos);
		const volatile TxEvent* const elem = GetTxEvent(index);
		if (elem->R1.bit.ET == 1)
		{
			CanId id;
			id.SetReceivedId(elem->R0.bit.ID);
			p_txCallback(elem->R1.bit.MM, id, elem->R1.bit.TXTS);
		}
		hw->REG(TXEFA) = index;
		__DSB();					// probably not needed, but just in case
	}
#endif
}

uint32_t CanDevice::GetErrorRegister() const noexcept
{
#if 1
	return 0;
#else
	return hw->REG(ECR);
#endif
}

// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
#if 1
	return false;
#else
#ifndef RTOS
	const uint32_t start = millis();
#endif

	bool bufferFree;
	if (whichBuffer == TxBufferNumber::fifo)
	{
#ifdef RTOS
		bufferFree = (READBITS(hw, TXFQS, TFQF) == 0);
		if (!bufferFree && timeout != 0)
		{
			const unsigned int bufferIndex = READBITS(hw, TXFQS, TFQPI);
			const uint32_t trigMask = (uint32_t)1 << bufferIndex;

			txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();

			{
				AtomicCriticalSectionLocker lock;
				hw->REG(TXBTIE) |= trigMask;
			}

			bufferFree = (READBITS(hw, TXFQS, TFQF) == 0);
			// In the following, when we call TaskBase::Take() the Move task sometimes gets woken up early by by the DDA ring
			// Therefore we loop calling Take() until either the call times out or the buffer is free
			while (!bufferFree)
			{
				const bool timedOut = !TaskBase::Take(timeout);
				bufferFree = (READBITS(hw, TXFQS, TFQF) == 0);
				if (timedOut)
				{
					break;
				}
			}
			txTaskWaiting[(unsigned int)whichBuffer] = nullptr;

			{
				AtomicCriticalSectionLocker lock;
				hw->REG(TXBTIE) &= ~trigMask;
			}
		}
#else
		do
		{
			bufferFree = READBITS(hw, TXFQS, TFQF) == 0;
		} while (!bufferFree && millis() - start < timeout);
#endif
	}
	else
	{
		const unsigned int bufferIndex = (unsigned int)whichBuffer - (unsigned int)TxBufferNumber::buffer0;
		const uint32_t trigMask = (uint32_t)1 << bufferIndex;
#ifdef RTOS
		bufferFree = (hw->REG(TXBRP) & trigMask) == 0;
		if (!bufferFree && timeout != 0)
		{
			txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();
			{
				AtomicCriticalSectionLocker lock;
				hw->REG(TXBTIE) |= trigMask;
			}
			bufferFree = (hw->REG(TXBRP) & trigMask) == 0;

			// In the following, when we call TaskBase::Take() assume that the task may get woken up early
			// Therefore we loop calling Take() until either the call times out or the buffer is free
			while (!bufferFree)
			{
				const bool timedOut = !TaskBase::Take(timeout);
				bufferFree = (hw->REG(TXBRP) & trigMask) == 0;
				if (timedOut)
				{
					break;
				}
			}
			txTaskWaiting[(unsigned int)whichBuffer] = nullptr;
			{
				AtomicCriticalSectionLocker lock;
				hw->REG(TXBTIE) &= ~trigMask;
			}
		}
#else
		do
		{
			bufferFree = (hw->REG(TXBRP) & trigMask) == 0;
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
	if (whichBuffer == TxBufferNumber::fifo)
	{
		return READBITS(hw, TXBC, TFQS) - READBITS(hw, TXFQS, TFFL);
	}

	const unsigned int bufferIndex = (unsigned int)whichBuffer - (unsigned int)TxBufferNumber::buffer0;
	return (hw->REG(TXBRP) >> bufferIndex) & 1u;
}

#endif

// Queue a message for sending via a buffer or FIFO. If the buffer isn't free, cancel the previous message (or oldest message in the fifo) and send it anyway.
// On return the caller must free or re-use the buffer.
uint32_t CanDevice::SendMessage(TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	uint32_t cancelledId = 0;
#if 0
	if ((uint32_t)whichBuffer < (uint32_t)TxBufferNumber::buffer0 + config->numTxBuffers)
	{
		const bool bufferFree = IsSpaceAvailable(whichBuffer, timeout);
		const uint32_t bufferIndex = (whichBuffer == TxBufferNumber::fifo)
										? READBITS(hw, TXFQS, TFQPI)
											: (uint32_t)whichBuffer - (uint32_t)TxBufferNumber::buffer0;
		const uint32_t trigMask = (uint32_t)1 << bufferIndex;
		if (!bufferFree)
		{
			// Retrieve details of the packet we are about to cancel
			cancelledId = GetTxBuffer(bufferIndex)->T0.bit.ID;
			// Cancel transmission of the oldest packet
			hw->REG(TXBCR) = trigMask;
			do
			{
				delay(1);
			}
			while ((hw->REG(TXBRP) & trigMask) != 0 || (whichBuffer == TxBufferNumber::fifo && READBITS(hw, TXFQS, TFQF)));
		}

		CopyMessageForTransmit(buffer, GetTxBuffer(bufferIndex));
		__DSB();								// this is needed on the SAME70, otherwise incorrect data sometimes gets transmitted
		hw->REG(TXBAR) = trigMask;
	}
#endif
	return cancelledId;
}

// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
bool CanDevice::ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *null buffer) noexcept
{
#if 1
	delay(timeout);
	return false;
#else
#ifndef RTOS
	const uint32_t start = millis();
#endif

	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		{
			// Check for a received message and wait if necessary
#ifdef RTOS
			if (READBITS(hw, RXF0S, F0FL) == 0)
			{
				if (timeout == 0)
				{
					return false;
				}
				TaskBase::ClearNotifyCount();
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				const bool success = (READBITS(hw, RXF0S, F0FL) != 0) || (TaskBase::Take(timeout), READBITS(hw, RXF0S, F0FL) != 0);
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while (READBITS(hw, RXF0S, F0FL) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			// Process the received message into the buffer
			const uint32_t getIndex = READBITS(hw, RXF0S, F0GI);
			CopyReceivedMessage(buffer, GetRxFifo0Buffer(getIndex));

			// Tell the hardware that we have taken the message
			WRITEBITS(hw, RXF0A, F0AI, getIndex);
		}
		return true;

	case RxBufferNumber::fifo1:
		// Check for a received message and wait if necessary
		{
#ifdef RTOS
			if (READBITS(hw, RXF1S, F1FL) == 0)
			{
				if (timeout == 0)
				{
					return false;
				}
				TaskBase::ClearNotifyCount();
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				const bool success = (READBITS(hw, RXF1S, F1FL) != 0) || (TaskBase::Take(timeout), READBITS(hw, RXF1S, F1FL) != 0);
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while (READBITS(hw, RXF1S, F1FL) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			// Process the received message into the buffer
			const uint32_t getIndex = READBITS(hw, RXF1S, F1GI);
			CopyReceivedMessage(buffer, GetRxFifo1Buffer(getIndex));

			// Tell the hardware that we have taken the message
			WRITEBITS(hw, RXF1A, F1AI, getIndex);
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
			if ((hw->REG(NDAT1) & ndatMask) == 0)
			{
				if (timeout == 0)
				{
					return false;
				}
				TaskBase::ClearNotifyCount();
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				rxBuffersWaiting |= ndatMask;
				const bool success = (hw->REG(NDAT1) & ndatMask) != 0 || (TaskBase::Take(timeout), (hw->REG(NDAT1) & ndatMask) != 0);
				rxBuffersWaiting &= ~ndatMask;
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while ((hw->REG(NDAT1) & ndatMask) == 0)
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
			hw->REG(NDAT1) = ndatMask;
			return true;
		}
		return false;
	}
#endif
}

bool CanDevice::IsMessageAvailable(RxBufferNumber whichBuffer) noexcept
{
#if 1
	return false;
#else
	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		return READBITS(hw, RXF0S, F0FL) != 0;
	case RxBufferNumber::fifo1:
		return READBITS(hw, RXF1S, F1FL) != 0;
	default:
		// We assume that not more than 32 dedicated receive buffers have been configured, so we only need to look at the NDAT1 register
		return (hw->REG(NDAT1) & ((uint32_t)1 << ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0))) != 0;
	}
#endif
}

// Disable a short ID filter element
void CanDevice::DisableShortFilterElement(unsigned int index) noexcept
{
	if (index < config->numShortFilterElements)
	{
#if 0
		rxStdFilter[index].S0.val = 0;
#endif
	}
}

// Set a short ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetShortFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numShortFilterElements)
	{
#if 0
		StandardMessageFilterElement::S0Type s0;
		s0.val = 0;										// disable filter, clear reserved fields
		s0.bit.SFID1 = id;
		s0.bit.SFT = 0x02;							// classic filter
		switch (whichBuffer)
		{
		case RxBufferNumber::fifo0:
			s0.bit.SFEC = 0x01;						// store in FIFO 0
			s0.bit.SFID2 = mask;
			break;
		case RxBufferNumber::fifo1:
			s0.bit.SFEC = 0x02;						// store in FIFO 1
			s0.bit.SFID2 = mask;
			break;
		default:
			if ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0 < config->numRxBuffers)
			{
				s0.bit.SFEC = 0x07;					// store in buffer
				s0.bit.SFID2 = (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0;
			}
			else
			{
				s0.bit.SFEC = 0x00;					// discard message
				s0.bit.SFID2 = mask;
			}
			break;
		}
		rxStdFilter[index].S0.val = s0.val;
#endif
	}
}

// Disable an extended ID filter element
void CanDevice::DisableExtendedFilterElement(unsigned int index) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
#if 0
		rxExtFilter[index].F0.val = 0;									// disable filter
#endif
	}
}

// Set an extended ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetExtendedFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
#if 0
		volatile ExtendedMessageFilterElement& efp = rxExtFilter[index];
		efp.F0.val = 0;									// disable filter

		ExtendedMessageFilterElement::F0Type f0;
		ExtendedMessageFilterElement::F1Type f1;
		f0.val = 0;									// clear all fields
		f1.val = 0;									// clear all fields
		f1.bit.EFT  = 0x02;							// classic filter
		f0.bit.EFID1 = id;
		switch (whichBuffer)
		{
		case RxBufferNumber::fifo0:
			f0.bit.EFEC = 0x01;						// store in FIFO 0
			f1.bit.EFID2 = mask;
			break;
		case RxBufferNumber::fifo1:
			f0.bit.EFEC = 0x02;						// store in FIFO 1
			f1.bit.EFID2 = mask;
			break;
		default:
			if ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0 < config->numRxBuffers)
			{
				f0.bit.EFEC = 0x07;
				f1.bit.EFID2 = (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0;
			}
			else
			{
				f0.bit.EFEC = 0x00;					// discard message
				f1.bit.EFID2 = mask;
			}
			break;
		}

		efp.F1.val = f1.val;						// update second word first while the filter is disabled
		efp.F0.val = f0.val;						// update first word and enable filter
#endif
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

#if 0
void CanDevice::Interrupt() noexcept
{
	uint32_t ir;
	while ((ir = hw->REG(IR) & statusMask) != 0)
	{
		hw->REG(IR) = ir;

		constexpr unsigned int rxFifo0WaitingIndex = (unsigned int)RxBufferNumber::fifo0;
		if ((ir & CAN_(IR_RF0N)) != 0)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo0WaitingIndex]);
		}

		constexpr unsigned int rxFifo1WaitingIndex = (unsigned int)RxBufferNumber::fifo1;
		if ((ir & CAN_(IR_RF1N)) != 0)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo1WaitingIndex]);
		}

		if (ir & CAN_(IR_DRX))
		{
			// Check which receive buffers have new messages
			uint32_t newData;
			while (((newData = hw->REG(NDAT1)) & rxBuffersWaiting) != 0)
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

		if (ir & CAN_(IR_TC))
		{
			// Check which transmit buffers have finished transmitting
			uint32_t transmitDone;
			while ((transmitDone = (~hw->REG(TXBRP)) & hw->REG(TXBTIE)) != 0)
			{
				const unsigned int bufferNumber = LowestSetBit(transmitDone);
				hw->REG(TXBTIE) &= ~((uint32_t)1 << bufferNumber);
				if (bufferNumber < READBITS(hw, TXBC, NDTB))
				{
					// Completed transmission from a dedicated transmit buffer
					const unsigned int waitingIndex = bufferNumber + (unsigned int)TxBufferNumber::buffer0;
					if (waitingIndex < ARRAY_SIZE(txTaskWaiting))
					{
						TaskBase::GiveFromISR(txTaskWaiting[waitingIndex]);
					}
				}
				else
				{
					// Completed transmission from a transmit FIFO buffer
					TaskBase::GiveFromISR(txTaskWaiting[(unsigned int)TxBufferNumber::fifo]);
				}
			}
		}

		if (ir & CAN_(IR_BO))
		{
			++busOffCount;
			DoHardwareInit();
			Enable();
			return;
		}

		if (ir & (CAN_(IR_RF0L) | CAN_(IR_RF1L)))
		{
			++messagesLost;
		}

		if (ir & CAN_(IR_TEFN))
		{
			uint32_t txefs;
			while (((txefs = hw->REG(TXEFS)) & CAN_(TXEFS_EFFL_Msk)) != 0)
			{
				const uint32_t index = (txefs & CAN_(TXEFS_EFGI_Msk)) >> CAN_(TXEFS_EFGI_Pos);
				const TxEvent* elem = GetTxEvent(index);
				if (elem->R1.bit.ET == 1)
				{
					CanId id;
					id.SetReceivedId(elem->R0.bit.ID);
					txCallback(elem->R1.bit.MM, id, elem->R1.bit.TXTS);
				}
				hw->REG(TXEFA) = index;
			}
		}
	}
}

// Interrupt handlers

void CAN0_Handler() noexcept
{
	devicesByPort[0]->Interrupt();
}

void CAN1_Handler() noexcept
{
	devicesByPort[1]->Interrupt();
}

#endif
#endif

#endif
