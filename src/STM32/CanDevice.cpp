/*
 * CanDevice.cpp
 *
 *  Created on: 8 Jan 2022
 *  Author: Andy
 */

#include "CanDevice.h"

#if SUPPORT_CAN && STM32H7
#include <CoreImp.h>
#include <Cache.h>
#include <CoreNotifyIndices.h>
#include <CanSettings.h>
#include <CanMessageBuffer.h>
#include <General/Bitmap.h>
#include <cstring>
#include <HardwareTimer.h>

// On the STM32H7 we use the built in CAN FD module
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

static FDCAN_GlobalTypeDef * const CanInstance[2] = {FDCAN1, FDCAN2};
static const IRQn_Type IRQnsByPort[2][2] = { {FDCAN1_IT0_IRQn, FDCAN1_IT1_IRQn}, {FDCAN2_IT0_IRQn, FDCAN2_IT1_IRQn} };
static CanDevice *devicesByPort[2] = { nullptr, nullptr };
static Can *hwByPort[2] = {nullptr, nullptr};
CanDevice CanDevice::devices[NumCanDevices];

// Clear statistics
void CanDevice::CanStats::Clear() noexcept
{
	messagesQueuedForSending = messagesReceived = messagesLost = protocolErrors = busOffCount = 0;
}

// Initialise a CAN device and return a pointer to it
/*static*/ CanDevice* CanDevice::Init(unsigned int p_whichCan, unsigned int p_whichPort, const Config& p_config, uint32_t *memStart, const CanTiming &timing, TxEventCallbackFunction p_txCallback, Pin ReadPin, Pin WritePin) noexcept
{
	if (   p_whichCan >= NumCanDevices									// device number out of range
		|| p_whichPort >= 2												// CAN instance number out of range
		|| devicesByPort[p_whichPort] != nullptr						// CAN instance number already in use
	   )
	{
		return nullptr;
	}

	CanDevice& dev = devices[p_whichCan];
	if (dev.hw.Instance != nullptr)												// device instance already in use
	{
		return nullptr;
	}
	//debugPrintf("Init Can device %d port %d\n", p_whichCan, p_whichPort);
	// Set up device number, peripheral number, hardware address etc.
	dev.whichCan = p_whichCan;
	dev.whichPort = p_whichPort;
	dev.config = &p_config;
	dev.txCallback = p_txCallback;
	devicesByPort[p_whichPort] = &dev;
	hwByPort[p_whichPort] = &dev.hw;
	uint32_t dataSize = 0;
	if (p_config.dataSize == 64)
		dataSize = FDCAN_DATA_BYTES_64;
	else if (p_config.dataSize == 8)
		dataSize = FDCAN_DATA_BYTES_8;
	else
	{
		dataSize = FDCAN_DATA_BYTES_64;
		debugPrintf("Unxepected Can data size %d\n", p_config.dataSize);
	}

	dev.hw.Instance = CanInstance[p_whichPort];
	FDCAN_InitTypeDef& Init = dev.hw.Init;
	Init.FrameFormat = (dataSize != FDCAN_DATA_BYTES_8 ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_CLASSIC);
	Init.Mode = FDCAN_MODE_NORMAL;
	Init.AutoRetransmission = ENABLE;
	Init.TransmitPause = ENABLE;
	Init.ProtocolException = ENABLE;
	dev.UpdateLocalCanTiming(timing);
	Init.MessageRAMOffset = 0;
	Init.StdFiltersNbr = p_config.numShortFilterElements;
	Init.ExtFiltersNbr = p_config.numExtendedFilterElements;
	Init.RxFifo0ElmtsNbr = p_config.rxFifo0Size;
	Init.RxFifo0ElmtSize = dataSize;
	Init.RxFifo1ElmtsNbr = p_config.rxFifo1Size;
	Init.RxFifo1ElmtSize = dataSize;
	Init.RxBuffersNbr = p_config.numRxBuffers;
	Init.RxBufferSize = dataSize;
	Init.TxEventsNbr = p_config.txEventFifoSize;
	Init.TxBuffersNbr = p_config.numTxBuffers;
	Init.TxFifoQueueElmtsNbr = p_config.txFifoSize;
	Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	Init.TxElmtSize = dataSize;

	dev.stats.Clear();

	// setup the hardware
	__HAL_RCC_FDCAN_CLK_ENABLE();
	pinmap_pinout(ReadPin, PinMap_CAN_RD);
	pinmap_pinout(WritePin, PinMap_CAN_TD);

	HAL_StatusTypeDef status = HAL_FDCAN_Init(&dev.hw);
	if (status != HAL_OK)
	{
		debugPrintf("FDCAN init failed %x\n", status);
		return nullptr;
	}
	for(uint32_t i = 0; i < Init.StdFiltersNbr; i++)
		dev.DisableShortFilterElement(i);
	for(uint32_t i = 0; i < Init.ExtFiltersNbr; i++)
		dev.DisableExtendedFilterElement(i);
	status = HAL_FDCAN_ConfigGlobalFilter(&dev.hw, FDCAN_REJECT, FDCAN_REJECT,
													FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	if (status != HAL_OK)
	{
		debugPrintf("Failed to set global filter %x\n", status);
	}
	HAL_FDCAN_ConfigRxFifoOverwrite(&dev.hw, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_BLOCKING);
	HAL_FDCAN_ConfigRxFifoOverwrite(&dev.hw, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_BLOCKING);
	HAL_FDCAN_ConfigFifoWatermark(&dev.hw, FDCAN_CFG_RX_FIFO0, 0);
	HAL_FDCAN_ConfigFifoWatermark(&dev.hw, FDCAN_CFG_RX_FIFO1, 0);
	HAL_FDCAN_ConfigFifoWatermark(&dev.hw, FDCAN_CFG_TX_EVENT_FIFO, 0);
	// Use one CAN bit time as the basis for our timestamps
	status = HAL_FDCAN_ConfigTimestampCounter(&dev.hw, FDCAN_TIMESTAMP_PRESC_1);
	if (status != HAL_OK)
	{
		debugPrintf("FDCAN failed to set t/s prescaler %x\n", status);
	}
	// and enable it using the internal counter
	status = HAL_FDCAN_EnableTimestampCounter(&dev.hw, FDCAN_TIMESTAMP_INTERNAL);
	if (status != HAL_OK)
	{
		debugPrintf("FDCAN failed to enable t/s counter %x\n", status);
	}
	// all interrupts go via int line 0
	status = HAL_FDCAN_ConfigInterruptLines(&dev.hw, FDCAN_IT_TX_COMPLETE|FDCAN_IT_RX_BUFFER_NEW_MESSAGE|FDCAN_IT_TX_EVT_FIFO_NEW_DATA|
													 FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO1_NEW_MESSAGE|FDCAN_IT_BUS_OFF|
													 FDCAN_IT_RX_FIFO0_MESSAGE_LOST|FDCAN_IT_RX_FIFO1_MESSAGE_LOST, 
													 FDCAN_INTERRUPT_LINE0);
	if (status != HAL_OK)
	{
		debugPrintf("FDCAN failed to configure interrupts %x\n", status);
	}
	// Enable the tx complete notification, buffer selection happens later
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_TX_COMPLETE, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_RX_FIFO1_MESSAGE_LOST, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&dev.hw, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_DisableTxDelayCompensation(&dev.hw);
#ifdef RTOS
	HAL_NVIC_EnableIRQ(IRQnsByPort[p_whichPort][0]);
	HAL_NVIC_EnableIRQ(IRQnsByPort[p_whichPort][1]);
#endif
	return &dev;
}

// get bits 2..15 of an address
static inline uint32_t Bits2to15(const volatile void *addr) noexcept
{
	return reinterpret_cast<uint32_t>(addr) & 0x0000FFFC;
}

// Do the low level hardware initialisation
void CanDevice::DoHardwareInit() noexcept
{

}

// Set the extended ID mask. May only be used while the interface is disabled.
void CanDevice::SetExtendedIdMask(uint32_t mask) noexcept
{
	HAL_StatusTypeDef status = HAL_FDCAN_ConfigExtendedIdMask(&hw, mask);
	if (status != HAL_OK)
		debugPrintf("Failed to set extended ID mask %x\n", status);
}

// Stop and free this device and the CAN port it uses
void CanDevice::DeInit() noexcept
{
	if (hw.Instance != nullptr)
	{
		Disable();
		HAL_NVIC_DisableIRQ(IRQnsByPort[whichPort][0]);
		HAL_NVIC_DisableIRQ(IRQnsByPort[whichPort][1]);
		HAL_FDCAN_DeInit(&hw);
		__HAL_RCC_FDCAN_FORCE_RESET();
		__HAL_RCC_FDCAN_RELEASE_RESET();
		devicesByPort[whichPort] = nullptr;									// free the port
		hw.Instance = nullptr;												// free the device
	}
}

// Enable this device
void CanDevice::Enable() noexcept
{
	if (hw.Instance != nullptr)
	{
		HAL_StatusTypeDef status = HAL_FDCAN_Start(&hw);
		if (status != HAL_OK)
			debugPrintf("Failed to enable can device %x\n", status);
	}
	else
		debugPrintf("hw is null\n");
}

// Disable this device
void CanDevice::Disable() noexcept
{
	if (hw.Instance != nullptr)
		HAL_FDCAN_Stop(&hw);
}

// Drain the Tx event fifo. Can use this instead of supplying a Tx event callback in Init() if we don't expect many events.
void CanDevice::PollTxEventFifo(TxEventCallbackFunction p_txCallback) noexcept
{
	FDCAN_TxEventFifoTypeDef elem;
	while(HAL_FDCAN_GetTxEvent(&hw, &elem) == HAL_OK)
	{
		CanId id;
		id.SetReceivedId(elem.Identifier);
		p_txCallback(elem.MessageMarker, id, elem.TxTimestamp);
	}
}

uint32_t CanDevice::GetErrorRegister() const noexcept
{
	return 0;
}

// Return true if space is available to send using this buffer or FIFO
bool CanDevice::IsSpaceAvailable(TxBufferNumber whichBuffer, uint32_t timeout) noexcept
{
#ifndef RTOS
	const uint32_t start = millis();
#endif

	bool bufferFree;
	if (whichBuffer == TxBufferNumber::fifo)
	{
#ifdef RTOS
		bufferFree = HAL_FDCAN_GetTxFifoFreeLevel(&hw) != 0;
		if (!bufferFree && timeout != 0)
		{
			// Get next buffer to be removed
			const unsigned int bufferIndex = ((hw.Instance->TXFQS & FDCAN_TXFQS_TFGI) >> FDCAN_TXFQS_TFGI_Pos);
			const uint32_t trigMask = (uint32_t)1 << bufferIndex;
			txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();

			{
				AtomicCriticalSectionLocker lock;
	  			SET_BIT(hw.Instance->TXBTIE, trigMask);
			}

			bufferFree = HAL_FDCAN_GetTxFifoFreeLevel(&hw) != 0;
			// In the following, when we call TaskBase::Take() the Move task sometimes gets woken up early by by the DDA ring
			// Therefore we loop calling Take() until either the call times out or the buffer is free
			while (!bufferFree)
			{
				const bool timedOut = !TaskBase::TakeIndexed(NotifyIndices::CanDevice, timeout);
				bufferFree = HAL_FDCAN_GetTxFifoFreeLevel(&hw) != 0;
				if (timedOut)
				{
					break;
				}
			}
			txTaskWaiting[(unsigned int)whichBuffer] = nullptr;

			{
				AtomicCriticalSectionLocker lock;
				CLEAR_BIT(hw.Instance->TXBTIE, trigMask);
			}
		}
#else
		do
		{
			bufferFree = HAL_FDCAN_GetTxFifoFreeLevel(&hw) != 0;
		} while (!bufferFree && millis() - start < timeout);
#endif
	}
	else
	{
		const unsigned int bufferIndex = (unsigned int)whichBuffer - (unsigned int)TxBufferNumber::buffer0;
#ifdef RTOS
		const uint32_t trigMask = (uint32_t)1 << bufferIndex;
		bufferFree = HAL_FDCAN_IsTxBufferMessagePending(&hw, trigMask) == 0;
		if (!bufferFree && timeout != 0)
		{
			//debugPrintf("Wait space available buffer %d timeout %d\n", whichBuffer, timeout);
			txTaskWaiting[(unsigned int)whichBuffer] = TaskBase::GetCallerTaskHandle();
			{
				AtomicCriticalSectionLocker lock;
				SET_BIT(hw.Instance->TXBTIE, trigMask);
			}
			bufferFree = HAL_FDCAN_IsTxBufferMessagePending(&hw, trigMask) == 0;

			// In the following, when we call TaskBase::Take() assume that the task may get woken up early
			// Therefore we loop calling Take() until either the call times out or the buffer is free
			while (!bufferFree)
			{
				const bool timedOut = !TaskBase::TakeIndexed(NotifyIndices::CanDevice, timeout);
				bufferFree = HAL_FDCAN_IsTxBufferMessagePending(&hw, trigMask) == 0;
				if (timedOut)
				{
					//debugPrintf("Space available timeout\n");
					break;
				}
			}
			txTaskWaiting[(unsigned int)whichBuffer] = nullptr;
			{
				AtomicCriticalSectionLocker lock;
				CLEAR_BIT(hw.Instance->TXBTIE, trigMask);
			}
		}
#else
		do
		{
			bufferFree = HAL_FDCAN_IsTxBufferMessagePending(&hw, bufferIndex) == 0;
		} while (!bufferFree && millis() - start < timeout);
#endif
	}
	return bufferFree;
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



static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static const uint8_t BytesToDLC[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 9, 10, 10, 10, 10, 
									 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 
									 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
									 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};

// Queue a message for sending via a buffer or FIFO. If the buffer isn't free, cancel the previous message (or oldest message in the fifo) and send it anyway.
// On return the caller must free or re-use the buffer.
uint32_t CanDevice::SendMessage(TxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
	uint32_t cancelledId = 0;
	//debugPrintf("Send message buffer %d num Tx %d\n", whichBuffer, config->numTxBuffers);
	if ((uint32_t)whichBuffer < (uint32_t)TxBufferNumber::buffer0 + config->numTxBuffers)
	{
		const bool bufferFree = IsSpaceAvailable(whichBuffer, timeout);
		const uint32_t bufferIndex = (whichBuffer == TxBufferNumber::fifo)
										? ((hw.Instance->TXFQS & FDCAN_TXFQS_TFGI) >> FDCAN_TXFQS_TFGI_Pos)
											: (uint32_t)whichBuffer - (uint32_t)TxBufferNumber::buffer0;
		const uint32_t trigMask = (uint32_t)1 << bufferIndex;
		if (!bufferFree)
		{
			//debugPrintf("about to cancel message\n");
			// Retrieve details of the packet we are about to cancel
			// FIXME can we get the ID ?
			cancelledId = 1;
			HAL_FDCAN_AbortTxRequest(&hw, trigMask);		
			do
			{
				delay(1);
			}
			while (HAL_FDCAN_IsTxBufferMessagePending(&hw, trigMask) != 0 || (whichBuffer == TxBufferNumber::fifo && HAL_FDCAN_GetTxFifoFreeLevel(&hw) == 0));
			//debugPrintf("Cancel complete\n");
		}
		FDCAN_TxHeaderTypeDef hdr;
		hdr.Identifier = buffer->id.GetWholeId();
		hdr.IdType = (buffer->extId ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID);
		hdr.TxFrameType = (buffer->remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME);
		uint32_t dataLen = buffer->dataLength;
		uint32_t dlcLen = BytesToDLC[dataLen];
		hdr.DataLength = dlcLen;
		dlcLen = DLCtoBytes[dlcLen];
		while (dataLen < dlcLen)
		{
			buffer->msg.raw[dataLen++] = 0;				// zero fill up to the CANFD buffer length
		}
		hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		hdr.BitRateSwitch = (buffer->useBrs ? FDCAN_BRS_ON : FDCAN_BRS_OFF);
		hdr.FDFormat = (buffer->fdMode ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN);
		hdr.TxEventFifoControl = (buffer->reportInFifo ? FDCAN_STORE_TX_EVENTS : FDCAN_NO_TX_EVENTS);
		hdr.MessageMarker = buffer->marker;
		HAL_StatusTypeDef status;
		if (whichBuffer == TxBufferNumber::fifo)
		{
			status = HAL_FDCAN_AddMessageToTxFifoQ(&hw, &hdr, buffer->msg.raw);
			HAL_FDCAN_EnableTxBufferRequest(&hw, HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hw));
		}
		else
		{
			status = HAL_FDCAN_AddMessageToTxBuffer(&hw, &hdr, buffer->msg.raw, trigMask);
			HAL_FDCAN_EnableTxBufferRequest(&hw, trigMask);
		}

		if (status != HAL_OK)
		{
			debugPrintf("Failed to send CAN message %x %x\n", status, (unsigned)hw.ErrorCode);
		}
		else
			++stats.messagesQueuedForSending;
		__DSB();								// this is needed on the SAME70, otherwise incorrect data sometimes gets transmitted
	}
	return cancelledId;
}


void CanDevice::CopyHeader(CanMessageBuffer *buffer, FDCAN_RxHeaderTypeDef *hdr) noexcept
{
	buffer->extId = (hdr->IdType == FDCAN_EXTENDED_ID ? 1 : 0);
	buffer->id.SetReceivedId(hdr->Identifier);
	buffer->remote = (hdr->RxFrameType == FDCAN_REMOTE_FRAME ? 1 : 0);
	buffer->timeStamp = hdr->RxTimestamp;
	buffer->dataLength = DLCtoBytes[hdr->DataLength];
	++stats.messagesReceived;
}

// Following function is a replacement for HAL_FDCAN_IsRxBufferMessageAvailable we do not want
// to clear the data present flags after we have read them.
static uint32_t FDCAN_IsRxBufferMessageAvailable(FDCAN_HandleTypeDef *hfdcan, uint32_t RxBufferIndex)
{
  /* Check function parameters */
  assert_param(IS_FDCAN_MAX_VALUE(RxBufferIndex, 63U));
  uint32_t NewData1 = hfdcan->Instance->NDAT1;
  uint32_t NewData2 = hfdcan->Instance->NDAT2;

  /* Check new message reception on the selected buffer */
  if (((RxBufferIndex < 32U) && ((NewData1 & (uint32_t)((uint32_t)1 << RxBufferIndex)) == 0U)) ||
      ((RxBufferIndex >= 32U) && ((NewData2 & (uint32_t)((uint32_t)1 << (RxBufferIndex & 0x1FU))) == 0U)))
  {
    return 0;
  }
  return 1;
}


// Receive a message in a buffer or fifo, with timeout. Returns true if successful, false if no message available even after the timeout period.
bool CanDevice::ReceiveMessage(RxBufferNumber whichBuffer, uint32_t timeout, CanMessageBuffer *buffer) noexcept
{
#ifndef RTOS
	const uint32_t start = millis();
#endif
	FDCAN_RxHeaderTypeDef hdr;
	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		{
			// Check for a received message and wait if necessary
#ifdef RTOS
			if (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO0) == 0)
			{
				if (timeout == 0)
				{
					return false;
				}
				TaskBase::ClearCurrentTaskNotifyCount(NotifyIndices::CanDevice);
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				const bool success = (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO0) != 0) || (TaskBase::TakeIndexed(NotifyIndices::CanDevice, timeout), HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO0) != 0);
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					//debugPrintf("Fifo 0 timeout\n");
					return false;
				}
			}
#else
			while (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO0) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			//debugPrintf("FIFO0 attempt read\n");
			HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&hw, FDCAN_RX_FIFO0, &hdr, buffer->msg.raw);
			if (status != HAL_OK)
			{
				debugPrintf("Failed to read fifo0 %x\n", status);
			}
			CopyHeader(buffer, &hdr);
		}
		return true;

	case RxBufferNumber::fifo1:
		// Check for a received message and wait if necessary
		{
#ifdef RTOS
			if (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO1) == 0)
			{
				if (timeout == 0)
				{
					//debugPrintf("Fifo 1 timeout\n");
					return false;
				}
				TaskBase::ClearCurrentTaskNotifyCount(NotifyIndices::CanDevice);
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				const bool success = (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO1) != 0) || (TaskBase::TakeIndexed(NotifyIndices::CanDevice, timeout), HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO1) != 0);
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while (HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO1) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			//debugPrintf("FIFO1 attempt read\n");
			HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&hw, FDCAN_RX_FIFO1, &hdr, buffer->msg.raw);
			if (status != HAL_OK)
			{
				debugPrintf("Failed to read fifo1 %x\n", status);
			}
			CopyHeader(buffer, &hdr);
		}
		return true;

	default:
		if ((uint32_t)whichBuffer < (uint32_t)RxBufferNumber::buffer0 + config->numRxBuffers)
		{
			// Check for a received message and wait if necessary
			// We assume that not more than 32 dedicated receive buffers have been configured, so we only need to look at the NDAT1 register
			const uint32_t bufferNumber = (unsigned int)whichBuffer - (unsigned int)RxBufferNumber::buffer0;
#ifdef RTOS
			const uint32_t ndatMask = (uint32_t)1 << bufferNumber;
			if (FDCAN_IsRxBufferMessageAvailable(&hw, bufferNumber) == 0)
			{
				if (timeout == 0)
				{
					debugPrintf("Buffer %d timeout\n", (int)bufferNumber);
					return false;
				}
				TaskBase::ClearCurrentTaskNotifyCount(NotifyIndices::CanDevice);
				const unsigned int waitingIndex = (unsigned int)whichBuffer;
				rxTaskWaiting[waitingIndex] = TaskBase::GetCallerTaskHandle();
				rxBuffersWaiting |= ndatMask;
				const bool success = (FDCAN_IsRxBufferMessageAvailable(&hw, bufferNumber) != 0 || (TaskBase::TakeIndexed(NotifyIndices::CanDevice, timeout), FDCAN_IsRxBufferMessageAvailable(&hw, bufferNumber) != 0));
				rxBuffersWaiting &= ~ndatMask;
				rxTaskWaiting[waitingIndex] = nullptr;
				if (!success)
				{
					return false;
				}
			}
#else
			while (FDCAN_IsRxBufferMessageAvailable(&hw, bufferNumber) == 0)
			{
				if (millis() - start >= timeout)
				{
					return false;
				}
			}
#endif
			//debugPrintf("Attempt to read from buffer %d\n", bufferNumber);
			HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&hw, bufferNumber, &hdr, buffer->msg.raw);
			if (status != HAL_OK)
			{
				debugPrintf("Failed to read buffer %x\n", status);
			}
			CopyHeader(buffer, &hdr);
			return true;
		}
		return false;
	}
	delay(100);
	return false;
}

bool CanDevice::IsMessageAvailable(RxBufferNumber whichBuffer) noexcept
{
	switch (whichBuffer)
	{
	case RxBufferNumber::fifo0:
		return HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO0) != 0;

	case RxBufferNumber::fifo1:
		return HAL_FDCAN_GetRxFifoFillLevel(&hw, FDCAN_RX_FIFO1) != 0;

	default:
		// We assume that not more than 32 dedicated receive buffers have been configured, so we only need to look at the NDAT1 register
		return FDCAN_IsRxBufferMessageAvailable(&hw, (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0) != 0;
	}
	return false;
}

// Disable a short ID filter element
void CanDevice::DisableShortFilterElement(unsigned int index) noexcept
{
	if (index < config->numShortFilterElements)
	{
		FDCAN_FilterTypeDef efd;
		efd.IdType = FDCAN_STANDARD_ID;
		efd.FilterIndex = index;
		efd.FilterType = FDCAN_FILTER_MASK;
		efd.FilterID1 = 0;
		efd.IsCalibrationMsg = 0;
		efd.FilterConfig = FDCAN_FILTER_DISABLE;
		efd.FilterID2 = 0;
		HAL_StatusTypeDef status = HAL_FDCAN_ConfigFilter(&hw, &efd);
		if (status != HAL_OK)
		{
			debugPrintf("Failed to disable extended filter %x\n", status);
		}
	}
}

// Set a short ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetShortFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numShortFilterElements)
	{
		FDCAN_FilterTypeDef efd;
		efd.IdType = FDCAN_STANDARD_ID;
		efd.FilterIndex = index;
		efd.FilterType = FDCAN_FILTER_MASK;
		efd.FilterID1 = id;
		efd.IsCalibrationMsg = 0;
		switch (whichBuffer)
		{
		case RxBufferNumber::fifo0:
			efd.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
			efd.FilterID2 = mask;
			break;
		case RxBufferNumber::fifo1:
			efd.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
			efd.FilterID2 = mask;
			break;
		default:
			if ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0 < config->numRxBuffers)
			{
				efd.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
				efd.FilterID2 = (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0;
			}
			else
			{
				efd.FilterConfig = FDCAN_FILTER_DISABLE;
				efd.FilterID2 = mask;
			}
			break;
		}
		HAL_StatusTypeDef status = HAL_FDCAN_ConfigFilter(&hw, &efd);
		if (status != HAL_OK)
		{
			debugPrintf("Failed to configure standard filter %x\n", status);
		}
	}
}

// Disable an extended ID filter element
void CanDevice::DisableExtendedFilterElement(unsigned int index) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
		FDCAN_FilterTypeDef efd;
		efd.IdType = FDCAN_EXTENDED_ID;
		efd.FilterIndex = index;
		efd.FilterType = FDCAN_FILTER_MASK;
		efd.FilterID1 = 0;
		efd.IsCalibrationMsg = 0;
		efd.FilterConfig = FDCAN_FILTER_DISABLE;
		efd.FilterID2 = 0;
		HAL_StatusTypeDef status = HAL_FDCAN_ConfigFilter(&hw, &efd);
		if (status != HAL_OK)
		{
			debugPrintf("Failed to disable extended filter %x\n", status);
		}

	}
}

// Set an extended ID field filter element. To disable the filter element, use a zero mask parameter.
// If whichBuffer is a buffer number not a fifo number, the mask field is ignored except that a zero mask disables the filter element; so only the XIDAM mask filters the ID.
void CanDevice::SetExtendedFilterElement(unsigned int index, RxBufferNumber whichBuffer, uint32_t id, uint32_t mask) noexcept
{
	if (index < config->numExtendedFilterElements)
	{
		//debugPrintf("Add filter index %d buffer %d id %x mask %x\n", index, whichBuffer, id, mask);
		FDCAN_FilterTypeDef efd;
		efd.IdType = FDCAN_EXTENDED_ID;
		efd.FilterIndex = index;
		efd.FilterType = FDCAN_FILTER_MASK;
		efd.FilterID1 = id;
		efd.IsCalibrationMsg = 0;
		switch (whichBuffer)
		{
		case RxBufferNumber::fifo0:
			efd.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
			efd.FilterID2 = mask;
			break;
		case RxBufferNumber::fifo1:
			efd.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
			efd.FilterID2 = mask;
			break;
		default:
			if ((uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0 < config->numRxBuffers)
			{
				efd.FilterConfig = FDCAN_FILTER_TO_RXBUFFER;
				efd.FilterID2 = (uint32_t)whichBuffer - (uint32_t)RxBufferNumber::buffer0;
			}
			else
			{
				efd.FilterConfig = FDCAN_FILTER_DISABLE;
				efd.FilterID2 = mask;
			}
			break;
		}
		HAL_StatusTypeDef status = HAL_FDCAN_ConfigFilter(&hw, &efd);
		if (status != HAL_OK)
		{
			debugPrintf("Failed to configure extended filter %x\n", status);
		}
	}
}

void CanDevice::GetLocalCanTiming(CanTiming &timing) const noexcept
{

	const uint32_t localNbtp = hw.Instance->NBTP;
	const uint32_t tseg1 = (localNbtp & FDCAN_NBTP_NTSEG1_Msk) >> FDCAN_NBTP_NTSEG1_Pos;
	const uint32_t tseg2 = (localNbtp & FDCAN_NBTP_NTSEG2_Msk) >> FDCAN_NBTP_NTSEG2_Pos;
	const uint32_t jw = (localNbtp & FDCAN_NBTP_NSJW_Msk) >> FDCAN_NBTP_NSJW_Pos;
	const uint32_t brp = (localNbtp & FDCAN_NBTP_NBRP_Msk) >> FDCAN_NBTP_NBRP_Pos;
	timing.period = (tseg1 + tseg2 + 3) * (brp + 1);
	timing.tseg1 = (tseg1 + 1) * (brp + 1);
	timing.jumpWidth = (jw + 1) * (brp + 1);
}

void CanDevice::SetLocalCanTiming(const CanTiming &timing) noexcept
{
	UpdateLocalCanTiming(timing);					// set up nbtp and dbtp variables
	Disable();
	hw.Instance->NBTP = ((((uint32_t)hw.Init.NominalSyncJumpWidth - 1U) << FDCAN_NBTP_NSJW_Pos) |
							(((uint32_t)hw.Init.NominalTimeSeg1 - 1U) << FDCAN_NBTP_NTSEG1_Pos)	|
							(((uint32_t)hw.Init.NominalTimeSeg2 - 1U) << FDCAN_NBTP_NTSEG2_Pos)	|
							(((uint32_t)hw.Init.NominalPrescaler - 1U) << FDCAN_NBTP_NBRP_Pos));

	hw.Instance->DBTP = ((((uint32_t)hw.Init.DataSyncJumpWidth - 1U) << FDCAN_DBTP_DSJW_Pos) |
							  (((uint32_t)hw.Init.DataTimeSeg1 - 1U) << FDCAN_DBTP_DTSEG1_Pos)	|
							  (((uint32_t)hw.Init.DataTimeSeg2 - 1U) << FDCAN_DBTP_DTSEG2_Pos)	|
							  (((uint32_t)hw.Init.DataPrescaler - 1U) << FDCAN_DBTP_DBRP_Pos));
	Enable();
}

void CanDevice::UpdateLocalCanTiming(const CanTiming &timing) noexcept
{
	// Sort out the bit timing
	uint32_t period = timing.period;
	uint32_t tseg1 = timing.tseg1;
	uint32_t jumpWidth = timing.jumpWidth;
	uint32_t prescaler = 1;							// 48MHz main clock
	uint32_t tseg2;

	// Use the highest prescaled clock frequency we can in order to get the most accurate timing
	for (;;)
	{
		tseg2 = period - tseg1 - 1;
		if (tseg1 <= 256 && tseg2 <= 128)
		{
			break;
		}

		// Currently we always use a prescaler that is a power of 2, but we could be more general
		prescaler <<= 1;
		period >>= 1;
		tseg1 >>= 1;
		jumpWidth >>= 1;
	}

	if (jumpWidth > tseg2) { jumpWidth = tseg2; }	// jump width cannot exceed tseg2
#if !SAME70
	bitPeriod = period * prescaler;					// the actual CAN normal bit period in 48MHz clocks (may be different from timing.period)
#endif

	FDCAN_InitTypeDef& Init = hw.Init;
	Init.NominalPrescaler = prescaler;
	Init.NominalSyncJumpWidth = jumpWidth;
	Init.NominalTimeSeg1 = tseg1;
	Init.NominalTimeSeg2 = tseg2;
	Init.DataPrescaler = prescaler;
	// We don't currently use BRS. For now we default the fast data rate to 2Mbps (or lower if the prescaler is greater than 1) with fixed timing,
	// just to have some sensible values to write to the register.
	constexpr uint32_t fast_period = CanTiming::ClockFrequency/2'000'000;	// 2Mbps divided by the prescaler
	constexpr uint32_t fast_tseg1 = fast_period/2 - 1;						// set sample point to 50%
	constexpr uint32_t fast_tseg2 = fast_period - fast_tseg1 - 1;			// make up the correct period
	constexpr uint32_t fast_jumpWidth = fast_tseg2;							// set jump width to maximum
	Init.DataPrescaler = prescaler;
	Init.DataSyncJumpWidth = fast_jumpWidth;
	Init.DataTimeSeg1 = fast_tseg1;
	Init.DataTimeSeg2 = fast_tseg2;
}

void CanDevice::GetAndClearStats(CanDevice::CanStats& dst) noexcept
{
	AtomicCriticalSectionLocker lock;
	dst = stats;
	stats.Clear();
}

#ifdef RTOS

void CanDevice::Interrupt() noexcept
{
#if 0
	uint32_t ir;
	while ((ir = hw->REG(IR) & statusMask) != 0)
	{
		hw->REG(IR) = ir;

		constexpr unsigned int rxFifo0WaitingIndex = (unsigned int)RxBufferNumber::fifo0;
		if ((ir & CAN_(IR_RF0N)) != 0)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo0WaitingIndex], NotifyIndices::CanDevice);
		}

		constexpr unsigned int rxFifo1WaitingIndex = (unsigned int)RxBufferNumber::fifo1;
		if ((ir & CAN_(IR_RF1N)) != 0)
		{
			TaskBase::GiveFromISR(rxTaskWaiting[rxFifo1WaitingIndex], NotifyIndices::CanDevice);
		}

		if (ir & CAN_(IR_DRX))
		{
			// Check which receive buffers have new messages
			uint32_t newData;
			while ((newData = hw->REG(NDAT1) & rxBuffersWaiting) != 0)
			{
				const unsigned int rxBufferNumber = LowestSetBit(newData);
				rxBuffersWaiting &= ~((uint32_t)1 << rxBufferNumber);
				const unsigned int waitingIndex = rxBufferNumber + (unsigned int)RxBufferNumber::buffer0;
				if (waitingIndex < ARRAY_SIZE(rxTaskWaiting))
				{
					TaskBase::GiveFromISR(rxTaskWaiting[waitingIndex], NotifyIndices::CanDevice);
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
						TaskBase::GiveFromISR(txTaskWaiting[waitingIndex], NotifyIndices::CanDevice);
					}
				}
				else
				{
					// Completed transmission from a transmit FIFO buffer
					TaskBase::GiveFromISR(txTaskWaiting[(unsigned int)TxBufferNumber::fifo], NotifyIndices::CanDevice);
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
			++stats.messagesLost;
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
#endif
}

// Interrupt handlers
extern "C" void FDCAN1_IT0_IRQHandler(void) noexcept
{
  HAL_FDCAN_IRQHandler(hwByPort[0]);
}

extern "C" void FDCAN2_IT0_IRQHandler(void) noexcept
{
  HAL_FDCAN_IRQHandler(hwByPort[1]);
}

extern "C" void FDCAN1_IT1_IRQHandler(void) noexcept
{
  HAL_FDCAN_IRQHandler(hwByPort[0]);
}

extern "C" void FDCAN2_IT1_IRQHandler(void) noexcept
{
  HAL_FDCAN_IRQHandler(hwByPort[1]);
}

extern "C" void FDCAN_CAL_IRQHandler(void) noexcept
{
  HAL_FDCAN_IRQHandler(hwByPort[0]);
}

extern "C" void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
	if ((TxEventFifoITs & FDCAN_IT_TX_EVT_FIFO_NEW_DATA) != 0)
	{
		FDCAN_TxEventFifoTypeDef elem;
		while(HAL_FDCAN_GetTxEvent(hfdcan, &elem) == HAL_OK)
		{
			CanId id;
			id.SetReceivedId(elem.Identifier);
			if (hfdcan == hwByPort[0])
				devicesByPort[0]->txCallback(elem.MessageMarker, id, elem.TxTimestamp);
			else
				devicesByPort[1]->txCallback(elem.MessageMarker, id, elem.TxTimestamp);
		}
	}

}

extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t transmitDone)
{
	CanDevice *dev = devicesByPort[hfdcan == hwByPort[0] ? 0 : 1];
	while (transmitDone != 0)
	{
		const unsigned int bufferNumber = LowestSetBit(transmitDone);
		transmitDone &= ~((uint32_t)1 << bufferNumber);
		if (bufferNumber <= dev->hw.Init.TxBuffersNbr )
		{
			// Completed transmission from a dedicated transmit buffer
			const unsigned int waitingIndex = bufferNumber + (unsigned int)CanDevice::TxBufferNumber::buffer0;
			if (waitingIndex < ARRAY_SIZE(dev->txTaskWaiting))
			{
				TaskBase::GiveFromISR(dev->txTaskWaiting[waitingIndex], NotifyIndices::CanDevice);
			}
		}
		else
		{
			// Completed transmission from a transmit FIFO buffer
			TaskBase::GiveFromISR(dev->txTaskWaiting[(unsigned int)CanDevice::TxBufferNumber::fifo], NotifyIndices::CanDevice);
		}
	}
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	CanDevice *dev = devicesByPort[hfdcan == hwByPort[0] ? 0 : 1];
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
	{
		TaskBase::GiveFromISR(dev->rxTaskWaiting[(unsigned int)CanDevice::RxBufferNumber::fifo0], NotifyIndices::CanDevice);	
	}
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0)
	{
		dev->stats.messagesLost++;
	}
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	CanDevice *dev = devicesByPort[hfdcan == hwByPort[0] ? 0 : 1];
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0)
	{
		TaskBase::GiveFromISR(dev->rxTaskWaiting[(unsigned int)CanDevice::RxBufferNumber::fifo1], NotifyIndices::CanDevice);
	}
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0)
	{
		dev->stats.messagesLost++;
	}
}

extern "C" void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{
	CanDevice *dev = devicesByPort[hfdcan == hwByPort[0] ? 0 : 1];
	uint32_t newData;
	while ((newData = dev->hw.Instance->NDAT1 & dev->rxBuffersWaiting) != 0)
	{
		const unsigned int rxBufferNumber = LowestSetBit(newData);
		dev->rxBuffersWaiting &= ~((uint32_t)1 << rxBufferNumber);
		const unsigned int waitingIndex = rxBufferNumber + (unsigned int)CanDevice::RxBufferNumber::buffer0;
		if (waitingIndex < ARRAY_SIZE(dev->rxTaskWaiting))
		{
			TaskBase::GiveFromISR(dev->rxTaskWaiting[waitingIndex], NotifyIndices::CanDevice);
		}
	}
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	CanDevice *dev = devicesByPort[hfdcan == hwByPort[0] ? 0 : 1];
	if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0)
	{
		dev->stats.busOffCount++;
		dev->Disable();
		dev->Enable();
	}
}
#endif
#endif
