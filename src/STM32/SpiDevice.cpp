/*
 * SpiDevice.cpp
 *
 *  Created on: 7 May 2022
 *      Author: David
 */

#include <SPI/SpiDevice.h>
#include <CoreNotifyIndices.h>
#include <Serial.h>

constexpr uint32_t DefaultSharedSpiClockFrequency = 2000000;
constexpr uint32_t SpiTimeout = 10000;

static SPI_TypeDef *const SpiDevices[] = {	SPI1, SPI2, SPI3, SPI4,
#ifdef SPI5
											SPI5
#endif
										 };

static IRQn SpiInterruptNumbers[] = {	SPI1_IRQn, SPI2_IRQn, SPI3_IRQn, SPI4_IRQn,
#ifdef SPI5
										SPI5_IRQn
#endif
									};

/*static*/ void SpiDevice::CommonInterrupt(void* param) noexcept
{
	((SpiDevice*)param)->Interrupt();
}

SpiDevice::SpiDevice(const SpiParameters& params) noexcept
	: hardware(SpiDevices[params.instanceNumber - 1]), instanceNumber(params.instanceNumber),
	  dmaChanTx(params.dmaChanTx), dmaChanRx(params.dmaChanRx), dmaPrioTx(params.dmaPrioTx), dmaPrioRx(params.dmaPrioRx)
{
	SetPinMode(params.mosiPin, INPUT_PULLDOWN);
	SetPinMode(params.misoPin, INPUT_PULLDOWN);
	SetPinMode(params.sclkPin, INPUT_PULLDOWN);
	SetPinFunction(params.mosiPin, params.pinFunction);
	SetPinFunction(params.misoPin, params.pinFunction);
	SetPinFunction(params.sclkPin, params.pinFunction);
	SetDriveStrength(params.mosiPin, 2);
	SetDriveStrength(params.sclkPin, 2);								// some devices (e.g. TFT LCD font chip) need fast rise and fall times

	EnableSpiClock(params.instanceNumber);

	// Set up the SPI instance
	Disable();

	// Enable the interrupt in the NVIC
	Serial::SetSpiVector(instanceNumber, CommonInterrupt, this);
	NVIC_SetPriority(SpiInterruptNumbers[instanceNumber - 1], params.irqPriority);
	NVIC_EnableIRQ(SpiInterruptNumbers[instanceNumber - 1]);

	// Leave the SPI disabled until after we have set its comms parameters
}

void SpiDevice::Disable() const noexcept
{
	constexpr uint32_t allIenBits = SPI_IER_MODFIE | SPI_IER_TIFREIE | SPI_IER_CRCEIE | SPI_IER_OVRIE | SPI_IER_UDRIE
									| SPI_IER_TXTFIE | SPI_IER_EOTIE | SPI_IER_DXPIE | SPI_IER_TXPIE | SPI_IER_RXPIE;
	hardware->IER &= ~allIenBits;															// disable all interrupts
	hardware->CR1 &= ~SPI_CR1_SPE;															// disable SPI
	hardware->IFCR |= SPI_IFCR_SUSPC | SPI_IFCR_TIFREC | SPI_IFCR_OVRC | SPI_IFCR_UDRC;		// clear flags
}

void SpiDevice::Enable() const noexcept
{
	hardware->CR1 |= SPI_CR1_SPE;
}

// Wait for transmitter ready returning true if timed out
inline bool SpiDevice::waitForTxReady() const noexcept
{
	uint32_t timeout = SpiTimeout;
	while (!(hardware->SR & SPI_SR_TXP))
	{
		if (--timeout == 0)
		{
			return true;
		}
	}
	return false;
}

// Wait for transmitter empty returning true if timed out
inline bool SpiDevice::waitForTxEmpty() const noexcept
{
	uint32_t timeout = SpiTimeout;
	while (!(hardware->SR & SPI_SR_TXC))
	{
		if (!timeout--)
		{
			return true;
		}
	}
	return false;
}

// Wait for receive data available returning true if timed out
inline bool SpiDevice::waitForRxReady() const noexcept
{
	uint32_t timeout = SpiTimeout;
	while (!(hardware->SR & SPI_SR_RXP))
	{
		if (--timeout == 0)
		{
			return true;
		}
	}
	return false;
}

void SpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode, bool nineBits) const noexcept
{
	// We have to disable SPI device in order to change the baud rate, mode or character length
	Disable();

	// Select the highest bit rate we can that does not exceed the requested bit rate.
	unsigned int prescaleFactor = 0;
	uint32_t actualFreq = GetSpiClockFrequency(instanceNumber);
	while (actualFreq > freq && prescaleFactor < 8)
	{
		actualFreq >>= 1;
		++prescaleFactor;
	}
	uint32_t cfg1 = ((nineBits) ? 8 : 7) << SPI_CFG1_DSIZE_Pos;
	if (prescaleFactor == 0)
	{
		cfg1 |= SPI_CFG1_BPASS;
	}
	else
	{
		cfg1 |= (prescaleFactor - 1) << SPI_CFG1_MBR_Pos;
	}
	hardware->CFG1 = cfg1;			//TODO need to add DMA bits of using DMA

	uint32_t cfg2 = SPI_CFG2_AFCNTR | SPI_CFG2_MASTER;

	if (((uint8_t)mode & 2) != 0)
	{
		cfg2 |= SPI_CFG2_CPOL;
	}
	if (((uint8_t)mode & 1) != 0)
	{
		cfg2 |= SPI_CFG2_CPHA;
	}
	hardware->CFG2 = cfg2;
	Enable();
}

// Send and receive data returning true if successful
bool SpiDevice::TransceivePacket(const uint8_t *_ecv_array null tx_data, uint8_t *_ecv_array null rx_data, size_t len) noexcept
{
	// Clear any existing data
	(void)hardware->RXDR;

# if defined(RTOS)
	if (len >= 20 && tx_data != nullptr)
	{
		// Sending a large amount of data, so use DMA
		DmacManager::DisableChannel(dmaChanTx);
		DmacManager::SetSourceAddress(dmaChanTx, tx_data);
		DmacManager::SetDestinationAddress(dmaChanTx, &(hardware->TXDR));
		DmacManager::SetBtctrl(dmaChanTx,
								  (0 << DMA_CTR1_DBL_1_Pos)						// destination burst length = 1
								| (0 << DMA_CTR1_DDW_LOG2_Pos)					// destination beat size = 1 byte
								| (0 << DMA_CTR1_SBL_1_Pos)						// source burst length = 1
								| (0 << DMA_CTR1_SDW_LOG2_Pos)					// source beat size = 1 byte
								| DMA_CTR1_SINC									// increment source address
							  );
		DmacManager::SetDataLength(dmaChanTx, len);
		DmacManager::SetTriggerSourceSpiTx(dmaChanTx, instanceNumber);
		waitingTask = TaskBase::GetCallerTaskHandle();
		DmacManager::SetInterruptCallback(dmaChanTx, SpiDevice::DmaComplete, CallbackParameter((void *)this));
		DmacManager::EnableCompletedInterrupt(dmaChanTx);
		if (rx_data != nullptr)
		{
			DmacManager::DisableChannel(dmaChanRx);
			DmacManager::SetSourceAddress(dmaChanRx, &(hardware->RXDR));
			DmacManager::SetDestinationAddress(dmaChanRx, rx_data);
			DmacManager::SetBtctrl(dmaChanRx,
									  (0 << DMA_CTR1_DBL_1_Pos)						// destination burst length = 1
									| (0 << DMA_CTR1_DDW_LOG2_Pos)					// destination beat size = 1 byte
									| (0 << DMA_CTR1_SBL_1_Pos)						// source burst length = 1
									| (0 << DMA_CTR1_SDW_LOG2_Pos)					// source beat size = 1 byte
									| DMA_CTR1_DINC									// increment destination address
								  );
			DmacManager::SetDataLength(dmaChanRx, len);
			DmacManager::SetTriggerSourceSpiTx(dmaChanRx, instanceNumber);
			DmacManager::SetInterruptCallback(dmaChanRx, SpiDevice::DmaComplete, CallbackParameter((void *)this));
			DmacManager::EnableCompletedInterrupt(dmaChanRx);
			DmacManager::EnableChannel(dmaChanRx, dmaPrioTx);
		}
		DmacManager::EnableChannel(dmaChanTx, dmaPrioTx);
		TaskBase::TakeIndexed(NotifyIndices::Spi, 10);			// maximum 3kb transfer should complete in about 2ms @ 14MHz clock speed
	}
	else
# endif
	{
		// For now we use polling mode
		for (uint32_t i = 0; i < len; ++i)
		{
			uint32_t dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;
			if (waitForTxReady())			// we have to write the first byte after enabling the device without waiting for DRE to be set
			{
				return false;
			}

			// Write to transmit register
			hardware->TXDR = dOut;

			// Some devices are transmit-only e.g. 12864 display, so don't wait for received data if we don't need to
			if (rx_data != nullptr)
			{
				// Wait for receive register
				if (waitForRxReady())
				{
					return false;
				}

				// Get data from receive register
				const uint8_t dIn = (uint8_t)hardware->RXDR;
				*rx_data++ = dIn;
			}
		}
	}

	// Wait for transmitter empty, to make sure that the last clock pulse has finished
	waitForTxEmpty();

	// If we were not receiving, clear data from the receive buffer
	if (rx_data == nullptr)
	{
		// The SAME5x seems to buffer more than one received character
		while (hardware->SR & SPI_SR_RXP)
		{
			(void)hardware->RXDR;
		}
	}

	return true;	// success
}

#if defined(RTOS)

void SpiDevice::DmaComplete(DmaCallbackReason reason) noexcept
{
	TaskBase::GiveFromISR(waitingTask, NotifyIndices::Spi);
	waitingTask = nullptr;
}

/*static*/ void SpiDevice::DmaComplete(CallbackParameter param, DmaCallbackReason reason) noexcept
{
	static_cast<SpiDevice*>(param.vp)->DmaComplete(reason);
}

void SpiDevice::Interrupt() noexcept
{
	// not currently used
}

#endif

// End
