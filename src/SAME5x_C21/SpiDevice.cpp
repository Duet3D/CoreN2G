/*
 * SpiDevice.cpp
 *
 *  Created on: 7 May 2022
 *      Author: David
 */

#include <SPI/SpiDevice.h>
#include <CoreNotifyIndices.h>

# include <SAME5x_C21/Serial.h>
# include <peripheral_clk_config.h>

constexpr uint32_t DefaultSharedSpiClockFrequency = 2000000;
constexpr uint32_t SpiCharTimeout = 10000;			// this is a count of how often we loop while waiting for the SPI peripheral to finish transmitting or receiving a character

SpiDevice::SpiDevice(const SpiParameters& params) noexcept
	: hardware(Serial::GetSercom(params.sercomNumber)), sercomNumber(params.sercomNumber), dmaChanTx(params.dmaChanTx), dmaPrioTx(params.dmaPrioTx)
{
	SetPinMode(params.mosiPin, INPUT_PULLDOWN);
	SetPinMode(params.misoPin, INPUT_PULLDOWN);
	SetPinMode(params.sclkPin, INPUT_PULLDOWN);
	SetPinFunction(params.mosiPin, params.pinFunction);
	SetPinFunction(params.misoPin, params.pinFunction);
	SetPinFunction(params.sclkPin, params.pinFunction);
	SetDriveStrength(params.mosiPin, 2);
	SetDriveStrength(params.sclkPin, 2);								// some devices (e.g. TFT LCD font chip) need fast rise and fall times

	Serial::EnableSercomClock(params.sercomNumber);

	// Set up the SERCOM
	const uint32_t regCtrlA = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DIPO(params.dataInPad) | SERCOM_SPI_CTRLA_DOPO(params.dataOutPad) | SERCOM_SPI_CTRLA_FORM(0);
	const uint32_t regCtrlB = 0;											// 8 bits, slave select disabled, receiver disabled for now
# if SAME5x
	const uint32_t regCtrlC = 0;											// not 32-bit mode
# endif

	if ((hardware->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_SWRST) == 0)
	{
		while (hardware->SPI.SYNCBUSY.reg & (SERCOM_SPI_SYNCBUSY_SWRST | SERCOM_SPI_SYNCBUSY_ENABLE)) { }
		if (hardware->SPI.CTRLA.reg & SERCOM_SPI_CTRLA_ENABLE)
		{
			hardware->SPI.CTRLA.reg &= ~SERCOM_SPI_CTRLA_ENABLE;
			while (hardware->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE) { }
		}
		hardware->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_SWRST | (regCtrlA & SERCOM_SPI_CTRLA_MODE_Msk);
	}
	while (hardware->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_SWRST) { }

	hardware->SPI.CTRLA.reg = regCtrlA;
	hardware->SPI.CTRLB.reg = regCtrlB;
#if SAME5x
	hardware->SPI.CTRLC.reg = regCtrlC;
#endif
	hardware->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * DefaultSharedSpiClockFrequency) - 1);
	hardware->SPI.DBGCTRL.reg = SERCOM_SPI_DBGCTRL_DBGSTOP;					// baud rate generator is stopped when CPU halted by debugger

	hardware->SPI.CTRLB.bit.RXEN = 1;
}

void SpiDevice::Disable() const noexcept
{
	hardware->SPI.CTRLA.bit.ENABLE = 0;
	while (hardware->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE) { }
}

void SpiDevice::Enable() const noexcept
{
	hardware->SPI.CTRLA.bit.ENABLE = 1;
	while (hardware->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE) { }
}

// Wait for transmitter ready returning true if timed out
inline bool SpiDevice::waitForTxReady() const noexcept
{
	uint32_t timeout = SpiCharTimeout;
	while (!(hardware->SPI.INTFLAG.bit.DRE))
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
	uint32_t timeout = SpiCharTimeout;
	while (!(hardware->SPI.INTFLAG.bit.TXC))
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
	uint32_t timeout = SpiCharTimeout;
	while (!(hardware->SPI.INTFLAG.bit.RXC))
	{
		if (--timeout == 0)
		{
			return true;
		}
	}
	return false;
}

void SpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode
#if SAME5x
											, bool nineBits
#endif
										) const noexcept
{
	// We have to disable SPI device in order to change the baud rate, mode and character length
	Disable();
	// Round the clock frequency rate down. For example, using 60MHz clock, if we ask for 4MHz:
	// Without rounding, divisor = 60/(2*4) = 7, actual clock rate = 4.3MHz
	// With rounding, divisor = 67/8 = 8, actual clock rate = 3.75MHz
	// To get more accurate speeds we could increase the clock frequency to 100MHz
	hardware->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD((Serial::SercomFastGclkFreq + (2 * freq) - 1)/(2 * freq) - 1);
	hardware->SPI.CTRLB.bit.CHSIZE =
#if SAME5x
									(nineBits) ? 1 :
#endif
											0;
	while (hardware->SPI.SYNCBUSY.bit.CTRLB) { }

	uint32_t regCtrlA = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_FORM(0);
	if (((uint8_t)mode & 2) != 0)
	{
		regCtrlA |= SERCOM_SPI_CTRLA_CPOL;
	}
	if (((uint8_t)mode & 1) != 0)
	{
		regCtrlA |= SERCOM_SPI_CTRLA_CPHA;
	}
	hardware->SPI.CTRLA.reg = regCtrlA;
	Enable();
}

// Send and receive data returning true if successful
bool SpiDevice::TransceivePacket(const uint8_t *_ecv_array null tx_data, uint8_t *_ecv_array null rx_data, size_t len, uint32_t dmaTimeout) noexcept
{
	// Clear any existing data
	(void)hardware->SPI.DATA.reg;

# if defined(RTOS)
	if (len >= 40 && rx_data == nullptr && tx_data != nullptr)
	{
		// Sending a large amount of data to LCD, so use DMA
		DmacManager::DisableChannel(dmaChanTx);
		DmacManager::SetSourceAddress(dmaChanTx, tx_data);
		DmacManager::SetDestinationAddress(dmaChanTx, &(hardware->SPI.DATA));
		DmacManager::SetBtctrl(dmaChanTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_NOACT);
		DmacManager::SetDataLength(dmaChanTx, len);
		DmacManager::SetTriggerSourceSercomTx(dmaChanTx, sercomNumber);
		waitingTask = TaskBase::GetCallerTaskHandle();
		DmacManager::SetInterruptCallback(dmaChanTx, SpiDevice::DmaComplete, CallbackParameter((void *)this));
		DmacManager::EnableCompletedInterrupt(dmaChanTx);
		DmacManager::EnableChannel(dmaChanTx, dmaPrioTx);
		TaskBase::TakeIndexed(NotifyIndices::Spi, dmaTimeout);			// maximum 3kb transfer should complete in about 2ms @ 14MHz clock speed
	}
	else
# endif
	{
		for (uint32_t i = 0; i < len; ++i)
		{
			uint32_t dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;
			if (waitForTxReady())										// we have to write the first byte after enabling the device without waiting for DRE to be set
			{
				return false;
			}

			// Write to transmit register
			hardware->SPI.DATA.reg = dOut;

			// Some devices are transmit-only e.g. 12864 display, so don't wait for received data if we don't need to
			if (rx_data != nullptr)
			{
				// Wait for receive register
				if (waitForRxReady())
				{
					return false;
				}

				// Get data from receive register
				const uint8_t dIn =
					(uint8_t)hardware->SPI.DATA.reg;
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
		while (hardware->SPI.INTFLAG.bit.RXC)
		{
			(void)hardware->SPI.DATA.reg;
		}
	}

	return true;	// success
}

#if SAME5x

// Send and receive data returning true if successful, using 16-bit data transfers (needed when using 9-bit characters). 'len' is in 16-bit words.
bool SpiDevice::TransceivePacketNineBit(const uint16_t *_ecv_array null tx_data, uint16_t *_ecv_array null rx_data, size_t len, uint32_t dmaTimeout) noexcept
{
	// Clear any existing data
	(void)hardware->SPI.DATA.reg;

#if defined(RTOS)
	if (len >= 40 && rx_data == nullptr && tx_data != nullptr)
	{
		// Sending a large amount of data to LCD, so use DMA. Currently only the TFT LCD uses this device, so we use a fixed DMA channel number.
		DmacManager::DisableChannel(dmaChanTx);
		DmacManager::SetSourceAddress(dmaChanTx, tx_data);
		DmacManager::SetDestinationAddress(dmaChanTx, &(hardware->SPI.DATA));
		DmacManager::SetBtctrl(dmaChanTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_BLOCKACT_NOACT);
		DmacManager::SetDataLength(dmaChanTx, len);
		DmacManager::SetTriggerSourceSercomTx(dmaChanTx, sercomNumber);
		waitingTask = TaskBase::GetCallerTaskHandle();
		DmacManager::SetInterruptCallback(dmaChanTx, SpiDevice::DmaComplete, CallbackParameter((void *)this));
		DmacManager::EnableCompletedInterrupt(dmaChanTx);
		DmacManager::EnableChannel(dmaChanTx, dmaPrioTx);
		TaskBase::TakeIndexed(NotifyIndices::Spi, dmaTimeout);			// maximum 3kb transfer should complete in about 2ms @ 14MHz clock speed
	}
	else
#endif
	{
		for (uint32_t i = 0; i < len; ++i)
		{
			uint32_t dOut = (tx_data == nullptr) ? 0x000001FF : (uint32_t)*tx_data++;
			if (waitForTxReady())										// we have to write the first byte after enabling the device without waiting for DRE to be set
			{
				return false;
			}

			// Write to transmit register
			hardware->SPI.DATA.reg = dOut;

			// Some devices are transmit-only e.g. 12864 display, so don't wait for received data if we don't need to
			if (rx_data != nullptr)
			{
				// Wait for receive register
				if (waitForRxReady())
				{
					return false;
				}

				// Get data from receive register
				const uint16_t dIn = (uint16_t)hardware->SPI.DATA.reg;
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
		while (hardware->SPI.INTFLAG.bit.RXC)
		{
			(void)hardware->SPI.DATA.reg;
		}
	}

	return true;	// success
}

#endif

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

#endif

// End

