/*
 * SpiDevice.cpp
 *
 *  Created on: 7 May 2022
 *      Author: David
 */

#include <SPI/SpiDevice.h>
#include <CoreNotifyIndices.h>

# include <pmc/pmc.h>
# include <Serial.h>

constexpr uint32_t DefaultSharedSpiClockFrequency = 2000000;
constexpr uint32_t SpiTimeout = 10000;

SpiDevice::SpiDevice(const SpiParameters& params) noexcept
	: hardware(Serial::GetUsart(params.usartNumber))
{
	SetPinFunction(params.sclkPin, params.pinFunction);
	SetPinFunction(params.mosiPin, params.pinFunction);
	SetPinFunction(params.misoPin, params.pinFunction);

	pmc_enable_periph_clk(Serial::GetUsartId(params.usartNumber));

	// Set USART in SPI master mode
	hardware->US_IDR = ~0u;
	hardware->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
	hardware->US_MR = US_MR_USART_MODE_SPI_MASTER
					| US_MR_USCLKS_MCK
					| US_MR_CHRL_8_BIT
					| US_MR_CHMODE_NORMAL;
	hardware->US_BRGR = SystemPeripheralClock()/DefaultSharedSpiClockFrequency;
	hardware->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;
}

void SpiDevice::Disable() const noexcept
{
	hardware->US_CR = US_CR_RXDIS | US_CR_TXDIS;			// disable transmitter and receiver
}

void SpiDevice::Enable() const noexcept
{
	hardware->US_CR = US_CR_RXEN | US_CR_TXEN;				// enable transmitter and receiver
}

// Wait for transmitter ready returning true if timed out
inline bool SpiDevice::waitForTxReady() const noexcept
{
	uint32_t timeout = SpiTimeout;
	while (!usart_is_tx_ready(hardware))
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
	while (!usart_is_tx_empty(hardware))
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
	while (!usart_is_rx_ready(hardware))
	{
		if (--timeout == 0)
		{
			return true;
		}
	}
	return false;
}

void SpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept
{
	// We have to disable SPI device in order to change the baud rate, mode and character length
	Disable();
	hardware->US_BRGR = SystemPeripheralClock()/freq;
	uint32_t mr = US_MR_USART_MODE_SPI_MASTER
					| US_MR_USCLKS_MCK
					| US_MR_CHRL_8_BIT
					| US_MR_CHMODE_NORMAL
					| US_MR_CLKO;
	if ((uint8_t)mode & 2)
	{
		mr |= US_MR_CPOL;
	}
	if (((uint8_t)mode & 1) == 0)							// the bit is called CPHA but is actually NPCHA
	{
		mr |= US_MR_CPHA;
	}
	hardware->US_MR = mr;
	hardware->US_CR = US_CR_RSTRX | US_CR_RSTTX;			// reset transmitter and receiver (required - see datasheet)
	Enable();
}

// Send and receive data returning true if successful
bool SpiDevice::TransceivePacket(const uint8_t *_ecv_array null tx_data, uint8_t *_ecv_array null rx_data, size_t len) noexcept
{
	// Clear any existing data
	(void)hardware->US_RHR;
	for (uint32_t i = 0; i < len; ++i)
	{
		uint32_t dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;
		if (waitForTxReady())			// we have to write the first byte after enabling the device without waiting for DRE to be set
		{
			return false;
		}

		// Write to transmit register
		hardware->US_THR = dOut;

		// Some devices are transmit-only e.g. 12864 display, so don't wait for received data if we don't need to
		if (rx_data != nullptr)
		{
			// Wait for receive register
			if (waitForRxReady())
			{
				return false;
			}

			// Get data from receive register
			const uint8_t dIn = (uint8_t)hardware->US_RHR;
			*rx_data++ = dIn;
		}
	}

	// Wait for transmitter empty, to make sure that the last clock pulse has finished
	waitForTxEmpty();

	// If we were not receiving, clear data from the receive buffer
	if (rx_data == nullptr)
	{
		(void)hardware->US_RHR;
	}

	return true;	// success
}

// End

