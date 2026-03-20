/*
 * SpiDevice.cpp for RP2040
 *
 *  Created on: 7 May 2022
 *      Author: David
 */

#include <SPI/SpiDevice.h>
#include <CoreNotifyIndices.h>

constexpr uint32_t DefaultSharedSpiClockFrequency = 2000000;
constexpr uint32_t SpiTimeout = 10000;

SpiDevice::SpiDevice(const SpiParameters& params) noexcept
	: hardware((params.instanceNumber == 0) ? spi0 : spi1)
{
	SetPinFunction(params.mosiPin, GpioPinFunction::Spi);
	SetPinFunction(params.misoPin, GpioPinFunction::Spi);
	SetPinFunction(params.sclkPin, GpioPinFunction::Spi);
	// Do we need anything else here? e.g. set high drive strength on mosi and sclk
}

void SpiDevice::Disable() const noexcept
{
	spi_deinit(hardware);
}

void SpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept
{
	spi_init(hardware, freq);
	spi_set_format(hardware, 8, ((uint8_t)mode & 2) ? SPI_CPOL_1 : SPI_CPOL_0, ((uint8_t)mode & 1) ? SPI_CPHA_1 : SPI_CPHA_0, SPI_MSB_FIRST);
}

// Send and receive data returning true if successful
bool SpiDevice::TransceivePacket(const uint8_t *_ecv_array null tx_data, uint8_t *_ecv_array null rx_data, size_t len) noexcept
{
	const int bytesTransferred = (rx_data == nullptr) ? spi_write_blocking(hardware, tx_data, len)
								: (tx_data == nullptr) ? spi_read_blocking(hardware, 0xFF, rx_data, len)
									: spi_write_read_blocking(hardware, tx_data, rx_data, len);
	return bytesTransferred == (int)len;
}

// End

