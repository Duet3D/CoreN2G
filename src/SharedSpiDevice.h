/*
 * SharedSpiDevice.h
 *
 *  Created on: 1 Jul 2019
 *      Author: David
 *
 *  This currently supports only a single SPI channel. To support multiple SPI channels we would need to make the underlying SERCOM device
 *  configured in SPI mode a separate object, and have a pointer or reference to it in SharedSpiDevice.
 */

#ifndef SRC_HARDWARE_SHAREDSPICLIENT_H_
#define SRC_HARDWARE_SHAREDSPICLIENT_H_

#include <CoreIO.h>
#include <RTOSIface/RTOSIface.h>

enum class SpiMode : uint8_t
{
	mode0 = 0, mode1, mode2, mode3
};

class SharedSpiDevice
{
public:
	SharedSpiDevice(uint8_t sercomNum) noexcept;

	void Disable() const noexcept;
	void Enable() const noexcept;
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept;
	bool Take(uint32_t timeout) const noexcept { return mutex.Take(timeout); }			// get ownership of this SPI, return true if successful
	void Release() const noexcept { mutex.Release(); }

private:
	bool waitForTxReady() const noexcept;
	bool waitForTxEmpty() const noexcept;
	bool waitForRxReady() const noexcept;

	Sercom * const hardware;
	Mutex mutex;
};

class SharedSpiClient
{
public:
	SharedSpiClient(SharedSpiDevice& dev, uint32_t clockFreq, SpiMode m, bool polarity) noexcept;

	void InitMaster() noexcept;
	bool Select(uint32_t timeout) const noexcept;										// get SPI ownership and select the device, return true if successful
	void Deselect() const noexcept;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept;
	void SetCsPin(Pin p) noexcept { csPin = p; }

private:
	SharedSpiDevice& device;
	uint32_t clockFrequency;
	Pin csPin;
	SpiMode mode;
	bool csActivePolarity;
};

#endif /* SRC_HARDWARE_SHAREDSPICLIENT_H_ */
