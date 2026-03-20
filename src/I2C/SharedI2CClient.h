/*
 * SharedI2CClient.h
 *
 *  Created on: 14 Mar 2021
 *      Author: David
 */

#ifndef SRC_HARDWARE_SHAREDI2CCLIENT_H_
#define SRC_HARDWARE_SHAREDI2CCLIENT_H_

#include <CoreIO.h>

#ifdef RTOS					// we dn't support I2C in non-RTOS builds

#include "SharedI2CMaster.h"

class SharedI2CClient
{
public:
	SharedI2CClient(SharedI2CMaster& dev, uint16_t addr) noexcept;
	void SetAddress(uint16_t addr) noexcept { address = addr; }
	bool Transfer(const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead, uint32_t timeout, bool releaseBus = true) noexcept;
	bool Take(uint32_t timeout) noexcept { return device.Take(timeout); }
	void Release() noexcept { device.Release(); }

private:
	SharedI2CMaster& device;
	uint16_t address;
};

#endif

#endif /* SRC_HARDWARE_SHAREDI2CCLIENT_H_ */
