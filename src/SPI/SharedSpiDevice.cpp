/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#include "SharedSpiDevice.h"

// SharedSpiDevice members

SharedSpiDevice::SharedSpiDevice(const SpiParameters& params, uint32_t interruptPriority) noexcept : SpiDevice(params, interruptPriority)
{
	mutex.Create("SPI");
}

// End
