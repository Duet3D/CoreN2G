/*
 * SharedI2CMaster.cpp
 *
 *  Created on: 1 Sept 2026
 *      Author: David
 */

#include <I2C/SharedI2CMaster.h>

#if SAME70 && defined(RTOS)					// we don't support I2C in non-RTOS builds

#include <CoreNotifyIndices.h>
#include "twihs/twihs.h"		//TODO not sure we need this

//******************** Public methods ********************

constexpr uint32_t DefaultSharedI2CClockFrequency = 400000;
constexpr uint32_t I2CTimeoutTicks = 100;
constexpr uint32_t ShutdownTimeoutMillis = 50;
constexpr uint32_t RecoveryHalfClockMicros = 5;
constexpr size_t MinBytesForDmaRead = 8;			// shorter reads are not worth the DMA setup cost

Twihs *const I2cInterfaces[] = { TWIHS0, TWIHS1, TWIHS2 };
constexpr IRQn I2cIrqns[] = { TWIHS0_IRQn, TWIHS1_IRQn, TWIHS2_IRQn };

SharedI2CMaster::SharedI2CMaster(const I2cParameters& params) noexcept
	: sclPin(params.sclPin), sdaPin(params.sdaPin), pinFunction(params.pinFunction),
	  rxDmaChannel(params.rxDmaChannel), rxDmaPriority(params.rxDmaPriority), hardware(I2cInterfaces[params.instanceNumber]), taskWaiting(nullptr), state(I2cState::idle)
{
	errors.Clear();

	RecoverBus();								// this also connects the pins to the SERCOM
	//TODO

	//TODO
	const IRQn irqn = I2cIrqns[params.instanceNumber];
	NVIC_SetPriority(irqn, params.irqPriority);
	NVIC_ClearPendingIRQ(irqn);
	NVIC_EnableIRQ(irqn);
}

// Resetting the processor part way through a transfer leaves the slave device driving SDA low. RecoverBus() clears that at the next startup, but better not to get there
void SharedI2CMaster::End() noexcept
{
	(void)Take(ShutdownTimeoutMillis);											// if we time out then the bus is already stuck, so reset anyway
	Disable();
}

void SharedI2CMaster::SetClockFrequency(uint32_t freq) noexcept
{
	//TODO
}

bool SharedI2CMaster::Transfer(uint16_t address, const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead) noexcept
{
	//TODO
}

// Get ownership of this I2C interface, return true if successful
bool SharedI2CMaster::Take(uint32_t timeout) noexcept
{
	//TODO
}

void SharedI2CMaster::Release() noexcept
{
	//TODO
}

//******************** Private methods ********************

void SharedI2CMaster::Enable() const noexcept
{
	//TODO
}

void SharedI2CMaster::Disable() const noexcept
{
	//TODO
}

// Release the bus if a slave device is holding SDA low, e.g. because the processor was reset part way through a read transfer.
// Nine clocks let the slave shift out the rest of its byte and see a NACK, then a STOP condition returns it to idle. Stopping early when SDA reads high is not enough,
// the slave is still mid-byte then and drives SDA low again on the next clock if its next bit is zero.
// The pins are driven as GPIO while doing this and connected to the SERCOM afterwards, so the SERCOM must be disabled or not yet set up
void SharedI2CMaster::RecoverBus() noexcept
{
	SetPinMode(sclPin, INPUT);
	SetPinMode(sdaPin, INPUT);
	if (!digitalRead(sdaPin))
	{
		for (unsigned int i = 0; i < 9; i++)
		{
			SetPinMode(sclPin, OUTPUT_LOW);
			delayMicroseconds(RecoveryHalfClockMicros);
			SetPinMode(sclPin, INPUT);
			delayMicroseconds(RecoveryHalfClockMicros);
		}
		SetPinMode(sclPin, OUTPUT_LOW);
		delayMicroseconds(RecoveryHalfClockMicros);
		SetPinMode(sdaPin, OUTPUT_LOW);
		delayMicroseconds(RecoveryHalfClockMicros);
		SetPinMode(sclPin, INPUT);
		delayMicroseconds(RecoveryHalfClockMicros);
		SetPinMode(sdaPin, INPUT);
		delayMicroseconds(RecoveryHalfClockMicros);
		errors.recoveries++;
	}
	SetPinFunction(sclPin, pinFunction);
	SetPinFunction(sdaPin, pinFunction);
}

bool SharedI2CMaster::InternalTransfer(uint16_t address, const uint8_t *_ecv_array txBuffer, uint8_t *_ecv_array rxBuffer, size_t numToWrite, size_t numToRead) noexcept
{
	//TODO
}

void SharedI2CMaster::ProtocolError()  noexcept
{
	//TODO
}

#endif

// End
