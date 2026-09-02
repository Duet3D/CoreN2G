/*
 * SharedI2CMaster.cpp
 *
 *  Created on: 1 Sept 2026
 *      Author: David
 */

#include <I2C/SharedI2CMaster.h>

#if SAME70 && defined(RTOS)							// we don't support I2C in non-RTOS builds

#include <CoreNotifyIndices.h>
#include "twihs/twihs.h"
#include <pmc/pmc.h>

//******************** Public methods ********************

constexpr uint32_t DefaultSharedI2CClockFrequency = 400000;
constexpr uint32_t I2CTimeoutTicks = 100;
constexpr uint32_t ShutdownTimeoutMillis = 50;
constexpr uint32_t RecoveryHalfClockMicros = 5;
constexpr size_t MinBytesForDmaRead = 8;			// shorter reads are not worth the DMA setup cost

Twihs *const I2cInterfaces[] = { TWIHS0, TWIHS1, TWIHS2 };
constexpr IRQn I2cIrqns[] = { TWIHS0_IRQn, TWIHS1_IRQn, TWIHS2_IRQn };
constexpr uint8_t TwihsIds[] = { ID_TWIHS0, ID_TWIHS1, ID_TWIHS2 };

SharedI2CMaster::SharedI2CMaster(const I2cParameters& params) noexcept
	: sclPin(params.sclPin), sdaPin(params.sdaPin), pinFunction(params.pinFunction),
	  rxDmaChannel(params.rxDmaChannel), rxDmaPriority(params.rxDmaPriority), hardware(I2cInterfaces[params.instanceNumber]), taskWaiting(nullptr), state(I2cState::idle)
{
	pmc_enable_periph_clk(TwihsIds[params.instanceNumber]);						// enable the peripheral clock
	errors.Clear();
	twihs_reset(hardware);

	RecoverBus();																// this also connects the pins to the TWIHS

	currentClockRate = 0;														// make sure that SetClockFrequency does the initialisation
	SetClockFrequency(DefaultSharedI2CClockFrequency);							// this also does the initialisation

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

// Set the clock frequency and initialise the I2C interface in master mode
void SharedI2CMaster::SetClockFrequency(uint32_t freq) noexcept
{
	if (currentClockRate != freq)
	{
		twihs_options_t opt;
		opt.speed = freq;
		opt.master_clk = SystemPeripheralClock();
		opt.chip = opt.smbus = 0;
		twihs_master_init(hardware, &opt);
		currentClockRate = freq;
	}
}

bool SharedI2CMaster::Transfer(uint16_t address, const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead) noexcept
{
	// If an empty transfer, nothing to do
	if (numToRead + numToWrite == 0)
	{
		return true;
	}

	for (unsigned int triesDone = 0; triesDone < 3; ++triesDone)
	{
		if (InternalTransfer(address, txBuffer, rxBuffer, numToWrite, numToRead))
		{
			return true;
		}

		// Had an I2C error, so re-initialise
		Disable();
		RecoverBus();
		Enable();
	}
	return false;
}

// Get ownership of this I2C interface, return true if successful
bool SharedI2CMaster::Take(uint32_t timeout) noexcept
{
	const bool success = mutex.Take(timeout);
	if (!success)
	{
		++errors.contentions;
	}
	return success;
}

// Release ownership of this I2C interface
void SharedI2CMaster::Release() noexcept
{
	// Now that Transfer() has an option to not release the bus, we may have called Take() several times, so we may need to call Release() several times
	while (mutex.GetHolder() == TaskBase::GetCallerTaskHandle())
	{
		mutex.Release();
	}
}

//******************** Private methods ********************

void SharedI2CMaster::Enable() const noexcept
{
	hardware->TWIHS_CR = TWIHS_CR_MSEN;
}

void SharedI2CMaster::Disable() const noexcept
{
	hardware->TWIHS_CR = TWIHS_CR_MSDIS;
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

// Wait for a status bit or NAK to be set, returning true if successful and it wasn't NAK
// It waits until either 2 clock ticks have passed (so we have waited for at least 1ms) or one or more of the status bits we are interested in has been set.
bool SharedI2CMaster::WaitForStatus(uint32_t statusBit, unsigned int& timeoutErrorCounter) noexcept
{
	//TODO use interrupts instead of polling
	const uint32_t startMillis = millis();
	bool timedOut;
	uint32_t sr;
	do
	{
		timedOut = (millis() - startMillis > 2);
		sr = hardware->TWIHS_SR;							// read this after checking for timeout, in case we get descheduled between the two statements
	} while (!timedOut && (sr & statusBit) == 0);

	if ((sr & TWIHS_SR_NACK) != 0)
	{
		++errors.naks;
		return false;
	}
	if ((sr & statusBit) != 0)
	{
		return true;
	}
	++timeoutErrorCounter;
	return false;
}

inline bool SharedI2CMaster::WaitTransferComplete() noexcept
{
	return WaitForStatus(TWIHS_SR_TXCOMP, errors.otherErrors);
}

inline bool SharedI2CMaster::WaitByteSent() noexcept
{
	return WaitForStatus(TWIHS_SR_TXRDY, errors.otherErrors);
}

inline bool SharedI2CMaster::WaitByteReceived() noexcept
{
	return WaitForStatus(TWIHS_SR_RXRDY, errors.otherErrors);
}

bool SharedI2CMaster::InternalTransfer(uint16_t address, const uint8_t *_ecv_array txBuffer, uint8_t *_ecv_array rxBuffer, size_t numToWrite, size_t numToRead) noexcept
{
	// Set up the mode register and address
	if (address >= 0x80)
	{
		// 10-bit address
		const uint32_t topAddress = 0b01111000 | ((address >> 8) & 3);
	    hardware->TWIHS_MMR = (1 << 8) | (topAddress << 16);
	    hardware->TWIHS_IADR = address & 0x00FF;
	}
	else
	{
		// 7-bit address
	    hardware->TWIHS_MMR = (uint32_t)address << 16;
	    hardware->TWIHS_IADR = 0;
	}

	size_t bytesSent = 0;
	if (numToWrite != 0)
	{
		// Send all bytes except the last one.
		// Ideally, if there are bytes to read as well as write, we would not send a STOP after sending all the bytes.
		// Unfortunately, the SAM TWI peripheral doesn't provide any means of reporting when the transmission is complete if we don't send STOP after it.
		while (bytesSent + 1 < numToWrite)
		{
			hardware->TWIHS_THR = *txBuffer++;
			if (!WaitByteSent())
			{
				hardware->TWIHS_CR = TWIHS_CR_STOP;
				(void)WaitTransferComplete();
				return bytesSent;
			}
			++bytesSent;
		}

		hardware->TWIHS_THR = *txBuffer++;
		hardware->TWIHS_CR = TWIHS_CR_STOP;
		if (WaitByteSent())
		{
			++bytesSent;
		}
		(void)WaitTransferComplete();
		if (bytesSent < numToWrite || numToRead == 0)
		{
			return bytesSent;
		}
	}

	// There are bytes to read, and if there were any bytes to send then we have sent them all
    hardware->TWIHS_MMR |= TWIHS_MMR_MREAD;							// change the mode to read
	size_t bytesReceived = 0;
	if (numToRead == 1)
	{
		hardware->TWIHS_CR = TWIHS_CR_START | TWIHS_CR_STOP;
		if (WaitByteReceived())
		{
			*rxBuffer = hardware->TWIHS_RHR;
			++bytesReceived;
		}

		(void)WaitTransferComplete();
		return bytesSent + bytesReceived;
	}

	// Multi-byte read. We must set the STOP flag before we read the penultimate byte from the RHR.
	hardware->TWIHS_CR = TWIHS_CR_START;
	for (;;)
	{
		if (!WaitByteReceived())
		{
			hardware->TWIHS_CR = TWIHS_CR_STOP;						// this may not do any good
			(void)WaitTransferComplete();							// neither may this
			return bytesSent + bytesReceived;
		}

		++bytesReceived;
		if (bytesReceived + 1 == numToRead)
		{
			break;
		}

		*rxBuffer++ = hardware->TWIHS_RHR;
	}

	// The penultimate byte is in the RHR
	hardware->TWIHS_CR = TWIHS_CR_STOP;
	*rxBuffer++ = hardware->TWIHS_RHR;
	if (WaitByteReceived())
	{
		*rxBuffer++ = hardware->TWIHS_RHR;
		++bytesReceived;
	}
	(void)WaitTransferComplete();
	return bytesSent + bytesReceived;
}

#endif

// End
