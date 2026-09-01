/*
 * SharedI2CMaster.cpp
 *
 *  Created on: 13 Mar 2021
 *      Author: David
 */

#include <I2C/SharedI2CMaster.h>

#if defined(RTOS)					// we don't support I2C in non-RTOS builds

#include <SAME5x_C21/Serial.h>
#include <CoreNotifyIndices.h>

#if SAME5x
# include <hri_sercom_e54.h>
#elif SAMC21
# include <hri_sercom_c21.h>
#endif

constexpr uint32_t DefaultSharedI2CClockFrequency = 400000;
constexpr uint32_t I2CTimeoutTicks = 100;
constexpr uint32_t ShutdownTimeoutMillis = 50;
constexpr uint32_t RecoveryHalfClockMicros = 5;
constexpr size_t MinBytesForDmaRead = 8;			// shorter reads are not worth the DMA setup cost

SharedI2CMaster::SharedI2CMaster(const I2cParameters& params) noexcept
	: sclPin(params.sclPin), sdaPin(params.sdaPin), pinFunction(params.pinFunction),
	  rxDmaChannel(params.rxDmaChannel), rxDmaPriority(params.rxDmaPriority), hardware(Serial::Sercoms[params.sercomNumber]), taskWaiting(nullptr), state(I2cState::idle)
{
	errors.Clear();

	RecoverBus();								// this also connects the pins to the SERCOM

	Serial::EnableSercomClock(params.sercomNumber);

	// Set up the SERCOM
	const uint32_t regCtrlA = SERCOM_I2CM_CTRLA_MODE(5) | SERCOM_I2CM_CTRLA_SPEED(0) | SERCOM_I2CM_CTRLA_SDAHOLD(2) | SERCOM_I2CM_CTRLA_MEXTTOEN | SERCOM_I2CM_CTRLA_SEXTTOEN;

	if (!hardware->I2CM.SYNCBUSY.bit.SWRST)
	{
		const uint32_t mode = regCtrlA & SERCOM_I2CM_CTRLA_MODE_Msk;
		if (hardware->I2CM.CTRLA.bit.ENABLE)
		{
			hri_sercomi2cm_clear_CTRLA_ENABLE_bit(hardware);
			hri_sercomi2cm_wait_for_sync(hardware, SERCOM_I2CM_SYNCBUSY_ENABLE);
		}
		hri_sercomi2cm_write_CTRLA_reg(hardware, SERCOM_I2CM_CTRLA_SWRST | mode);
	}
	hri_sercomi2cm_wait_for_sync(hardware, SERCOM_I2CM_SYNCBUSY_SWRST);

	hri_sercomi2cm_write_CTRLA_reg(hardware, regCtrlA);
	hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;
#if SAME5x
	hardware->I2CM.CTRLC.reg = 0;													// 8-bit mode
#endif
	hri_sercomi2cm_write_BAUD_reg(hardware, SERCOM_I2CM_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * DefaultSharedI2CClockFrequency) - 1));
	currentClockRate = DefaultSharedI2CClockFrequency;
	hri_sercomi2cm_write_DBGCTRL_reg(hardware, SERCOM_I2CM_DBGCTRL_DBGSTOP);		// baud rate generator is stopped when CPU halted by debugger

	if (rxDmaChannel != NoDmaChannel)
	{
		// Set up the parts of the receive DMA descriptor that do not change. The destination address and the length are set for each transfer
		DmacManager::SetBtctrl(rxDmaChannel, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
											| DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_STEPSIZE_X1);
		DmacManager::SetSourceAddress(rxDmaChannel, &(hardware->I2CM.DATA.reg));
		DmacManager::SetTriggerSourceSercomRx(rxDmaChannel, params.sercomNumber);
		DmacManager::SetInterruptCallback(rxDmaChannel, RxDmaCompleteCallback, CallbackParameter(this));
	}

	const IRQn irqn = Serial::GetSercomIRQn(params.sercomNumber);
#if SAMC21
	Serial::SetSercomVector(params.sercomNumber, CommonInterrupt, this);
#elif SAME5x
	Serial::SetSercomVector(params.sercomNumber, CommonInterrupt, CommonInterrupt, nullptr, CommonInterrupt, this);		// we use interrupts 0, 1, 3
#endif
	NVIC_SetPriority(irqn, params.irqPriority);
	NVIC_ClearPendingIRQ(irqn);
	NVIC_EnableIRQ(irqn);
#if SAME5x
	NVIC_SetPriority((IRQn)(irqn + 1), params.irqPriority);
	NVIC_EnableIRQ((IRQn)(irqn + 1));
	NVIC_SetPriority((IRQn)(irqn + 3), params.irqPriority);
	NVIC_EnableIRQ((IRQn)(irqn + 3));
#endif

	mutex.Create("I2C");

	Enable();
}

// Resetting the processor part way through a transfer leaves the slave device driving SDA low. RecoverBus() clears that at the next startup, but better not to get there
void SharedI2CMaster::End() noexcept
{
	(void)Take(ShutdownTimeoutMillis);											// if we time out then the bus is already stuck, so reset anyway
	Disable();
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

// Set the I2C clock frequency. Caller must own the mutex first.
void SharedI2CMaster::SetClockFrequency(uint32_t freq) noexcept
{
	if (freq != currentClockRate)
	{
		// We have to disable I2C device in order to change the baud rate
		Disable();
		hri_sercomi2cm_write_BAUD_reg(hardware, SERCOM_I2CM_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * freq) - 1));
		currentClockRate = freq;
		Enable();
	}
}

void SharedI2CMaster::Enable() const noexcept
{
	hardware->I2CM.CTRLA.bit.ENABLE = 1;
	while (hardware->I2CM.SYNCBUSY.bit.ENABLE) { }
	hardware->I2CM.STATUS.reg = SERCOM_I2CM_STATUS_BUSSTATE(0x01);
	while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
}

void SharedI2CMaster::Disable() const noexcept
{
	hardware->I2CM.CTRLA.bit.ENABLE = 0;
	while (hardware->I2CM.SYNCBUSY.bit.ENABLE) { }
}

// Write then read data. Caller must own the mutex first.
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

// Send the read address and prepare to receive the data. Called with interrupts deferred, either from InternalTransfer or from the interrupt handler.
// Reads long enough to be worth it are received by DMA except for the last byte, which the interrupt handler must NAK before reading it.
// The DMA is armed before the address goes out so that it cannot miss the first byte
void SharedI2CMaster::StartReading(uint32_t addressToSend) noexcept
{
	if (rxDmaChannel != NoDmaChannel && numLeftToRead >= MinBytesForDmaRead)
	{
		DmacManager::SetDestinationAddress(rxDmaChannel, rxTransferBuffer);
		DmacManager::SetDataLength(rxDmaChannel, numLeftToRead - 1);
		DmacManager::EnableCompletedInterrupt(rxDmaChannel);
		DmacManager::EnableChannel(rxDmaChannel, rxDmaPriority);
		state = I2cState::readingWithDma;
		hardware->I2CM.ADDR.reg = addressToSend;
		while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
		hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_ERROR;	// SB is serviced by the DMA; MB is only raised here if the slave NAKs the address
	}
	else
	{
		state = I2cState::reading;
		hardware->I2CM.ADDR.reg = addressToSend;
		while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
		hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB;
	}
}

// Callback from the DMA controller when all but the last byte of a read have been received
/*static*/ void SharedI2CMaster::RxDmaCompleteCallback(CallbackParameter cp, DmaCallbackReason reason) noexcept
{
	static_cast<SharedI2CMaster *>(cp.vp)->RxDmaComplete(reason);
}

void SharedI2CMaster::RxDmaComplete(DmaCallbackReason reason) noexcept
{
	if (state != I2cState::readingWithDma)
	{
		return;
	}

	DmacManager::DisableCompletedInterrupt(rxDmaChannel);
	if ((uint8_t)reason & (uint8_t)DmaCallbackReason::error)
	{
		DmacManager::DisableChannel(rxDmaChannel);
		ProtocolError();
		return;
	}

	// Hand the last byte back to the interrupt handler. INTFLAG.SB is level sensitive, so if that byte has already arrived we get the interrupt as soon as we enable it
	rxTransferBuffer += numLeftToRead - 1;
	numLeftToRead = 1;
	state = I2cState::reading;
	hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB;
}

bool SharedI2CMaster::InternalTransfer(uint16_t address, const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead) noexcept
{
	currentAddress = address << 1;											// SERCOM uses the bottom bit as the Read flag
	txTransferBuffer = txBuffer;
	rxTransferBuffer = rxBuffer;
	numLeftToRead = numToRead;
	numLeftToWrite = numToWrite;
	hardware->I2CM.INTFLAG.reg = 0xFF;										// clear all flag bits
	hardware->I2CM.STATUS.reg = SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_RXNACK | SERCOM_I2CM_STATUS_ARBLOST;		// clear all status bits
	hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;						// make sure the ACKACT bit is clear and CMD is zero

	TaskBase::ClearCurrentTaskNotifyCount(NotifyIndices::I2C);

	{
		AtomicCriticalSectionLocker lock;									// avoid getting descheduled between sending the command and enabling the interrupt

		// Send the address
		if (numToWrite != 0)
		{
			state = I2cState::writing;
			hardware->I2CM.ADDR.reg = (currentAddress >= 0x100) ? currentAddress | SERCOM_I2CM_ADDR_TENBITEN : currentAddress;
			while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
			hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB;
		}
		else if (currentAddress >= 0x100)
		{
			state = I2cState::sendingTenBitAddressForRead;
			hardware->I2CM.ADDR.reg = currentAddress | SERCOM_I2CM_ADDR_TENBITEN;
			while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
			hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB;
		}
		else
		{
			StartReading(currentAddress | 0x0001);
		}
		taskWaiting = TaskBase::GetCallerTaskHandle();
	}

	TaskBase::TakeIndexed(NotifyIndices::I2C, I2CTimeoutTicks);
	if (state == I2cState::idle)
	{
		return true;
	}

	// We timed out or had an error. A receive DMA left armed would write into the buffer of the next transfer, so always stop it
	if (rxDmaChannel != NoDmaChannel)
	{
		DmacManager::DisableChannel(rxDmaChannel);
	}
	state = I2cState::idle;
	return false;
}

void SharedI2CMaster::ProtocolError() noexcept
{
	hardware->I2CM.INTFLAG.reg = 0xFF;
	const uint16_t status = hardware->I2CM.STATUS.reg;
	hardware->I2CM.STATUS.reg = status;
	if (status & (SERCOM_I2CM_STATUS_BUSERR | SERCOM_I2CM_STATUS_ARBLOST))
	{
		++errors.busErrors;
	}
	else if (status & SERCOM_I2CM_STATUS_RXNACK)
	{
		++errors.naks;
	}
	else
	{
		++errors.otherErrors;
	}
	hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN | SERCOM_I2CM_CTRLB_CMD(0x03);			// send stop command, get off bus
	state = I2cState::protocolError;
	while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
	TaskBase::GiveFromISR(taskWaiting, NotifyIndices::I2C);
	taskWaiting = nullptr;
}

// The following is inline because it is only called from one place
inline void SharedI2CMaster::Interrupt() noexcept
{
	const uint8_t flags = hardware->I2CM.INTFLAG.reg & (SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB | SERCOM_I2CM_INTFLAG_ERROR);
	hardware->I2CM.INTENCLR.reg = 0xFF;
	switch (state)
	{
	default:			// should not occur
		break;

	case I2cState::writing:
		if (flags == SERCOM_I2CM_INTFLAG_MB)
		{
			if (numLeftToWrite != 0)
			{
				hardware->I2CM.DATA.reg = *txTransferBuffer++;
				--numLeftToWrite;
				while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
				hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB;
			}
			else if (numLeftToRead == 0)
			{
				hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN | SERCOM_I2CM_CTRLB_CMD(0x03);			// send stop command
				state = I2cState::idle;
				while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
				TaskBase::GiveFromISR(taskWaiting, NotifyIndices::I2C);
				taskWaiting = nullptr;
			}
			else
			{
				StartReading((currentAddress >= 0x100) ? (currentAddress >> 8) | 0b1111001 : currentAddress | 0x0001);
			}
		}
		else
		{
			ProtocolError();
		}
		break;

	case I2cState::sendingTenBitAddressForRead:
		if (flags == SERCOM_I2CM_INTFLAG_MB)
		{
			StartReading((currentAddress >> 8) | 0b1111001);
		}
		else
		{
			ProtocolError();
		}
		break;

	case I2cState::reading:
		if (flags == SERCOM_I2CM_INTFLAG_SB)
		{
			--numLeftToRead;
			if (numLeftToRead == 0)
			{
				// App note says we need to NAK the last byte and send the stop command before we read the data
				hardware->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN | SERCOM_I2CM_CTRLB_ACKACT | SERCOM_I2CM_CTRLB_CMD(0x03);	// NAK and stop
				state = I2cState::idle;
				while (hardware->I2CM.SYNCBUSY.bit.SYSOP) { }
				*rxTransferBuffer++ = hardware->I2CM.DATA.reg;
				TaskBase::GiveFromISR(taskWaiting, NotifyIndices::I2C);
				taskWaiting = nullptr;
			}
			else
			{
				*rxTransferBuffer++ = hardware->I2CM.DATA.reg;			// read the data and acknowledge it because we have set SMEN in CTRLB
				hardware->I2CM.INTENSET.reg = SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB;
			}
		}
		else
		{
			ProtocolError();
		}
		break;

	case I2cState::readingWithDma:
		// The DMA services the received data, so the only interrupts we asked for here mean the transfer has failed
		DmacManager::DisableChannel(rxDmaChannel);
		ProtocolError();
		break;
	}
}

// Common interrupt entry for all I2C masters
/*static*/ void SharedI2CMaster::CommonInterrupt(void *param) noexcept
{
	((SharedI2CMaster*)param)->Interrupt();
}

#endif

// End
