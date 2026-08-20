/*
 * SharedI2CMaster.h
 *
 *  Created on: 13 Mar 2021
 *      Author: David
 */

#ifndef SRC_HARDWARE_SHAREDI2CMASTER_H_
#define SRC_HARDWARE_SHAREDI2CMASTER_H_

#include <CoreIO.h>
#include "I2cParameters.h"

#ifdef RTOS					// we dn't support I2C in non-RTOS builds

#include <RTOSIface/RTOSIface.h>
#if SAME5x || SAMC21
# include <DmacManager.h>
#endif

struct I2cErrors
{
	unsigned int busErrors, naks, contentions, otherErrors, recoveries;

	void Clear() noexcept
	{
		busErrors = naks = contentions = otherErrors = recoveries = 0;
	}
};

class SharedI2CMaster
{
public:
	explicit SharedI2CMaster(const I2cParameters& params) noexcept;

	void End() noexcept;						// wait for any transfer in progress to complete, then shut down
	void SetClockFrequency(uint32_t freq) noexcept;
	bool Transfer(uint16_t address, const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead) noexcept;

	bool Take(uint32_t timeout) noexcept;		// get ownership of this I2C interface, return true if successful
	void Release() noexcept;

	void GetAndClearErrors(I2cErrors& errs) noexcept
	{
		errs = errors;
		errors.Clear();
	}

private:
	enum class I2cState : uint8_t
	{
		idle = 0, writing, sendingTenBitAddressForRead, reading, readingWithDma, protocolError
	};

	void Enable() const noexcept;
	void Disable() const noexcept;
	void RecoverBus() noexcept;
	bool InternalTransfer(uint16_t address, const uint8_t *_ecv_array txBuffer, uint8_t *_ecv_array rxBuffer, size_t numToWrite, size_t numToRead) noexcept;
	void ProtocolError()  noexcept;

#if SAME5x || SAMC21
	static void CommonInterrupt(void *param) noexcept;
	void Interrupt() noexcept;
	void StartReading(uint32_t addressToSend) noexcept;
	static void RxDmaCompleteCallback(CallbackParameter cp, DmaCallbackReason reason) noexcept;
	void RxDmaComplete(DmaCallbackReason reason) noexcept;

	Sercom * const hardware;
	const Pin sclPin, sdaPin;
	const GpioPinFunction pinFunction;
	const DmaChannel rxDmaChannel;
	const DmaPriority rxDmaPriority;
#endif

	TaskHandle taskWaiting;
	Mutex mutex;

	uint32_t currentClockRate;
	const uint8_t *_ecv_array txTransferBuffer;
	uint8_t *_ecv_array rxTransferBuffer;
	size_t numLeftToRead, numLeftToWrite;
	I2cErrors errors;
	uint16_t currentAddress;
	volatile I2cState state;
};

#endif

#endif /* SRC_HARDWARE_SHAREDI2CMASTER_H_ */
