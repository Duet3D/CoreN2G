/*
 * Uart.h
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_C21_ASYNCSERIAL_H_
#define SRC_HARDWARE_SAME5X_C21_ASYNCSERIAL_H_

#include <CoreIO.h>
#include <Stream.h>
#include <General/RingBuffer.h>
#include "Serial.h"
#include <UART/UartParameters.h>

class AsyncSerial : public Stream
{
public:
	typedef void (*InterruptCallbackFn)(AsyncSerial*) noexcept;
#if SAME5x
	typedef void (*OnTransmissionEndedFn)(CallbackParameter) noexcept;
#endif

	union Errors
	{
		uint32_t all;
		struct
		{
			uint32_t uartOverrun : 11,
					 framing : 11,
					 bufferOverrun : 10;
		};
		Errors() noexcept { all = 0; }
	};

	AsyncSerial(const UartParameters& params) noexcept;

	// Overridden virtual functions
	int available() noexcept override;
	int read() noexcept override;
	void flush() noexcept override;
	size_t canWrite() noexcept override;

    size_t write(uint8_t) noexcept override;
    size_t write(const uint8_t *buffer, size_t size) noexcept override;		// this has a default implementation, but can be overridden for efficiency

	void ClearTransmitBuffer() noexcept;
	void ClearReceiveBuffer() noexcept;
	void DisableTransmit() noexcept;
	void EnableTransmit() noexcept;

	// Compatibility functions
	void begin(uint32_t baudRate) noexcept;
	void end() noexcept;
	void setInterruptPriority(uint32_t rxPrio, uint32_t txAndErrorPrio) const noexcept;

	InterruptCallbackFn _ecv_null SetInterruptCallback(InterruptCallbackFn _ecv_null f) noexcept;

#if SAME5x
	OnTransmissionEndedFn _ecv_null SetOnTxEndedCallback(OnTransmissionEndedFn _ecv_null f, CallbackParameter cp) noexcept;
#endif

#if 0
	// Non-blocking block write
	size_t TryPutBlock(const uint8_t *buffer, size_t buflen) noexcept;
#endif

	// Get and clear the errors
	Errors GetAndClearErrors() noexcept;

private:
	RingBuffer<uint8_t> txBuffer;
	RingBuffer<uint8_t> rxBuffer;
	Sercom * const sercom;
#ifdef RTOS
	volatile TaskHandle txWaitingTask;
#endif
    InterruptCallbackFn interruptCallback;

#if SAME5x
	void Interrupt0() noexcept;
	void Interrupt1() noexcept;
	void Interrupt2() noexcept;
	void Interrupt3() noexcept;
	static void CommonInterrupt0(void *param) noexcept;
	static void CommonInterrupt1(void *param) noexcept;
	static void CommonInterrupt2(void *param) noexcept;
	static void CommonInterrupt3(void *param) noexcept;
#elif SAMC21
	void Interrupt() noexcept;
	static void CommonInterrupt(void *param) noexcept;
#endif

#if SAME5x
	OnTransmissionEndedFn _ecv_null onTransmissionEndedFn;
	CallbackParameter _ecv_null onTransmissionEndedCp;
#endif

	Errors errors;
	const uint8_t sercomNumber;
	const Pin rxPin;
	const Pin txPin;
	const GpioPinFunction pinFunction;
	const uint8_t rxPad;
	const uint8_t txPad;

	uint8_t numInterruptBytesMatched;
    bool bufferOverrunPending;
    bool txEnabled;

    static constexpr uint8_t interruptSeq[2] = { 0xF0, 0x0F };
};

#endif /* SRC_HARDWARE_SAME5X_C21_ASYNCSERIAL_H_ */
