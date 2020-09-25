/*
 * Uart.h
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_UART_H_
#define SRC_HARDWARE_SAME5X_UART_H_

#include "Stream.h"
#include "Serial.h"
#include <General/RingBuffer.h>

class Uart : public Stream
{
public:
	typedef void (*InterruptCallbackFn)(Uart*) noexcept;
	typedef void (*OnBeginFn)(Uart*) noexcept;
	typedef void (*OnEndFn)(Uart*) noexcept;

	union ErrorFlags
	{
		uint8_t all;
		uint8_t overrun : 1,
				framing : 1;
	};

	Uart(uint8_t sercomNum, uint8_t rxp, size_t numTxSlots, size_t numRxSlots, OnBeginFn p_onBegin, OnEndFn p_onEnd) noexcept;

	// Overridden virtual functions
	int available() noexcept override;
	int read() noexcept override;
	void flush() noexcept override;
	size_t canWrite() noexcept override;

    size_t write(uint8_t) noexcept override;
    size_t write(const uint8_t *buffer, size_t size) noexcept override;		// this has a default implementation, but can be overridden for efficiency

    using Print::write; // pull in write(str) and write(buf, size) from Print

	// Compatibility functions
	void begin(uint32_t baudRate) noexcept;
	void end() noexcept;
	void setInterruptPriority(uint32_t rxPrio, uint32_t txAndErrorPrio) const noexcept;
    InterruptCallbackFn SetInterruptCallback(InterruptCallbackFn f) noexcept;

#if 0
	// Non-blocking block write
	size_t TryPutBlock(const uint8_t *buffer, size_t buflen) noexcept;
#endif

	// ISRs, must be called by the ISRs for the SERCOM

#if SAME5x
	void Interrupt0() noexcept;
	// We don't use interrupt 1
	void Interrupt2() noexcept;
	void Interrupt3() noexcept;
#elif SAMC21
	void Interrupt() noexcept;
#endif

	// Get and clear the errors
	ErrorFlags GetAndClearErrors() noexcept;

private:
	RingBuffer<uint8_t> txBuffer;
	RingBuffer<uint8_t> rxBuffer;
	Sercom * const sercom;
#ifdef RTOS
	volatile TaskHandle txWaitingTask;
#endif
    InterruptCallbackFn interruptCallback;
    OnBeginFn onBegin;
    OnEndFn onEnd;
	const uint8_t sercomNumber;
	const uint8_t rxPad;
	ErrorFlags errors;
    uint8_t numInterruptBytesMatched;

    static constexpr uint8_t interruptSeq[2] = { 0xF0, 0x0F };
};

#endif /* SRC_HARDWARE_SAME5X_UART_H_ */
