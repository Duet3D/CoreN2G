/*
 * Serial.h
 *
 *  Created on: 9 Aug 2019
 *      Author: David
 */

#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_

#include <CoreIO.h>
#include <UART/UartMode.h>
#include <General/RingBuffer.h>
#include <RTOSIface/RTOSIface.h>

namespace Serial
{
#if SAME5x
	constexpr uint32_t SercomFastGclkNum = GclkNum96MHz;
	constexpr uint32_t SercomFastGclkFreq = 96000000;
	constexpr uint32_t SercomSlowGclkNum = GclkNum31KHz;
#else
	constexpr uint32_t SercomFastGclkNum = GclkNum48MHz;
	constexpr uint32_t SercomFastGclkFreq = 48000000;
	constexpr uint32_t SercomSlowGclkNum = GclkNum31KHz;
#endif

	static Sercom * const Sercoms[] =
	{
		SERCOM0, SERCOM1, SERCOM2, SERCOM3, SERCOM4, SERCOM5,
#if SAME5x && (defined(__SAME54P20A__) || defined(__SAME51N19A__) || defined(__SAMD51N19A__))
		SERCOM6, SERCOM7
#endif
	};

	constexpr IRQn SercomIRQns[] =
	{
#if SAMC21
		SERCOM0_IRQn, SERCOM1_IRQn, SERCOM2_IRQn, SERCOM3_IRQn, SERCOM4_IRQn, SERCOM5_IRQn
#elif SAME5x
		SERCOM0_0_IRQn, SERCOM1_0_IRQn, SERCOM2_0_IRQn, SERCOM3_0_IRQn, SERCOM4_0_IRQn, SERCOM5_0_IRQn,
# if defined(__SAME54P20A__) || defined(__SAME51N19A__) || defined(__SAMD51N19A__)
		SERCOM6_0_IRQn, SERCOM7_0_IRQn
# endif
#endif
	};

	void Init() noexcept;

	inline Sercom *GetSercom(uint8_t sercomNumber) noexcept { return Sercoms[sercomNumber]; }
	inline constexpr IRQn GetSercomIRQn(uint8_t sercomNumber) noexcept { return SercomIRQns[sercomNumber]; }

	void EnableSercomClock(uint8_t sercomNumber) noexcept;
	void InitUart(uint8_t sercomNumber, uint32_t baudRate, uint8_t rxPad, uint8_t txPad, UartMode uartMode
#if SAME5x
		, bool use32bitMode = false
#endif
		) noexcept;
	void DisableSercom(uint8_t sercomNumber) noexcept;

	// Support for serial interrupt vector reassignment

	// Define indirect interrupt handlers so that we can change the interrupt vectors dynamically
	typedef void (*IrqFunc)(void*) noexcept;

#if SAMC21
	void SetSercomVector(uint8_t sercomNumber, IrqFunc f, void *param) noexcept;
#elif SAME5x
	void SetSercomVector(uint8_t sercomNumber, IrqFunc f0, IrqFunc f1, IrqFunc f2, IrqFunc f3, void *param) noexcept;
#endif
	void ReleaseSercomVector(uint8_t sercomNumber) noexcept;
}

#endif /* SRC_SERIAL_H_ */
