/*
 * Serial.h
 *
 *  Created on: 9 Aug 2019
 *      Author: David
 */

#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_

#include <CoreIO.h>
#include <General/RingBuffer.h>
#include <RTOSIface/RTOSIface.h>

namespace Serial
{
	static Sercom * const Sercoms[] =
	{
		SERCOM0, SERCOM1, SERCOM2, SERCOM3, SERCOM4, SERCOM5,
#if SAME5x
		SERCOM6, SERCOM7
#endif
	};

	static constexpr IRQn SercomIRQns[] =
	{
#if SAMC21
		SERCOM0_IRQn, SERCOM1_IRQn, SERCOM2_IRQn, SERCOM3_IRQn, SERCOM4_IRQn, SERCOM5_IRQn
#elif SAME5x
		SERCOM0_0_IRQn, SERCOM1_0_IRQn, SERCOM2_0_IRQn, SERCOM3_0_IRQn, SERCOM4_0_IRQn, SERCOM5_0_IRQn, SERCOM6_0_IRQn, SERCOM7_0_IRQn
#endif
	};

	inline Sercom *GetSercom(uint8_t sercomNumber) noexcept { return Sercoms[sercomNumber]; }
	inline constexpr IRQn GetSercomIRQn(uint8_t sercomNumber) noexcept { return SercomIRQns[sercomNumber]; }

	void EnableSercomClock(uint8_t sercomNumber) noexcept;
	void InitUart(uint8_t sercomNumber, uint32_t baudRate, uint8_t rxPad
#if SAME5x
		, bool use32bitMode = false
#endif
		) noexcept;
	void Disable(uint8_t sercomNumber) noexcept;
}

#endif /* SRC_SERIAL_H_ */
