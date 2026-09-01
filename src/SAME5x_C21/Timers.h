/*
 * Timers.h
 *
 *  Created on: 21 Dec 2025
 *      Author: David
 */

#ifndef SRC_SAME5X_C21_TIMERS_H_
#define SRC_SAME5X_C21_TIMERS_H_

#include <CoreIO.h>

namespace Timers
{
	volatile Tc* const TcDevices[] =
	{
		TC0, TC1, TC2, TC3, TC4,
#if SAME5x
		TC5,
# if defined(__SAME54P20A__) || defined(__SAME54N20A__) || defined(__SAME51N19A__) || defined(__SAME51P20A__)
		TC6, TC7					// CAUTION! lower pin count variants of SAME5x devices don't have these
# endif
#endif
	};

	volatile Tcc* const TccDevices[] =
	{
		TCC0, TCC1, TCC2,
#if SAME5x
		TCC3, TCC4
#endif
	};

#if SAME5x
	constexpr unsigned int TcGclkNum = GclkNum60MHz;
	constexpr uint32_t TcGclkFreq = 60000000;
#elif SAMC21
	constexpr unsigned int TcGclkNum = GclkNum48MHz;
	constexpr uint32_t TcGclkFreq = 48000000;
#else
# error Unsupported processor
#endif

	constexpr unsigned int NumTcDevices = ARRAY_SIZE(TcDevices);
	constexpr unsigned int NumTccDevices = ARRAY_SIZE(TccDevices);

	constexpr unsigned int TccCounterBits[NumTccDevices] =
	{
		24, 24, 16,
#if SAME5x
		16, 16
#endif
	};

	constexpr uint8_t TccNumChannels[NumTccDevices] =
	{
#if SAME5x
		6, 4, 3, 2, 2
#elif SAMC21
		4, 2, 2
#endif
	};

	// Convert a float in 0..1 to unsigned integer in 0..N
	static inline uint32_t ConvertRange(float f, uint32_t top) noexcept
	pre(0.0 <= ulValue; ulValue <= 1.0)
	post(result <= top)
	{
		return lrintf(f * (float)(top + 1));
	}

	// Choose the most appropriate TC or TCC prescaler for the PWM frequency we want.
	// Some TCs and TCCs share a clock selection, so we always use the same GCLK
	// 'counterBits' is 16 or 24 but we might also use 8 in future
	// Return the prescaler register value and set 'top' to the required TC or TCC TOP value
	uint32_t ChoosePrescaler(uint32_t freq, unsigned int counterBits, uint32_t& top) noexcept;
}

#endif /* SRC_SAME5X_C21_TIMERS_H_ */
