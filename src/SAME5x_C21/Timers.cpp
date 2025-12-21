/*
 * Timers.cpp
 *
 *  Created on: 21 Dec 2025
 *      Author: David
 */

#include "Timers.h"

// Choose the most appropriate TC or TCC prescaler for the PWM frequency we want.
// Some TCs and TCCs share a clock selection, so we always use the same GCLK
// 'counterBits' is 16 or 24 but we might also use 8 in future
// Return the prescaler register value
uint32_t Timers::ChoosePrescaler(uint16_t freq, unsigned int counterBits, uint32_t& top) noexcept
{
	static const unsigned int PrescalerShifts[] = { 0, 1, 2, 3, 4, 6, 8, 10 };		// available prescalers are 1 2 4 8 16 64 256 1024
	for (uint32_t i = 0; i < ARRAY_SIZE(PrescalerShifts); ++i)
	{
		if ((TcGclkFreq >> (PrescalerShifts[i] + counterBits)) <= (uint32_t)freq)
		{
			top = ((TcGclkFreq >> PrescalerShifts[i])/(uint32_t)freq) - 1;
			return i;
		}
	}
	top = (1ul << counterBits) - 1;
	return ARRAY_SIZE(PrescalerShifts) - 1;
}

// End
