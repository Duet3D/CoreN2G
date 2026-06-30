/*
 * AnalogOutput.cpp
 *
 *      Author: David
 *      License: GPL 3.0
 */

#include <AnalogOut.h>
#include <cmath>
#include <cstring>

#include <pmc/pmc.h>
#include <pio/pio.h>
#include <tc/tc.h>

// Initialise this module
extern void AnalogOut::Init() noexcept
{
	// Nothing to do yet
}

// Convert a float in 0..1 to unsigned integer in 0..N
static inline uint32_t ConvertRange(float f, uint32_t top) noexcept
pre(0.0 <= f; f <= 1.0)
post(_ecv_result <= top)
{
	return lrintf(f * (float)top);
}

// Analog write to a timer or plain output pin
// Setting the frequency of a TC or PWM pin to zero resets it so that the next call to AnalogOut with a non-zero frequency
// will re-initialise it. The pinMode function relies on this.
void AnalogOut::Write(Pin pin, float ulValue, PwmFrequency freq) noexcept
{
	const PinDescriptionBase *_ecv_from _ecv_null const pinDesc = AppGetPinDescription(pin);
	if (pinDesc == nullptr || std::isnan(ulValue))
	{
		return;
	}

	ulValue = constrain<float>(ulValue, 0.0, 1.0);
	const TimerOutput tout = pinDesc->to;

	if (tout != TimerOutput::none)
	{
		// We have a hardware timer output on this pin
		const unsigned int timerNumber = GetTimerNumber(tout);
		if (IsLowPowerTimer(timerNumber))
		{
			LPTIM_TypeDef *lpTimer = GetLowPowerHardwareTimer(timerNumber);
			qq;
		}
		else
		{
			TIM_TypeDef *lpTimer = GetHardwareTimer(timerNumber);
			qq;
		}

		return;
	}

	// Fall back to digital write
	SetPinMode(pin, (ulValue < 0.5) ? OUTPUT_LOW : OUTPUT_HIGH);
}

// End
