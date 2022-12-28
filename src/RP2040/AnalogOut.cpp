/*
 Copyright (c) 2011 Arduino.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <AnalogOut.h>
#include <cmath>
#include <cstring>

#include <hardware/pwm.h>

// Initialise this module
extern void AnalogOut::Init()
{
	// Nothing to do yet
}

// Convert a float in 0..1 to unsigned integer in 0..N
static inline uint32_t ConvertRange(float f, uint32_t top) noexcept
pre(0.0 <= ulValue; ulValue <= 1.0)
post(result <= top)
{
	return lrintf(f * (float)top);
}

const unsigned int numPwmChannels = 8;

static uint16_t PWMChanFreq[numPwmChannels] = {0};
static uint16_t PWMChanPeriod[numPwmChannels];

// AnalogWrite to a PWM pin
// Return true if successful, false if we need to fall back to digitalWrite
static bool AnalogWritePwm(Pin pin, const PinDescriptionBase * const pinDesc, float val, PwmFrequency freq) noexcept
pre(0.0 <= ulValue; ulValue <= 1.0)
{
	const uint32_t chanIndex = GetChannelNumber(pinDesc->pwm);
	if (freq == 0)
	{
		PWMChanFreq[chanIndex] = freq;
		return false;
	}

	if (PWMChanFreq[chanIndex] != freq)
	{
		// Determine what divisor to use for the PWM. The maximum available is 256, which along with the 16-bit PWM counter and dual slope counting gives a minimum PWM frequency of about 3.7Hz.
		float divisor = (float)(SystemCoreClockFreq/2)/(float)((uint32_t)freq * 65535);
		uint16_t period;
		if (divisor < 1.0)
		{
			// User has asked for a high PWM frequency, too high to count all the way up to 65535
			divisor = 1.0;
			period = SystemCoreClockFreq/freq - 1;
		}
		else
		{
			period = 65535;					// use the maximum resolution
			if (divisor > 256.0)
			{
				divisor = 256.0;			// user asked for less than 3.7Hz
			}
		}

		PWMChanFreq[chanIndex] = freq;
		PWMChanPeriod[chanIndex] = period;

		pwm_set_enabled(chanIndex, false);
		pwm_set_phase_correct(chanIndex, true);
		pwm_set_wrap(chanIndex, period);
		pwm_set_clkdiv(chanIndex, divisor);
		pwm_set_output_polarity(chanIndex, false, false);
		pwm_set_chan_level(chanIndex, GetOutputNumber(pinDesc->pwm), ConvertRange(val, period));
		pwm_set_counter(chanIndex, 0);
		pwm_set_enabled(chanIndex, true);

		// Now setup the PWM output pin for PWM this channel - do this after configuring the PWM to avoid glitches
		SetPinFunction(pin, GetPeriNumber(pinDesc->pwm));
	}
	else
	{
		pwm_set_chan_level(chanIndex, GetOutputNumber(pinDesc->pwm), ConvertRange(val, PWMChanPeriod[chanIndex]));
	}
	return true;
}

// Analog write to DAC, PWM, TC or plain output pin
// Setting the frequency of a TC or PWM pin to zero resets it so that the next call to AnalogOut with a non-zero frequency
// will re-initialise it. The pinMode function relies on this.
void AnalogOut::Write(Pin pin, float val, PwmFrequency freq) noexcept
{
	const PinDescriptionBase * const pinDesc = AppGetPinDescription(pin);
	if (pinDesc == nullptr || std::isnan(val))
	{
		return;
	}

	val = constrain<float>(val, 0.0, 1.0);

	if (pinDesc->pwm != PwmOutput::none)
	{
		if (AnalogWritePwm(pin, pinDesc, val, freq))
		{
			return;
		}
	}

	// Fall back to digital write
	pinMode(pin, (val < 0.5) ? OUTPUT_LOW : OUTPUT_HIGH);
}

// End
