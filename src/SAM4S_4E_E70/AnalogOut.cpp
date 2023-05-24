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

#include <pmc/pmc.h>
#include <pio/pio.h>
#include <tc/tc.h>

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

#if SAME70
const unsigned int numPwmChannels = 8;
#elif SAM4E || SAM4S
const unsigned int numPwmChannels = 4;
#endif

static bool PWMEnabled = false;
static uint16_t PWMChanFreq[numPwmChannels] = {0};
static uint16_t PWMChanPeriod[numPwmChannels];

// AnalogWrite to a PWM pin
// Return true if successful, false if we need to fall back to digitalWrite
static bool AnalogWritePwm(Pin pin, const PinDescriptionBase * const pinDesc, float pwmValue, PwmFrequency freq) noexcept
pre(0.0 <= ulValue; ulValue <= 1.0)
pre((pinDesc.ulPinAttribute & PIN_ATTR_PWM) != 0)
{
	const uint32_t chanIndex = GetChannelNumber(pinDesc->pwm);
	if (freq == 0)
	{
		PWMChanFreq[chanIndex] = freq;
		return false;
	}

	// Which PWM interface and channel within that interface do we need to work with?
#if SAME70
	Pwm * const PWMInterface = (chanIndex < 4) ? PWM0 : PWM1;
	const uint32_t chan = chanIndex & 3;							// SAME70 has two PWM controllers with 3 channels each
#else
	Pwm * const PWMInterface = PWM;
	const uint32_t chan = chanIndex;								// other supported processors have one PWM controller with 4 or 8 channels
#endif

	if (PWMChanFreq[chanIndex] != freq)
	{
		if (!PWMEnabled)
		{
			// PWM Startup code
#if SAME70
			pmc_enable_periph_clk(ID_PWM0);
			pmc_enable_periph_clk(ID_PWM1);
			// The SAME70 PWM channels let us divide the system clock by 1 to 1024 or use CLKA or CLKB. Set up CLKA to divider by 2048 and CLKB to divide by 4096.
			PWM0->PWM_CLK = PWM_CLK_PREA_CLK_DIV1024 | PWM_CLK_DIVA(2) | PWM_CLK_PREB_CLK_DIV1024 | PWM_CLK_DIVB(4);
			PWM0->PWM_SCM = 0;										// ensure no sync channels
			PWM1->PWM_CLK = PWM_CLK_PREA_CLK_DIV1024 | PWM_CLK_DIVA(2) | PWM_CLK_PREB_CLK_DIV1024 | PWM_CLK_DIVB(4);
			PWM1->PWM_SCM = 0;										// ensure no sync channels
#else
			pmc_enable_periph_clk(ID_PWM);
			PWM->PWM_CLK = PWM_CLK_PREA(10) | PWM_CLK_DIVA(2) | PWM_CLK_PREB(10) | PWM_CLK_DIVB(4);
			PWM->PWM_SCM = 0;										// ensure no sync channels
#endif
			PWMEnabled = true;
		}

		// Work out what clock divider we need for best resolution (we have a customer needing 1 in 6000 @ about 10kHz)
		uint32_t clockFreq = SystemPeripheralClock();
		uint32_t clockShift = 0;
		uint32_t period;
		while ((period = clockFreq/(uint32_t)freq) > 65535 && clockShift <= 12)
		{
			clockFreq >>= 1;
			++clockShift;
		}

		if (period > 65535)
		{
			period = 65535;											// requested PWM frequency must have been less than SystemPeripheralClock/(2^26)
		}

		PWMChanFreq[chanIndex] = freq;
		PWMChanPeriod[chanIndex] = (uint16_t)period;
		const uint32_t duty = ConvertRange(pwmValue, period);

		// Set up the PWM channel
		// On the SAM3X, the update-period register doesn't appear to work; but we no longer support that processor.
		PWMInterface->PWM_DIS = (1ul << chan);						// disable channel
		PWMInterface->PWM_CH_NUM[chan].PWM_CPRDUPD = period;
		PWMInterface->PWM_CH_NUM[chan].PWM_CPRD = period;
		PWMInterface->PWM_CH_NUM[chan].PWM_CMR = clockShift | PWM_CMR_DTHI;
		PWMInterface->PWM_CH_NUM[chan].PWM_CDTYUPD = duty;
		PWMInterface->PWM_CH_NUM[chan].PWM_CDTY = duty;
		PWMInterface->PWM_ENA = (1ul << chan);						// enable channel

		// Now setup the PWM output pin for PWM this channel - do this after configuring the PWM to avoid glitches
		SetPinFunction(pin, GetPeriNumber(pinDesc->pwm));
	}
	else
	{
		PWMInterface->PWM_CH_NUM[chan].PWM_CDTYUPD = ConvertRange(pwmValue, PWMChanPeriod[chanIndex]);
	}
	return true;
}

#if SAM4S
const unsigned int NumTcChannels = 6;
#elif SAM4E
const unsigned int NumTcChannels = 9;
#elif SAME70
const unsigned int NumTcChannels = 12;
#endif

// Map from timer channel to TC channel number
static const uint8_t channelToChNo[NumTcChannels] =
{
	0, 1, 2,
	0, 1, 2,
#if SAM4E || SAME70
	0, 1, 2,
#endif
#if SAME70
	0, 1, 2
#endif
};

// Map from timer channel to TC number
static Tc * const channelToTC[NumTcChannels] =
{
	TC0, TC0, TC0,
	TC1, TC1, TC1,
#if SAM4E || SAME70
	TC2, TC2, TC2,
#endif
#if SAME70
	TC3, TC3, TC3
#endif
};

// Map from timer channel to TIO number
static const uint8_t channelToId[NumTcChannels] =
{
	ID_TC0, ID_TC1, ID_TC2,
	ID_TC3, ID_TC4, ID_TC5,
#if SAM4E || SAME70
	ID_TC6, ID_TC7, ID_TC8,
#endif
#if SAME70
	ID_TC9, ID_TC10, ID_TC11
#endif
};

// Current frequency of each TC channel
static uint16_t TCChanFreq[NumTcChannels] = {0};

static inline void TC_SetCMR_ChannelA(Tc *tc, uint32_t chan, uint32_t v) noexcept
{
	tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xFFF0FFFF) | v;
}

static inline void TC_SetCMR_ChannelB(Tc *tc, uint32_t chan, uint32_t v) noexcept
{
	tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xF0FFFFFF) | v;
}

static inline void TC_WriteCCR(Tc *tc, uint32_t chan, uint32_t v) noexcept
{
	tc->TC_CHANNEL[chan].TC_CCR = v;
}

// AnalogWrite to a TC pin
// Return true if successful, false if we need to fall back to digitalWrite
// WARNING: this will screw up big time if you try to use both the A and B outputs of the same timer at different frequencies.
// The Duet boards use only A outputs, so this is OK.
static bool AnalogWriteTc(Pin pin, const PinDescriptionBase * const pinDesc, float ulValue, PwmFrequency freq) noexcept
pre(0.0 <= ulValue; ulValue <= 1.0)
pre((pinDesc.ulPinAttribute & PIN_ATTR_TIMER) != 0)
{
	const uint32_t chan = (uint32_t)GetDeviceNumber(pinDesc->tc);
	if (freq == 0)
	{
		TCChanFreq[chan] = freq;
		return false;
	}
	else
	{
		Tc* const chTC = channelToTC[chan];
		const uint32_t chNo = channelToChNo[chan];
		const bool doInit = (TCChanFreq[chan] != freq);

		if (doInit)
		{
			TCChanFreq[chan] = freq;

			// Enable the peripheral clock to this timer
			pmc_enable_periph_clk(channelToId[chan]);

			// Set up the timer mode and top count
#if SAM4S || SAME70
			// The timer/counters are only 16 bits wide on the SAM4S and SAME70 so we need to use a higher prescaler
			tc_init(chTC, chNo,
							TC_CMR_TCCLKS_TIMER_CLOCK4 |			// clock is MCLK/128 (SAM4S) or peripheral_clock/128 (SAME70)
							TC_CMR_WAVE |         					// Waveform mode
							TC_CMR_WAVSEL_UP_RC | 					// Counter running up and then down when equals to RC
							TC_CMR_EEVT_XC0 |     					// Set external events from XC0 (this allows TIOB to be an output)
							TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
							TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR |
							TC_CMR_ASWTRG_SET | TC_CMR_BSWTRG_SET);	// Software trigger will let us set the output high
			const uint32_t top = min<uint32_t>((SystemPeripheralClock()/128)/(uint32_t)freq, 65535);	// with 120MHz clock (SAM4S) this varies between 14 @ 65.535kHz and 65535 @ 14.3Hz
#else
			tc_init(chTC, chNo,
							TC_CMR_TCCLKS_TIMER_CLOCK2 |			// clock is MCLK/8 to save a little power and avoid overflow later on
							TC_CMR_WAVE |         					// Waveform mode
							TC_CMR_WAVSEL_UP_RC | 					// Counter running up and reset when equals to RC
							TC_CMR_EEVT_XC0 |     					// Set external events from XC0 (this allows TIOB to be an output)
							TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
							TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR |
							TC_CMR_ASWTRG_SET | TC_CMR_BSWTRG_SET);	// Software trigger will let us set the output high
			const uint32_t top = (SystemPeripheralClock()/8)/(uint32_t)freq;	// with 120MHz clock this varies between 228 @ 65.535kHz and 15 million @ 1Hz
#endif
			// The datasheet doesn't say directly how the period relates to the RC value, but from measurement it seems that we do not need to subtract one from top
			tc_write_rc(chTC, chNo, top);

			// When using TC channels to do PWM control of heaters with active low outputs on the Duet WiFi, if we don't take precautions
			// then we get a glitch straight after initialising the channel, because the compare output starts in the low state.
			// To avoid that, set the output high here if a high PWM was requested.
			if (ulValue >= 0.5)
			{
				TC_WriteCCR(chTC, chan, TC_CCR_SWTRG);
			}
		}

		const uint32_t threshold = ConvertRange(ulValue, tc_read_rc(chTC, chNo));
		if (threshold == 0)
		{
			if (((uint32_t)GetOutputNumber(pinDesc->tc) & 1) == 0)
			{
				tc_write_ra(chTC, chNo, 1);
				TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR);
			}
			else
			{
				tc_write_rb(chTC, chNo, 1);
				TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
			}
		}
		else
		{
			if (((uint32_t)GetOutputNumber(pinDesc->tc) & 1) == 0)
			{
				tc_write_ra(chTC, chNo, threshold);
				TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
			}
			else
			{
				tc_write_rb(chTC, chNo, threshold);
				TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET);
			}
		}

		if (doInit)
		{
			SetPinFunction(pin, GetPeriNumber(pinDesc->tc));
			tc_start(chTC, chNo);
		}
	}
	return true;
}

// Analog write to DAC, PWM, TC or plain output pin
// Setting the frequency of a TC or PWM pin to zero resets it so that the next call to AnalogOut with a non-zero frequency
// will re-initialise it. The pinMode function relies on this.
void AnalogOut::Write(Pin pin, float ulValue, PwmFrequency freq) noexcept
{
	const PinDescriptionBase * const pinDesc = AppGetPinDescription(pin);
	if (pinDesc == nullptr || std::isnan(ulValue))
	{
		return;
	}

	ulValue = constrain<float>(ulValue, 0.0, 1.0);

	if (pinDesc->pwm != PwmOutput::none)
	{
		if (AnalogWritePwm(pin, pinDesc, ulValue, freq))
		{
			return;
		}
	}
	else if (pinDesc->tc != TcOutput::none)
	{
		if (AnalogWriteTc(pin, pinDesc, ulValue, freq))
		{
			return;
		}
	}

	// Fall back to digital write
	pinMode(pin, (ulValue < 0.5) ? OUTPUT_LOW : OUTPUT_HIGH);
}

// End
