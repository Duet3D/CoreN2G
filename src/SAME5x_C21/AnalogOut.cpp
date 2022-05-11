/*
 * AnalogOut.cpp
 *
 *  Created on: 9 Jul 2019
 *      Author: David
 */

#include "AnalogOut.h"

#if SAME5x

# include <hri_tc_e54.h>
# include <hri_tcc_e54.h>
# include <hri_mclk_e54.h>

constexpr unsigned int TcGclkNum = GclkNum60MHz;
constexpr uint32_t TcGclkFreq = 60000000;

#elif SAMC21

# include <hri_tc_c21.h>
# include <hri_tcc_c21.h>
# include <hri_mclk_c21.h>

constexpr unsigned int TcGclkNum = GclkNum48MHz;
constexpr uint32_t TcGclkFreq = 48000000;

#else
# error Unsupported processor
#endif

namespace AnalogOut
{

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
	// Return the prescaler register value
	static uint32_t ChoosePrescaler(uint16_t freq, unsigned int counterBits, uint32_t& top) noexcept
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

	// Write PWM to the specified TC device. 'output' may be 0 or 1.
	static bool AnalogWriteTc(Pin pin, unsigned int device, unsigned int output, GpioPinFunction peri, float val, PwmFrequency freq) noexcept
	{
		static volatile Tc* const TcDevices[] =
		{
			TC0, TC1, TC2, TC3, TC4,
#if SAME5x
			TC5, TC6, TC7
#endif
		};

		static uint16_t tcFreq[ARRAY_SIZE(TcDevices)] = { 0 };
		static uint32_t tcTop[ARRAY_SIZE(TcDevices)] = { 0 };

		if (device < ARRAY_SIZE(TcDevices))
		{
			if (freq == 0)
			{
				tcFreq[device] = freq;
				return false;
			}

			volatile Tc * const tcdev = TcDevices[device];
			if (freq != tcFreq[device])
			{
				const uint32_t prescaler = ChoosePrescaler(freq, 16, tcTop[device]);
				if (output == 0)
				{
					// We need to use CC0 for the compare output, so we can't use it to define TOP. We will get a lower frequency than requested.
					// TODO see if we can use 8-bit mode instead
					tcTop[device] = 0xFFFF;
				}

				const uint32_t cc = ConvertRange(val, tcTop[device]);

				if (tcFreq[device] == 0)
				{
					EnableTcClock(device, TcGclkNum);

					// Initialise the TC
					hri_tc_clear_CTRLA_ENABLE_bit(tcdev);
					hri_tc_set_CTRLA_SWRST_bit(tcdev);
					tcdev->COUNT16.CTRLA.bit.MODE = TC_CTRLA_MODE_COUNT16_Val;
					if (output == 0)
					{
						tcdev->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_NPWM_Val;
					}
					else
					{
						tcdev->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MPWM_Val;
					}
				}
				else
				{
					hri_tc_clear_CTRLA_ENABLE_bit(tcdev);
				}
				tcdev->COUNT16.CTRLA.bit.PRESCALER = prescaler;
				if (output != 0)
				{
					tcdev->COUNT16.CC[0].bit.CC = tcTop[device];
					tcdev->COUNT16.CCBUF[0].bit.CCBUF = tcTop[device];
				}
				tcdev->COUNT16.CC[output].bit.CC = cc;
				tcdev->COUNT16.CCBUF[output].bit.CCBUF = cc;
				hri_tc_set_CTRLA_ENABLE_bit(tcdev);
				tcdev->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
				tcFreq[device] = freq;
			}
			else
			{
				// Just update the compare register
				// Don't call hri_tccount16_write_CCBUF_CCBUF_bf here! It loops for up to one TC period waiting for sync.
				const uint16_t cc = ConvertRange(val, tcTop[device]);
				tcdev->COUNT16.CCBUF[output].bit.CCBUF = cc;
			}

			SetPinFunction(pin, peri);
			return true;
		}
		return false;
	}

	static volatile Tcc* const TccDevices[] =
	{
		TCC0, TCC1, TCC2,
#if SAME5x
		TCC3, TCC4
#endif
	};

	// Write PWM to the specified TCC device. 'output' may be 0..5.]
	// If 'bipolar' is true then 'output must be 0, and we invert output 0 but no other outputs.
	static bool AnalogWriteTcc(Pin pin, unsigned int device, unsigned int output, GpioPinFunction peri, float val, PwmFrequency freq, bool bipolar) noexcept
	{
		static constexpr unsigned int TccCounterBits[ARRAY_SIZE(TccDevices)] =
		{
			24, 24, 16,
#if SAME5x
			16, 16
#endif
		};
		static constexpr uint8_t NumChannels[ARRAY_SIZE(TccDevices)] =
		{
#if SAME5x
			6, 4, 3, 2, 2
#elif SAMC21
			4, 2, 2
#endif
		};
		static uint16_t tccFreq[ARRAY_SIZE(TccDevices)] = { 0 };
		static uint32_t tccTop[ARRAY_SIZE(TccDevices)] = { 0 };

		if (device < ARRAY_SIZE(TccDevices))
		{
			if (freq == 0)
			{
				tccFreq[device] = freq;
				return false;
			}

			volatile Tcc * const tccdev = TccDevices[device];
			const unsigned int outputToUse = output % NumChannels[device];		// some TCCs have more outputs than compare channels, so we can't always use the compare channel that corresponds to the output
			if (freq != tccFreq[device])
			{
				const uint32_t prescaler = ChoosePrescaler(freq, TccCounterBits[device], tccTop[device]);
				const uint32_t cc = ConvertRange(val, tccTop[device]);

				if (tccFreq[device] == 0)
				{
					EnableTccClock(device, TcGclkNum);

					// Initialise the TCC
					hri_tcc_clear_CTRLA_ENABLE_bit(tccdev);
					hri_tcc_set_CTRLA_SWRST_bit(tccdev);
					tccdev->CTRLA.bit.PRESCALER = prescaler;
					tccdev->CTRLA.bit.RESOLUTION = 0;
					hri_tcc_write_WAVE_WAVEGEN_bf(tccdev, TCC_WAVE_WAVEGEN_NPWM_Val);
				}
				else
				{
					hri_tcc_clear_CTRLA_ENABLE_bit(tccdev);
					tccdev->CTRLA.bit.PRESCALER = prescaler;
				}

				tccdev->PERBUF.bit.PERBUF = tccTop[device];
				tccdev->PER.bit.PER = tccTop[device];

				tccdev->CCBUF[outputToUse].bit.CCBUF = cc;
				tccdev->CC[outputToUse].bit.CC = cc;

				if (bipolar)
				{
					tccdev->WEXCTRL.reg = TCC_WEXCTRL_OTMX(2);								// distribute channel 0 to all outputs
					tccdev->DRVCTRL.reg = TCC_DRVCTRL_INVEN0 << output;						// invert the output on the first pin, don't invert any other outputs
				}

				hri_tcc_set_CTRLA_ENABLE_bit(tccdev);
				tccdev->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;							// if we don't do this then there is a delay before PWM starts
				tccFreq[device] = freq;
			}
			else
			{
				// Just update the compare register
				const uint32_t cc = ConvertRange(val, tccTop[device]);
				tccdev->CCBUF[outputToUse].bit.CCBUF = cc;
			}

			SetPinFunction(pin, peri);
			return true;
		}
		return false;
	}
}

// Initialise this module
void AnalogOut::Init() noexcept
{
	// Nothing to do yet
}

// Analog write to DAC, PWM, TC or plain output pin
// Setting the frequency of a TC or PWM pin to zero resets it so that the next call to AnalogOut with a non-zero frequency
// will re-initialise it. The pinMode function relies on this.
void AnalogOut::Write(Pin pin, float val, PwmFrequency freq) noexcept
{
	const PinDescriptionBase * const pd = AppGetPinDescription(pin);
	if (pd == nullptr || std::isnan(val))
	{
		return;
	}

	// if writing 0 or 1, do plain digital output for faster response
	if (val > 0.0 && val < 1.0)
	{
		{
			const TcOutput tc = pd->tc;
			if (tc != TcOutput::none)
			{
				if (AnalogWriteTc(pin, GetDeviceNumber(tc), GetOutputNumber(tc), GetPeriNumber(tc), val, freq))
				{
					return;
				}
			}
		}

		{
			const TccOutput tcc = pd->tcc;
			if (tcc != TccOutput::none)
			{
				if (AnalogWriteTcc(pin, GetDeviceNumber(tcc), GetOutputNumber(tcc), GetPeriNumber(tcc), val, freq, false))
				{
					return;
				}
			}
		}
	}

	// Fall back to digital write
	pinMode(pin, (val < 0.5) ? OUTPUT_LOW : OUTPUT_HIGH);
}

#if SAME5x || SAMC21

// Set the beeper to produce a tone using two TCC output pins as differential outputs. Set frequency to zero to stop the beeper.
// Pin1 must be a TCCx.0 pin. Pin2 must be a TXXx.y pin for the same x and y /= 0.
// Caution! This does not check that the pins are valid.
void AnalogOut::Beep(Pin pin1, Pin pin2, PwmFrequency freq) noexcept
{
	if (freq == 0)
	{
		SetPinMode(pin1, OUTPUT_LOW, 0);
		SetPinMode(pin2, OUTPUT_LOW, 0);
	}
	else
	{
		const PinDescriptionBase * const pd1 = AppGetPinDescription(pin1);
		const PinDescriptionBase * const pd2 = AppGetPinDescription(pin2);
		const TccOutput tcc = pd1->tcc;
		if (tcc != TccOutput::none)
		{
			// Writing to the DRVCTRL and WEXCTRL registers works when the device is not enabled, so do it first
			SetPinFunction(pin2, GetPeriNumber(pd2->tcc));							// enable the other output pin
			AnalogWriteTcc(pin1, GetDeviceNumber(tcc), GetOutputNumber(tcc), GetPeriNumber(tcc), 0.5, freq, true);
			return;
		}
	}
}

#endif

// End
