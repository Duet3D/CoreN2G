/*
 * AnalogOutput.cpp
 *
 *      Author: David
 *      License: GPL 3.0
 */

#include <AnalogOut.h>
#include <cmath>
#include <cstring>

// Current frequency of each timer
static uint16_t timerFreq[32] = {0};

// Current period of each timer. Only needed for the low-power timers.
//static uint32_t timerPeriod[32] = {0};

// Initialise this module
void AnalogOut::Init() noexcept
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

static void SetCompareValue(TIM_TypeDef *timer, unsigned int channel, uint32_t val) noexcept
{
	switch (channel)
	{
	case 0:		timer->CCR1 = val; break;
	case 1:		timer->CCR2 = val; break;
	case 2:		timer->CCR3 = val; break;
	case 3:		timer->CCR4 = val; break;
#if STM32H7
	case 4:		timer->CCR5 = val; break;
	case 5:		timer->CCR6 = val; break;
#endif
	}
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
		if (freq == 0)
		{
			timerFreq[timerNumber] = freq;
			// Fall through to digital output
		}
		else
		{
			const bool reprogram = (freq != timerFreq[timerNumber]);
			if (reprogram)
			{
				EnableTimerClock(timerNumber);
				timerFreq[timerNumber] = freq;
			}

			const unsigned int channel = GetTimerChannel(tout);
			if (IsLowPowerTimer(timerNumber))
			{
#if 1
				// The low power timer doesn't allow us to use the full PWM frequency range because of the limited prescaling (min. 28Hz @ 240MHz clock)
				// Let's avoid using it if we can. For now fall through to digitalWrite.
#else
				LPTIM_TypeDef *const lpTimer = GetLowPowerHardwareTimer(timerNumber);
				if (reprogram)
				{
					//TODO
				}
				(void)lpTimer;	//TODO
#endif
			}
			else
			{
				TIM_TypeDef *const timer = GetHardwareTimer(timerNumber);
				const uint32_t val = ConvertRange(ulValue, 65536);
				if (reprogram)
				{
					timer->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_UIFREMAP | TIM_CR1_ARPE | TIM_CR1_OPM | TIM_CR1_URS | TIM_CR1_UDIS | TIM_CR1_CKD);

					// All regular timers can take a prescale factor of between 1 and 65535. We run all timers in 16-bit compare mode.
					const uint16_t prescaler = (uint16_t)((GetTimerClockFrequency(timerNumber)/(uint32_t)freq) >> 16);
					timer->PSC = prescaler;
					timer->ARR = 0x0000FFFF;

					// Set the PWM mode
					const uint32_t pwmMode = (GetIsOutputInverted(tout)) ? 7 : 6;
					switch (channel)
					{
					case 0:		timer->CCMR1 = (timer->CCMR1 & ~(1u << 16 | 7u << 4))  | pwmMode << 4; break;
					case 1:		timer->CCMR1 = (timer->CCMR1 & ~(1u << 24 | 7u << 12)) | pwmMode << 12; break;
					case 2:		timer->CCMR2 = (timer->CCMR2 & ~(1u << 16 | 7u << 4))  | pwmMode << 4; break;
					case 3:		timer->CCMR2 = (timer->CCMR2 & ~(1u << 24 | 7u << 12)) | pwmMode << 12; break;
#if STM32H7
					case 4:		timer->CCMR3 = (timer->CCMR3 & ~(1u << 16 | 7u << 4))  | pwmMode << 4; break;
					case 5:		timer->CCMR3 = (timer->CCMR3 & ~(1u << 24 | 7u << 12)) | pwmMode << 12; break;
#endif
					}

					SetCompareValue(timer, channel, val);

					// Set the output pin function
					SetPinFunction(pin, GetPinFunction(tout));

					// Start the timer
			        timer->CR1 |= TIM_CR1_CEN;
				}
				else
				{
					SetCompareValue(timer, channel, val);
				}
				return;
			}
		}
	}

	// Fall back to digital write
	SetPinMode(pin, (ulValue < 0.5) ? OUTPUT_LOW : OUTPUT_HIGH);
}

// End
