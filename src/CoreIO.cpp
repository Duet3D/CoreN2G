/*
 * CoreIO.cpp
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 *
 *  Glue to allow some of our C++ functions to be called from C
 */

#include <CoreIO.h>
#include <DmacManager.h>
#include "Interrupts.h"
#include "AnalogIn.h"
#include "AnalogOut.h"

#ifdef RTOS
# include <FreeRTOS.h>
# include <task.h>
#endif

#if SAME5x
# include <hri_wdt_e54.h>
# include <hal_gpio.h>
#elif SAMC21
# include <hri_wdt_c21.h>
# include <hal_gpio.h>
#elif SAM4E || SAM4S || SAME70
# include <asf/sam/drivers/pmc/pmc.h>
# include <asf/sam/drivers/pio/pio.h>
#endif


// Delay for a specified number of CPU clock cycles from the starting time. Return the time at which we actually stopped waiting.
extern "C" uint32_t DelayCycles(uint32_t start, uint32_t cycles) noexcept
{
	const uint32_t reload = (SysTick->LOAD & 0x00FFFFFF) + 1;
	uint32_t now = start;

	// Wait for the systick counter to cycle round until we need to wait less than the reload value
	while (cycles >= reload)
	{
		const uint32_t last = now;
		now = SysTick->VAL & 0x00FFFFFF;
		if (now > last)
		{
			cycles -= reload;
		}
	}

	uint32_t when;
	if (start >= cycles)
	{
		when = start - cycles;
	}
	else
	{
		when = start + reload - cycles;

		// Wait for the counter to cycle again
		while (true)
		{
			const uint32_t last = now;
			now = SysTick->VAL & 0x00FFFFFF;
			if (now > last)
			{
				break;
			}
		}
	}

	// Wait until the counter counts down to 'when' or below, or cycles again
	while (true)
	{
		const uint32_t last = now;
		now = SysTick->VAL & 0x00FFFFFF;
		if (now <= when || now > last)
		{
			return now;
		}
	}
}

/**
 * @brief Set the function of an I/O pin
 *
 * @param p The pin number
 * @param f The required pin function
 */
void SetPinFunction(Pin p, GpioPinFunction f) noexcept
{
#if SAME5x || SAMC21
	const uint8_t port = p >> 5;
	const uint8_t pin  = p & 0x1F;
	uint8_t tmp = PORT->Group[port].PMUX[pin >> 1].reg;
	if (pin & 1)
	{
		// Odd numbered pin
		tmp &= ~PORT_PMUX_PMUXO_Msk;
		tmp |= PORT_PMUX_PMUXO((uint8_t)f);
	}
	else
	{
		// Even numbered pin
		tmp &= ~PORT_PMUX_PMUXE_Msk;
		tmp |= PORT_PMUX_PMUXE((uint8_t)f);
	}
	PORT->Group[port].PMUX[pin >> 1].reg = tmp;
	PORT->Group[port].PINCFG[pin].bit.PMUXEN = 1;
#elif SAME70 || SAM4E || SAM4S
	Pio * const p_pio = GpioPort(p);
	const uint32_t mask = (uint32_t)1 << (p & 0x1F);
	p_pio->PIO_IDR = mask;									// disable interrupts on the pin
	uint32_t sr0 = p_pio->PIO_ABCDSR[0];
	uint32_t sr1 = p_pio->PIO_ABCDSR[1];
	if ((uint8_t)f & 0x01)
	{
		sr0 |= mask;
	}
	else
	{
		sr0 &= ~mask;
	}
	if ((uint8_t)f & 0x02)
	{
		sr1 |= mask;
	}
	else
	{
		sr1 &= ~mask;
	}
	p_pio->PIO_ABCDSR[0] = sr0;
	p_pio->PIO_ABCDSR[1] = sr1;
	p_pio->PIO_PDR = mask;									// remove the pins from under the control of PIO
#else
# error Unsupported processor
#endif
}

/**
 * @brief Set a pin back to ordinary digital I/O
 *
 * @param p The pin number
 */
void ClearPinFunction(Pin p) noexcept
{
#if SAME5x || SAMC21
	PORT->Group[p >> 5].PINCFG[p & 0x1F].bit.PMUXEN = 0;
#elif SAME70 || SAM4E || SAM4S
	Pio * const p_pio = GpioPort(p);
	const uint32_t mask = GpioMask(p);
	p_pio->PIO_PER = mask;									// put the pins under the control of PIO
#else
# error Unsupported processor
#endif
}

// Enable or disable the pullup[ resistor
void SetPullup(Pin p, bool on) noexcept
{
#if SAM4E || SAM4S || SAME70
	pio_pull_up(GpioPort(p), GpioMask(p), (uint32_t)on) ;
#else
	gpio_set_pin_pull_mode(p, (on) ? GPIO_PULL_UP : GPIO_PULL_OFF);
#endif
}

// IoPort::SetPinMode calls this
void SetPinMode(Pin pin, enum PinMode mode, uint32_t debounceCutoff = 0) noexcept
{
#if SAM4E || SAM4S || SAME70
	constexpr uint32_t PioIds[] =
	{
		ID_PIOA, ID_PIOB, ID_PIOC,
# if SAM4E || SAME70
		ID_PIOD, ID_PIOE,
# endif
	};
#endif

	if (pin < NumTotalPins)
	{
		switch (mode)
		{
		case INPUT:
			ClearPinFunction(pin);
#if SAM4E || SAM4S || SAME70
			pmc_enable_periph_clk(PioIds[GpioPortNumber(pin)]);				// enable peripheral for clocking input *
			GpioPort(pin)->PIO_PUDR = GpioMask(pin);						// turn off pullup
			GpioPort(pin)->PIO_PPDDR = GpioMask(pin);						// turn off pulldown
			pio_set_input(GpioPort(pin), GpioMask(pin), (debounceCutoff == 0) ? 0 : PIO_DEBOUNCE);
			if (debounceCutoff != 0)
			{
				pio_set_debounce_filter(GpioPort(pin), GpioMask(pin), debounceCutoff);	// enable debounce filter with specified cutoff frequency
			}
#else
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
#endif
			break;

		case INPUT_PULLUP:
			ClearPinFunction(pin);
#if SAM4E || SAM4S || SAME70
			pmc_enable_periph_clk(PioIds[GpioPortNumber(pin)]);				// enable peripheral for clocking input *
			GpioPort(pin)->PIO_PUER = GpioMask(pin);						// turn on pullup
			GpioPort(pin)->PIO_PPDDR = GpioMask(pin);						// turn off pulldown
			pio_set_input(GpioPort(pin), GpioMask(pin), (debounceCutoff == 0) ? 0 : PIO_DEBOUNCE);
			if (debounceCutoff != 0)
			{
				pio_set_debounce_filter(GpioPort(pin), GpioMask(pin), debounceCutoff);	// enable debounce filter with specified cutoff frequency
			}
#else
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_UP);
#endif
			break;

		case INPUT_PULLDOWN:
			ClearPinFunction(pin);
#if SAM4E || SAM4S || SAME70
			pmc_enable_periph_clk(PioIds[GpioPortNumber(pin)]);				// enable peripheral for clocking input *
			GpioPort(pin)->PIO_PPDDR = GpioMask(pin);						// turn off pullup
			GpioPort(pin)->PIO_PPDER = GpioMask(pin);						// turn on pulldown
			pio_set_input(GpioPort(pin), GpioMask(pin), (debounceCutoff == 0) ? 0 : PIO_DEBOUNCE);
			if (debounceCutoff != 0)
			{
				pio_set_debounce_filter(GpioPort(pin), GpioMask(pin), debounceCutoff);	// enable debounce filter with specified cutoff frequency
			}
#else
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_DOWN);
#endif
			break;

		case OUTPUT_LOW:
			ClearPinFunction(pin);
#if SAM4E || SAM4S || SAME70
			pio_set_output(GpioPort(pin), GpioMask(pin), 0, 0, 0);
			// If all pins are output, disable PIO Controller clocking, reduce power consumption
			if (GpioPort(pin)->PIO_OSR == 0xffffffff)
			{
				pmc_disable_periph_clk(PioIds[GpioPortNumber(pin)]);
			}
#else
			gpio_set_pin_level(pin, false);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
#endif
			break;

		case OUTPUT_HIGH:
			ClearPinFunction(pin);
#if SAM4E || SAM4S || SAME70
			pio_set_output(GpioPort(pin), GpioMask(pin), 1, 0, 0);
			// If all pins are output, disable PIO Controller clocking, reduce power consumption
			if (GpioPort(pin)->PIO_OSR == 0xffffffff)
			{
				pmc_disable_periph_clk(PioIds[GpioPortNumber(pin)]);
			}
#else
			gpio_set_pin_level(pin, true);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
#endif
			break;

		case AIN:
#if SAM4E || SAM4S || SAME70
			GpioPort(pin)->PIO_PUDR = GpioMask(pin);						// turn off pullup
			GpioPort(pin)->PIO_PPDDR = GpioMask(pin);						// turn off pulldown
			// Ideally we should record which pins are being used as analog inputs, then we can disable the clock
			// on any PIO that is being used solely for outputs and ADC inputs. But for now we don't do that.
#else
			// The SAME70 errata says we must disable the pullup resistor before enabling the AFEC channel
			gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OFF);				// disable the data input buffer
			SetPinFunction(pin, GpioPinFunction::B);						// ADC is always on peripheral B for the SAMC21 an SAME5x
#endif
			break;

		default:
			break;
		}
	}
}

// C-callable version of SetPinMode
extern "C" void pinMode(Pin pin, enum PinMode mode) noexcept
{
	SetPinMode(pin, mode, 0);
}

// IoPort::ReadPin calls this
extern "C" bool digitalRead(Pin pin) noexcept
{
	return (pin < NumTotalPins) && fastDigitalRead(pin);
}

// IoPort::WriteDigital calls this
extern "C" void digitalWrite(Pin pin, bool high) noexcept
{
	if (pin < NumTotalPins)
	{
		if (high)
		{
			fastDigitalWriteHigh(pin);
		}
		else
		{
			fastDigitalWriteLow(pin);
		}
	}
}

// Tick handler
static volatile uint64_t g_ms_ticks = 0;		// Count of 1ms time ticks

uint32_t millis() noexcept
{
    return (uint32_t)g_ms_ticks;
}

uint64_t millis64() noexcept
{
	const irqflags_t flags = cpu_irq_save();
	const uint64_t ret = g_ms_ticks;			// take a copy with interrupts disabled to guard against rollover while we read it
	cpu_irq_restore(flags);
	return ret;
}

void delay(uint32_t ms) noexcept
{
#ifdef RTOS
	vTaskDelay(ms);
#else
	const uint32_t start = millis();
	while (millis() - start < ms) { }
#endif
}

void CoreSysTick() noexcept
{
	g_ms_ticks++;
}

// Optimised version of memcpy for when we know that the source and destination are 32-bit aligned and a whole number of 32-bit words are to be copied
void __attribute__ ((__optimize__ ("-fno-tree-loop-distribute-patterns"))) memcpyu32(uint32_t *dst, const uint32_t *src, size_t numWords) noexcept
{
	while (numWords >= 4)
	{
		*dst++ = *src++;
		*dst++ = *src++;
		*dst++ = *src++;
		*dst++ = *src++;
		numWords -= 4;
	}

	if ((numWords & 2) != 0)
	{
		*dst++ = *src++;
		*dst++ = *src++;
	}

	if ((numWords & 1) != 0)
	{
		*dst++ = *src++;
	}
}

#if SAME5x || SAME70

// Random number generator
static void RandomInit()
{
#if SAME5x
	hri_mclk_set_APBCMASK_TRNG_bit(MCLK);
	hri_trng_set_CTRLA_ENABLE_bit(TRNG);
#elif SAME70
	pmc_enable_periph_clk(ID_TRNG);
	TRNG->TRNG_IDR = TRNG_IDR_DATRDY;							// Disable all interrupts
	TRNG->TRNG_CR = TRNG_CR_KEY(0x524e47) | TRNG_CR_ENABLE;		// Enable TRNG with security key (required)
#endif
}

#endif

void CoreInit() noexcept
{
#if SAME5x || SAMC21
	DmacManager::Init();
	InitialiseExints();
#endif

#if SAME5x || SAME70
	RandomInit();
#endif
}

void WatchdogInit() noexcept
{
#if SAME5x || SAMC21
	hri_mclk_set_APBAMASK_WDT_bit(MCLK);
	delayMicroseconds(5);
	hri_wdt_write_CTRLA_reg(WDT, 0);
	hri_wdt_write_CONFIG_reg(WDT, WDT_CONFIG_PER_CYC1024);		// about 1 second
	hri_wdt_write_EWCTRL_reg(WDT, WDT_EWCTRL_EWOFFSET_CYC512);	// early warning control, about 0.5 second
	hri_wdt_set_INTEN_EW_bit(WDT);								// enable early earning interrupt
	hri_wdt_write_CTRLA_reg(WDT, WDT_CTRLA_ENABLE);
#elif SAME70
	// This assumes the slow clock is running at 32.768 kHz, watchdog frequency is therefore 32768 / 128 = 256 Hz
	constexpr uint16_t watchdogTicks = 256;						// about 1 second
	WDT->WDT_MR = WDT_MR_WDRSTEN | WDT_MR_WDV(watchdogTicks) | WDT_MR_WDD(watchdogTicks);
#endif
}

void WatchdogReset() noexcept
{
#if SAME5x || SAMC21
	// If we kick the watchdog too often, sometimes it resets us. It uses a 1024Hz nominal clock, so presumably it has to be reset less often than that.
	if ((((uint32_t)g_ms_ticks) & 0x07) == 0 && (WDT->SYNCBUSY.reg & WDT_SYNCBUSY_CLEAR) == 0)
	{
		WDT->CLEAR.reg = 0xA5;
	}
#elif SAME70 || SAM4E || SAM4S
	WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;
#endif
}

#if SAM4E || SAME70

void WatchdogResetSecondary() noexcept
{
	constexpr uint32_t RSWDT_KEY_PASSWORD = 0xC4000000;
	RSWDT->RSWDT_CR = RSWDT_KEY_PASSWORD | RSWDT_CR_WDRSTT;
}

#endif

void Reset() noexcept
{
	SCB->AIRCR = (0x5FA << 16) | (1u << 2);						// reset the processor
	for (;;) { }
}

#if SAME5x || SAMC21

// Enable a GCLK. This function doesn't allow access to some GCLK features, e.g. the DIVSEL or OOV or RUNSTDBY bits.
// Only GCLK1 can have a divisor greater than 255.
void ConfigureGclk(unsigned int index, GclkSource source, uint16_t divisor, bool enableOutput) noexcept
{
	uint32_t regVal = GCLK_GENCTRL_DIV(divisor) | GCLK_GENCTRL_SRC((uint32_t)source) | GCLK_GENCTRL_GENEN;
	if (divisor & 1u)
	{
		regVal |= 1u << GCLK_GENCTRL_IDC_Pos;
	}
	if (enableOutput)
	{
		regVal |= 1u << GCLK_GENCTRL_OE_Pos;
	}

	GCLK->GENCTRL[index].reg = regVal;
	while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK) { }
}

#endif

void EnableTcClock(unsigned int tcNumber, unsigned int gclkNum) noexcept
{
#if SAME5x || SAMC21
	static constexpr uint8_t TcClockIDs[] =
	{
		TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID,
# if SAME5x
		TC5_GCLK_ID, TC6_GCLK_ID, TC7_GCLK_ID
# endif
	};

	hri_gclk_write_PCHCTRL_reg(GCLK, TcClockIDs[tcNumber], GCLK_PCHCTRL_GEN(gclkNum) | GCLK_PCHCTRL_CHEN);

	switch (tcNumber)
	{
# if SAME5x
	case 0:	MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC0; break;
	case 1:	MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC1; break;
	case 2:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC2; break;
	case 3:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC3; break;
	case 4:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4; break;
	case 5:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC5; break;
	case 6: MCLK->APBDMASK.reg |= MCLK_APBDMASK_TC6; break;
	case 7: MCLK->APBDMASK.reg |= MCLK_APBDMASK_TC7; break;
# elif SAMC21
	case 0:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC0; break;
	case 1:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC1; break;
	case 2:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC2; break;
	case 3:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC3; break;
	case 4:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4; break;
# else
#  error Unsupported processor
# endif
	}
#elif SAME70 || SAM4E || SAM4S
	// Map from timer channel to TIO ID
	static const uint8_t ChannelToId[] =
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

	if (tcNumber < ARRAY_SIZE(ChannelToId))
	{
		pmc_enable_periph_clk(ChannelToId[tcNumber]);
	}
#else
# error Unsupported processor
#endif
}

#if SAME5x || SAMC21

void EnableTccClock(unsigned int tccNumber, unsigned int gclkNum) noexcept
{
	static constexpr uint8_t TccClockIDs[] =
	{
		TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID,
#if SAME5x
		TCC3_GCLK_ID, TCC4_GCLK_ID
#endif
	};

	hri_gclk_write_PCHCTRL_reg(GCLK, TccClockIDs[tccNumber], GCLK_PCHCTRL_GEN(gclkNum) | GCLK_PCHCTRL_CHEN);

	switch (tccNumber)
	{
#if SAME5x
	case 0:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC0; break;
	case 1:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC1; break;
	case 2:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2; break;
	case 3:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC3; break;
	case 4:	MCLK->APBDMASK.reg |= MCLK_APBDMASK_TCC4; break;
#elif SAMC21
	case 0:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC0; break;
	case 1:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC1; break;
	case 2:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2; break;
#else
# error Unsupported processor
#endif
	}
}

#endif

// Get the analog input channel that a pin uses
AnalogChannelNumber PinToAdcChannel(Pin p) noexcept
{
	const PinDescriptionBase * const pinDesc = AppGetPinDescription(p);
	return (pinDesc == nullptr) ? AdcInput::none : pinDesc->adc;
}

#if SAMC21

// Get the SDADC channel that a pin uses
AnalogChannelNumber PinToSdAdcChannel(Pin p) noexcept
{
	const PinDescriptionBase * const pinDesc = AppGetPinDescription(p);
	return (pinDesc == nullptr) ? AdcInput::none : pinDesc->sdadc;
}

#endif

// Core random number generator
extern "C" uint32_t random32() noexcept
{
#if SAME5x

	// Use the true random number generator peripheral
	while (!hri_trng_get_INTFLAG_reg(TRNG, TRNG_INTFLAG_DATARDY)) { }		// Wait until data ready
	return hri_trng_read_DATA_reg(TRNG);

#elif SAME70

	while (!(TRNG->TRNG_ISR & TRNG_ISR_DATRDY)) {}
	return (uint32_t)TRNG->TRNG_ODATA;

#else

	static bool isInitialised = false;

	if (!isInitialised)
	{
		srand(SysTick->VAL);
		isInitialised = true;
	}

	return rand();

#endif
}

// End
