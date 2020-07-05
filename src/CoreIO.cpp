/*
 * Core_C.cpp
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
#include <peripheral_clk_config.h>

// IoPort::SetPinMode calls this
extern "C" void pinMode(Pin pin, enum PinMode mode) noexcept
{
	if (pin < NumTotalPins)
	{
		switch (mode)
		{
		case INPUT:
			ClearPinFunction(pin);
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
			break;

		case INPUT_PULLUP:
			ClearPinFunction(pin);
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_UP);
			break;

		case INPUT_PULLDOWN:
			ClearPinFunction(pin);
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_DOWN);
			break;

		case OUTPUT_LOW:
			ClearPinFunction(pin);
			gpio_set_pin_level(pin, false);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
			break;

		case OUTPUT_HIGH:
			ClearPinFunction(pin);
			gpio_set_pin_level(pin, true);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
			break;

		case AIN:
			// The SAME70 errata says we must disable the pullup resistor before enabling the AFEC channel
			gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OFF);		// disable the data input buffer
			SetPinFunction(pin, GpioPinFunction::B);				// ADC is always on peripheral B
			break;

		default:
			break;
		}
	}
}

// IoPort::ReadPin calls this
extern "C" bool digitalRead(Pin pin) noexcept
{
	if (pin < NumTotalPins)
	{
		const uint8_t port = GPIO_PORT(pin);
		const uint32_t pinMask = 1U << GPIO_PIN(pin);
		return (hri_port_read_IN_reg(PORT, port) & pinMask) != 0;
	}

	return false;
}

// IoPort::WriteDigital calls this
extern "C" void digitalWrite(Pin pin, bool high) noexcept
{
	if (pin < NumTotalPins)
	{
		const uint8_t port = GPIO_PORT(pin);
		const uint32_t pinMask = 1U << GPIO_PIN(pin);
		if (high)
		{
			hri_port_set_OUT_reg(PORT, port, pinMask);
		}
		else
		{
			hri_port_clear_OUT_reg(PORT, port, pinMask);
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
	hal_atomic_t flags;
	atomic_enter_critical(&flags);
	const uint64_t ret = g_ms_ticks;			// take a copy with interrupts disabled to guard against rollover while we read it
	atomic_leave_critical(&flags);
	return ret;
}

void CoreSysTick() noexcept
{
	g_ms_ticks++;
}

#if SAME5x

// Random number generator
static void RandomInit()
{
	hri_mclk_set_APBCMASK_TRNG_bit(MCLK);
	hri_trng_set_CTRLA_ENABLE_bit(TRNG);
}

#endif

void CoreInit() noexcept
{
	DmacManager::Init();

#if SAME5x
	RandomInit();
#endif

	// Initialise the I/O subsystem
	InitialiseExints();
}

void WatchdogInit() noexcept
{
	hri_mclk_set_APBAMASK_WDT_bit(MCLK);
	delayMicroseconds(5);
	hri_wdt_write_CTRLA_reg(WDT, 0);
	hri_wdt_write_CONFIG_reg(WDT, WDT_CONFIG_PER_CYC1024);		// about 1 second
	hri_wdt_write_EWCTRL_reg(WDT, WDT_EWCTRL_EWOFFSET_CYC512);	// early warning control, about 0.5 second
	hri_wdt_set_INTEN_EW_bit(WDT);								// enable early earning interrupt
	hri_wdt_write_CTRLA_reg(WDT, WDT_CTRLA_ENABLE);
}

void watchdogReset() noexcept
{
	// If we kick the watchdog too often, sometimes it resets us. It uses a 1024Hz nominal clock, so presumably it has to be reset less often than that.
	if ((((uint32_t)g_ms_ticks) & 0x07) == 0)
	{
		WDT->CLEAR.reg = 0xA5;
	}
}

void Reset() noexcept
{
	SCB->AIRCR = (0x5FA << 16) | (1u << 2);						// reset the processor
	for (;;) { }
}

// Enable a GCLK. This function doesn't allow access to some GCLK features, e.g. the DIVSEL or OOV or RUNSTDBY bits.
// Only GCLK1 can have a divisor greater than 255.
void EnableGclk(unsigned int index, GclkSource source, uint16_t divisor, bool enableOutput) noexcept
{
	uint32_t regVal = GCLK_GENCTRL_DIV(divisor) | (uint32_t)source | (1u << GCLK_GENCTRL_GENEN_Pos);
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

void EnableTcClock(unsigned int tcNumber, uint32_t gclkVal) noexcept
{
	static constexpr uint8_t TcClockIDs[] =
	{
		TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID,
#if SAME5x
		TC5_GCLK_ID, TC6_GCLK_ID, TC7_GCLK_ID
#endif
	};

	hri_gclk_write_PCHCTRL_reg(GCLK, TcClockIDs[tcNumber], gclkVal | (1 << GCLK_PCHCTRL_CHEN_Pos));

	switch (tcNumber)
	{
#if SAME5x
	case 0:	MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC0; break;
	case 1:	MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC1; break;
	case 2:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC2; break;
	case 3:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC3; break;
	case 4:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4; break;
	case 5:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC5; break;
	case 6: MCLK->APBDMASK.reg |= MCLK_APBDMASK_TC6; break;
	case 7: MCLK->APBDMASK.reg |= MCLK_APBDMASK_TC7; break;
#elif SAMC21
	case 0:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC0; break;
	case 1:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC1; break;
	case 2:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC2; break;
	case 3:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC3; break;
	case 4:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4; break;
#else
# error Unsupported processor
#endif
	}
}

void EnableTccClock(unsigned int tccNumber, uint32_t gclkVal) noexcept
{
	static constexpr uint8_t TccClockIDs[] =
	{
		TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID,
#if SAME5x
		TCC3_GCLK_ID, TCC4_GCLK_ID
#endif
	};

	hri_gclk_write_PCHCTRL_reg(GCLK, TccClockIDs[tccNumber], gclkVal | (1 << GCLK_PCHCTRL_CHEN_Pos));

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

// Random number generator
int32_t random(uint32_t howbig) noexcept
{
	if (howbig == 0)
	{
		return 0;
	}

#if SAME5x

	while (!hri_trng_get_INTFLAG_reg(TRNG, TRNG_INTFLAG_DATARDY)) { }		// Wait until data ready
	return hri_trng_read_DATA_reg(TRNG) % howbig;

#else
	static bool isInitialised = false;

	if (!isInitialised)
	{
		srand(SysTick->VAL);
		isInitialised = true;
	}

	return rand() % (uint32_t)howbig;
#endif
}

// End
