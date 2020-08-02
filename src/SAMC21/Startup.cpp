/*
 * Startup.cpp
 *
 *  Created on: 2 Aug 2020
 *      Author: David
 */

#include <CoreIO.h>
#include <hpl_div.h>

// Symbols defined by the linker
extern uint32_t _sfixed;
//extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;

extern "C" void __libc_init_array() noexcept;

// Forward declaration
static void InitClocks() noexcept;

/**
 * \brief This is the code that gets called on processor reset.
 * To initialize the device, and call the main() routine.
 */
void Reset_Handler() noexcept
{
	// Initialize the relocate segment
	uint32_t *pSrc = &_etext;
	uint32_t *pDest = &_srelocate;

	if (pSrc != pDest)
	{
		for (; pDest < &_erelocate; )
		{
			*pDest++ = *pSrc++;
		}
	}

	// Clear the zero segment
	for (pDest = &_szero; pDest < &_ezero; )
	{
		*pDest++ = 0;
	}

	// Set the vector table base address
	pSrc = (uint32_t *) & _sfixed;
	SCB->VTOR = ((uint32_t) pSrc & SCB_VTOR_TBLOFF_Msk);

	// Initialise the divide and square root accelerator
	_div_init();

	// Initialize the C library
	__libc_init_array();

	// Set up the standard clocks
	InitClocks();

	// Temporarily set up systick so that delayMicroseconds works
	SysTick->LOAD = ((SystemCoreClockFreq/1000) - 1) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1 << SysTick_CTRL_ENABLE_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);

	// Initialise application, which includes setting up any additional clocks
	AppInit();

	// Run the application
	AppMain();

	while (1) { }
}

static void InitClocks() noexcept
{
	// Set the number of flash wait states
	hri_nvmctrl_set_CTRLB_RWS_bf(NVMCTRL, 2);							// 2 wait states needed at clock frequencies >38MHz

	// If we have entered via the bootloader then we have already configured the clocks.
	// We could just leave them alone, but only if we definitely want to use the same clock configuration.
	// So instead we reset the clock configuration.
	// First reset the generic clock generator. This sets all clock generators to default values and the CPU clock to the 48MHz DFLL output.
#if 1
	// 2020-06-03: on the SammyC21 board the software reset of GCLK never completed, so reset it manually
	OSCCTRL->OSC48MCTRL.reg = OSCCTRL_OSC48MCTRL_ENABLE;				// make sure OSC48M is enabled, clear the on-demand bit
	while ((OSCCTRL->STATUS.reg & OSCCTRL_STATUS_OSC48MRDY) == 0) { }	// wait for it to become ready
	GCLK->GENCTRL[0].reg = 0x00000106;									// this is the reset default
	OSCCTRL->OSC48MCTRL.reg = OSCCTRL_OSC48MCTRL_ENABLE | OSCCTRL_OSC48MCTRL_ONDEMAND;		// back to reset default
#else
	// The following code works on Duet3D boards, but it hangs on the SammyC21
	GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
	while ((GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) != 0) { }
#endif

	// If we don't disable DPLL0 here then programming it later fails in a release build
	OSCCTRL->DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->DPLLSYNCBUSY.bit.ENABLE) { }

	// Now it's safe to configure the clocks
	// 32kHz oscillators
	const uint16_t calib = hri_osc32kctrl_read_OSCULP32K_CALIB_bf(OSC32KCTRL);
	hri_osc32kctrl_write_OSCULP32K_reg(OSC32KCTRL, OSC32KCTRL_OSCULP32K_CALIB(calib));
	hri_osc32kctrl_write_RTCCTRL_reg(OSC32KCTRL, OSC32KCTRL_RTCCTRL_RTCSEL(OSC32KCTRL_RTCCTRL_RTCSEL_ULP32K_Val));

	// Get the XOSC details from the application
	const unsigned int xoscFrequency = AppGetXoscFrequency();

	// Crystal oscillator
	const int32_t gain = (xoscFrequency > 16) ? 4 : 3;		// we are assuming that the frequency is >8MHz so we always need gain at least 3
	hri_oscctrl_write_XOSCCTRL_reg(OSCCTRL,
	    	  OSCCTRL_XOSCCTRL_STARTUP(0)
			| (0 << OSCCTRL_XOSCCTRL_AMPGC_Pos)
	        | OSCCTRL_XOSCCTRL_GAIN(gain)
			| (1 << OSCCTRL_XOSCCTRL_RUNSTDBY_Pos)
	        | (0 << OSCCTRL_XOSCCTRL_SWBEN_Pos)
			| (0 << OSCCTRL_XOSCCTRL_CFDEN_Pos)
	        | (1 << OSCCTRL_XOSCCTRL_XTALEN_Pos)
			| (1 << OSCCTRL_XOSCCTRL_ENABLE_Pos));

	hri_oscctrl_write_EVCTRL_reg(OSCCTRL, (0 << OSCCTRL_EVCTRL_CFDEO_Pos));

	while (!hri_oscctrl_get_STATUS_XOSCRDY_bit(OSCCTRL)) { }
	hri_oscctrl_set_XOSCCTRL_AMPGC_bit(OSCCTRL);

	// DPLL
	// We can divide the crystal oscillator by any even number up to 512 to get an input in the range 32kHz to 2MHz for the DPLL
	// To support all crystal frequencies that are an integral number of MHz, we divide down to 500kHz
	hri_oscctrl_write_DPLLRATIO_reg(OSCCTRL, OSCCTRL_DPLLRATIO_LDRFRAC(0) | OSCCTRL_DPLLRATIO_LDR(95));		// multiply input frequency by 96
	hri_oscctrl_write_DPLLCTRLB_reg(OSCCTRL,
	    	  OSCCTRL_DPLLCTRLB_DIV(xoscFrequency - 1)		// divide by 2 * xoscFrequency to get 0.5MHz
			| (0 << OSCCTRL_DPLLCTRLB_LBYPASS_Pos)
	        | OSCCTRL_DPLLCTRLB_LTIME(0)
			| OSCCTRL_DPLLCTRLB_REFCLK(1)					// reference clock is XOSC
	        | (0 << OSCCTRL_DPLLCTRLB_WUF_Pos)
			| (0 << OSCCTRL_DPLLCTRLB_LPEN_Pos)
	        | OSCCTRL_DPLLCTRLB_FILTER(0));
	hri_oscctrl_write_DPLLPRESC_reg(OSCCTRL, OSCCTRL_DPLLPRESC_PRESC(0));
	hri_oscctrl_write_DPLLCTRLA_reg(OSCCTRL,
			  (0 << OSCCTRL_DPLLCTRLA_RUNSTDBY_Pos)
			| (1 << OSCCTRL_DPLLCTRLA_ENABLE_Pos));
	while (!(hri_oscctrl_get_DPLLSTATUS_LOCK_bit(OSCCTRL) || hri_oscctrl_get_DPLLSTATUS_CLKRDY_bit(OSCCTRL))) { }

	// MCLK
	hri_mclk_write_CPUDIV_reg(MCLK, MCLK_CPUDIV_CPUDIV(1));

	// GCLK 0: 48MHz from DPLL
	hri_gclk_write_GENCTRL_reg(GCLK, 0,
		GCLK_GENCTRL_DIV(1) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DPLL96M);

	// GCLK 1: 31250Hz (1MHz divided by 32)
	hri_gclk_write_GENCTRL_reg(GCLK, 1,
		GCLK_GENCTRL_DIV(32 * xoscFrequency) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC);

	// Set up system clock frequency variable for FreeRTOS
	SystemCoreClock = SystemCoreClockFreq;
}

// End
