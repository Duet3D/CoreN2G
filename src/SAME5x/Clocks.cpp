/*
 * Clocks.cpp
 *
 *  Created on: 1 Aug 2020
 *      Author: David
 */

#include <Clocks.h>

void InitStandardClocks(unsigned int xoscFrequency, unsigned int xoscNumber) noexcept
{
#if 1
	hri_nvmctrl_set_CTRLA_RWS_bf(NVMCTRL, 0);				// rely on the AUTOWS bit
#else
	hri_nvmctrl_set_CTRLA_RWS_bf(NVMCTRL, 6);				// need 6 wait states @ 120MHz for EFP part numbers
	hri_nvmctrl_clear_CTRLA_AUTOWS_bit(NVMCTRL);			// clear the auto WS bit
#endif

	// Initialise 32kHz oscillator
	const uint16_t calib = hri_osc32kctrl_read_OSCULP32K_CALIB_bf(OSC32KCTRL);
	hri_osc32kctrl_write_OSCULP32K_reg(OSC32KCTRL, OSC32KCTRL_OSCULP32K_CALIB(calib));

	// Initialise the XOSC
	const uint32_t imult = (xoscFrequency > 24) ? 6
							: (xoscFrequency > 16) ? 5
								: 4;						// we assume the crystal frequency is always >8MHz
	const uint32_t iptat = 3;								// we assume the crystal frequency is always >8MHz
	hri_oscctrl_write_XOSCCTRL_reg(OSCCTRL, xoscNumber,
			  OSCCTRL_XOSCCTRL_CFDPRESC(3)
			| OSCCTRL_XOSCCTRL_STARTUP(0)
			| (0 << OSCCTRL_XOSCCTRL_SWBEN_Pos)
			| (0 << OSCCTRL_XOSCCTRL_CFDEN_Pos)
			| (0 << OSCCTRL_XOSCCTRL_ENALC_Pos)
			| OSCCTRL_XOSCCTRL_IMULT(imult)
			| OSCCTRL_XOSCCTRL_IPTAT(iptat)
			| (0 << OSCCTRL_XOSCCTRL_LOWBUFGAIN_Pos)
			| (0 << OSCCTRL_XOSCCTRL_ONDEMAND_Pos)
			| (0 << OSCCTRL_XOSCCTRL_RUNSTDBY_Pos)
			| (1 << OSCCTRL_XOSCCTRL_XTALEN_Pos)
			| (1 << OSCCTRL_XOSCCTRL_ENABLE_Pos));

	const uint32_t xoscReadyBit = 1ul << (OSCCTRL_STATUS_XOSCRDY0_Pos + xoscNumber);	// The XOSCRDY1 bit is one position higher than the XOSCRDY0 bit
	while ((OSCCTRL->STATUS.reg & xoscReadyBit) == 0) { }

	// Initialise MCLK
	hri_mclk_write_CPUDIV_reg(MCLK, MCLK_CPUDIV_DIV(MCLK_CPUDIV_DIV_DIV1_Val));

	// Initialise FDPLL0
	// We can divide the crystal oscillator by any even number up to 512 to get an input in the range 32kHz to 3MHz for the DPLL
	// The errata says that at 400kHz and below we can get false unlock indications. So we try to use 1MHz or above.
	uint32_t multiplier;
	uint32_t divisor;										// must be even
	if ((xoscFrequency & 1) == 0)
	{
		divisor = xoscFrequency;							// use 1MHz
		multiplier = 120;
	}
	else if ((xoscFrequency % 5) == 0)						// e.g. 25MHz as used on Duet 3 Mini 5+
	{
		divisor = 2 * (xoscFrequency/5);					// use 2.5MHz
		multiplier = (2 * 120)/5;
	}
	else
	{
		divisor = 2 * xoscFrequency;
		multiplier = 2 * 120;
	}

	hri_oscctrl_write_DPLLRATIO_reg(OSCCTRL, 0,
			  OSCCTRL_DPLLRATIO_LDRFRAC(0)
			| OSCCTRL_DPLLRATIO_LDR(multiplier - 1));
	hri_oscctrl_write_DPLLCTRLB_reg(OSCCTRL, 0,
			  OSCCTRL_DPLLCTRLB_DIV(divisor/2 - 1)
			| (0 << OSCCTRL_DPLLCTRLB_DCOEN_Pos)
			| OSCCTRL_DPLLCTRLB_DCOFILTER(0)
			| (0 << OSCCTRL_DPLLCTRLB_LBYPASS_Pos)
			| OSCCTRL_DPLLCTRLB_LTIME(0)
			| OSCCTRL_DPLLCTRLB_REFCLK(2u + xoscNumber)		// source is XOSC0 or XOSC1
			| (0 << OSCCTRL_DPLLCTRLB_WUF_Pos)
			| OSCCTRL_DPLLCTRLB_FILTER(0));
	hri_oscctrl_write_DPLLCTRLA_reg(OSCCTRL, 0,
			  (0 << OSCCTRL_DPLLCTRLA_RUNSTDBY_Pos)
			| (1 << OSCCTRL_DPLLCTRLA_ENABLE_Pos));

	while (!(hri_oscctrl_get_DPLLSTATUS_LOCK_bit(OSCCTRL, 0) || hri_oscctrl_get_DPLLSTATUS_CLKRDY_bit(OSCCTRL, 0))) { }

	// We must initialise GCLKs 0 and 1 before we touch the DFLL:
	// - GCLK0 is the CPU clock and defaults to the DFLL
	// - GCLK1 is used as the reference when we reprogram the DFLL

	// GCLK0: from FDPLL0 direct
	hri_gclk_write_GENCTRL_reg(GCLK, 0,
			  GCLK_GENCTRL_DIV(1) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DPLL0);

	// GCLK1: XOSC0 divided by 31 * frequency_in_MHz to give 32258Hz
	hri_gclk_write_GENCTRL_reg(GCLK, 1,
			  GCLK_GENCTRL_DIV(31 * xoscFrequency) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_XOSC0_Val + xoscNumber));

	// Initialise DFLL48M in closed loop mode
	hri_gclk_write_PCHCTRL_reg(GCLK, OSCCTRL_GCLK_ID_DFLL48, GCLK_PCHCTRL_GEN_GCLK1_Val | GCLK_PCHCTRL_CHEN);		// set GCLK1 as DFLL reference
	hri_oscctrl_write_DFLLCTRLA_reg(OSCCTRL, 0);

	hri_oscctrl_write_DFLLMUL_reg(OSCCTRL, OSCCTRL_DFLLMUL_CSTEP(4) | OSCCTRL_DFLLMUL_FSTEP(4) | OSCCTRL_DFLLMUL_MUL(48 * 31));
	while (hri_oscctrl_get_DFLLSYNC_DFLLMUL_bit(OSCCTRL)) { }

	hri_oscctrl_write_DFLLCTRLB_reg(OSCCTRL, 0);
	while (hri_oscctrl_get_DFLLSYNC_DFLLCTRLB_bit(OSCCTRL)) { }

	hri_oscctrl_write_DFLLCTRLA_reg(OSCCTRL, (0 << OSCCTRL_DFLLCTRLA_RUNSTDBY_Pos) | OSCCTRL_DFLLCTRLA_ENABLE);
	while (hri_oscctrl_get_DFLLSYNC_ENABLE_bit(OSCCTRL)) { }

	hri_oscctrl_write_DFLLVAL_reg(OSCCTRL, hri_oscctrl_read_DFLLVAL_reg(OSCCTRL));
	while (hri_oscctrl_get_DFLLSYNC_DFLLVAL_bit(OSCCTRL)) { }

	hri_oscctrl_write_DFLLCTRLB_reg(OSCCTRL,
			  (0 << OSCCTRL_DFLLCTRLB_WAITLOCK_Pos) | (0 << OSCCTRL_DFLLCTRLB_BPLCKC_Pos)
			| (0 << OSCCTRL_DFLLCTRLB_QLDIS_Pos) | (0 << OSCCTRL_DFLLCTRLB_CCDIS_Pos)
			| (0 << OSCCTRL_DFLLCTRLB_USBCRM_Pos) | (0 << OSCCTRL_DFLLCTRLB_LLAW_Pos)
			| (0 << OSCCTRL_DFLLCTRLB_STABLE_Pos) | (1u << OSCCTRL_DFLLCTRLB_MODE_Pos));
	while (hri_oscctrl_get_DFLLSYNC_DFLLCTRLB_bit(OSCCTRL)) { }

	// Initialise the other GCLKs
	// GCLK3: FDPLL0 divided by 2, 60MHz for peripherals that need less than 120MHz
	hri_gclk_write_GENCTRL_reg(GCLK, 3,
			  GCLK_GENCTRL_DIV(2) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DPLL0);

	// GCLK4: DFLL48M for CAN and step timer
	hri_gclk_write_GENCTRL_reg(GCLK, 4,
			  GCLK_GENCTRL_DIV(1) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL_Val);
}

// End
