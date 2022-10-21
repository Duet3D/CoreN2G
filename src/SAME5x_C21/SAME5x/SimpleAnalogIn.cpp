/*
 * SimpleAnalogIn.cpp
 *
 *  Created on: 19 Oct 2022
 *      Author: David
 */

/*
 * SimpleAnalogIn.cpp
 *
 *  Created on: 12 Jun 2020
 *      Author: David
 */

#include "AnalogIn.h"

#if SAME5x && !defined(RTOS)

# include <hri_adc_e54.h>

constexpr unsigned int AdcGclkNum = GclkNum60MHz;

void AnalogIn::Init(Adc * device)
{
	// Enable ADC clock
	if (device == ADC0)
	{
		hri_mclk_set_APBDMASK_ADC0_bit(MCLK);
		hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, GCLK_PCHCTRL_GEN(AdcGclkNum) | GCLK_PCHCTRL_CHEN);
	}
	else
	{
		hri_mclk_set_APBDMASK_ADC1_bit(MCLK);
		hri_gclk_write_PCHCTRL_reg(GCLK, ADC1_GCLK_ID, GCLK_PCHCTRL_GEN(AdcGclkNum) | GCLK_PCHCTRL_CHEN);
	}

	// Register values we send. These are constant except for INPUTCTRL which changes to select the required ADC channel
	constexpr uint32_t CtrlB = ADC_CTRLB_RESSEL_16BIT;
	constexpr uint32_t RefCtrl = ADC_REFCTRL_REFSEL_INTVCC1;
	constexpr uint32_t AvgCtrl = ADC_AVGCTRL_SAMPLENUM_64;
	constexpr uint32_t SampCtrl = ADC_SAMPCTRL_OFFCOMP;

	hri_adc_write_CTRLA_reg(device, ADC_CTRLA_SWRST);
	hri_adc_wait_for_sync(device, ADC_SYNCBUSY_SWRST);

	hri_adc_write_CTRLA_reg(device, ADC_CTRLA_PRESCALER_DIV8);			// GCLK1 is 60MHz, divided by 8 is 7.5MHz
	hri_adc_write_CTRLB_reg(device, CtrlB);
	hri_adc_write_REFCTRL_reg(device,  RefCtrl);
	hri_adc_write_EVCTRL_reg(device, 0);
	hri_adc_write_INPUTCTRL_reg(device, ADC_INPUTCTRL_MUXNEG_GND);
	hri_adc_write_AVGCTRL_reg(device, AvgCtrl);
	hri_adc_write_SAMPCTRL_reg(device, SampCtrl);						// this also extends the sample time to 4 ADC clocks
	hri_adc_write_WINLT_reg(device, 0);
	hri_adc_write_WINUT_reg(device, 0xFFFF);
	hri_adc_write_GAINCORR_reg(device, 1u << 11);
	hri_adc_write_OFFSETCORR_reg(device, 0);
	hri_adc_write_DSEQCTRL_reg(device, 0);
	hri_adc_write_DBGCTRL_reg(device, 0);

	// Load CALIB with NVM data calibration results
	do
	{
		uint32_t biasComp, biasRefbuf, biasR2R;
		if (device == ADC0)
		{
			biasComp = (*reinterpret_cast<const uint32_t*>(ADC0_FUSES_BIASCOMP_ADDR) & ADC0_FUSES_BIASCOMP_Msk) >> ADC0_FUSES_BIASCOMP_Pos;
			biasRefbuf = (*reinterpret_cast<const uint32_t*>(ADC0_FUSES_BIASREFBUF_ADDR) & ADC0_FUSES_BIASREFBUF_Msk) >> ADC0_FUSES_BIASREFBUF_Pos;
			biasR2R = (*reinterpret_cast<const uint32_t*>(ADC0_FUSES_BIASR2R_ADDR) & ADC0_FUSES_BIASR2R_Msk) >> ADC0_FUSES_BIASR2R_Pos;
		}
		else if (device == ADC1)
		{
			biasComp = (*reinterpret_cast<const uint32_t*>(ADC1_FUSES_BIASCOMP_ADDR) & ADC1_FUSES_BIASCOMP_Msk) >> ADC1_FUSES_BIASCOMP_Pos;
			biasRefbuf = (*reinterpret_cast<const uint32_t*>(ADC1_FUSES_BIASREFBUF_ADDR) & ADC1_FUSES_BIASREFBUF_Msk) >> ADC1_FUSES_BIASREFBUF_Pos;
			biasR2R = (*reinterpret_cast<const uint32_t*>(ADC1_FUSES_BIASR2R_ADDR) & ADC1_FUSES_BIASR2R_Msk) >> ADC1_FUSES_BIASR2R_Pos;
		}
		else
		{
			break;
		}
		hri_adc_write_CALIB_reg(device, ADC_CALIB_BIASCOMP(biasComp) | ADC_CALIB_BIASREFBUF(biasRefbuf) | ADC_CALIB_BIASR2R(biasR2R));
	} while (false);

	device->INTENCLR.reg = 0x07;										// disable all interrupts
	hri_adc_set_CTRLA_ENABLE_bit(device);
	hri_adc_wait_for_sync(device, ADC_SYNCBUSY_ENABLE);
}

uint16_t AnalogIn::ReadChannel(Adc * device, uint8_t channel)
{
	device->INPUTCTRL.bit.MUXPOS = channel;
	device->INTFLAG.reg = ADC_INTFLAG_RESRDY;							// clear RESRDY bit
	device->SWTRIG.reg = ADC_SWTRIG_START;
	while (!device->INTFLAG.bit.RESRDY) { }
	return device->RESULT.reg;
}

void AnalogIn::Disable(Adc * device)
{
	hri_adc_clear_CTRLA_ENABLE_bit(device);
	hri_adc_wait_for_sync(device, ADC_SYNCBUSY_ENABLE);
}

#endif

// End
