/*
 * AnalogIn.cpp
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifdef RTOS

#include <CoreIO.h>

#if !SAME5x
# error Wrong processor!
#endif

#include "AnalogIn.h"
#include <RTOSIface/RTOSIface.h>
#include <DmacManager.h>
#include <Cache.h>

#include <hri_adc_e54.h>

constexpr unsigned int AdcGclkNum = GclkNum60MHz;
constexpr uint32_t AdcConversionTimeout = 5;		// milliseconds

static uint32_t conversionsStarted = 0;
static uint32_t conversionsCompleted = 0;
static uint32_t conversionTimeouts = 0;

// Constants that control the DMA sequencing
// The SAME5x errata doc from Microchip say that order to use averaging, we need to include the AVGCTRL register in the sequence even if it doesn't change,
// and that the prescaler must be <= 8 when we use DMA sequencing.

// In order to use averaging, we need to include the AVGCTRL register in the sequence even if it doesn't change (see the SAME5x errata doc from Microchip).
// We have to set the AUTOSTART bit in DmaSeqVal, otherwise the ADC requires one trigger per channel converted.
constexpr size_t DmaDwordsPerChannel = 2;		// the number of DMA registers we write for each channel that we sample
constexpr uint32_t DmaSeqVal = ADC_DSEQCTRL_INPUTCTRL | ADC_DSEQCTRL_AVGCTRL | ADC_DSEQCTRL_AUTOSTART;

// Register values we send. These are constant except for INPUTCTRL which changes to select the required ADC channel
constexpr uint32_t CtrlB = ADC_CTRLB_RESSEL_16BIT;
constexpr uint32_t RefCtrl = ADC_REFCTRL_REFSEL_INTVCC1;
constexpr uint32_t AvgCtrl = ADC_AVGCTRL_SAMPLENUM_64;
constexpr uint32_t SampCtrl = ADC_SAMPCTRL_OFFCOMP;

class AdcClass
{
public:
	enum class State : uint8_t
	{
		noChannels = 0,
		starting,
		idle,
		converting,
		ready
	};

	AdcClass(Adc * const p_device, DmaChannel p_dmaChan, DmaPriority txPriority, DmaPriority rxPriority, DmaTrigSource p_trigSrc) noexcept;

	State GetState() const noexcept { return state; }
	bool EnableChannel(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept;
	bool SetCallback(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept;
	bool IsChannelEnabled(unsigned int chan) const noexcept;
	bool StartConversion() noexcept;
	uint16_t ReadChannel(unsigned int chan) const noexcept { return resultsByChannel[chan]; }
	bool EnableTemperatureSensor(unsigned int sensorNumber, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall) noexcept;

	void ResultReadyCallback(DmaCallbackReason reason) noexcept;
	void ExecuteCallbacks() noexcept;

	void Exit() noexcept;

private:
	bool InternalEnableChannel(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept;
	size_t GetChannel(size_t slot) noexcept { return inputRegisters[DmaDwordsPerChannel * slot] & 0x1F; }
	void ReInit() noexcept;

	static void DmaCompleteCallback(CallbackParameter cp, DmaCallbackReason reason) noexcept;

	static constexpr size_t NumAdcChannels = 32;			// number of channels per ADC including temperature sensor inputs etc.
	static constexpr size_t MaxSequenceLength = 16;			// the maximum length of the read sequence

	Adc * const device;
	const DmaChannel dmaChan;
	const DmaPriority dmaTxPrio, dmaRxPrio;
	const DmaTrigSource trigSrc;
	volatile DmaCallbackReason dmaFinishedReason;
	volatile size_t numChannelsEnabled;						// volatile because multiple tasks access it
	size_t numChannelsConverting;
	volatile uint32_t channelsEnabled;
	volatile TaskHandle taskToWake;
	uint32_t whenLastConversionStarted;
	volatile State state;
	AnalogInCallbackFunction callbackFunctions[MaxSequenceLength];
	CallbackParameter callbackParams[MaxSequenceLength];
	uint32_t ticksPerCall[MaxSequenceLength];
	uint32_t ticksAtLastCall[MaxSequenceLength];
	uint32_t inputRegisters[MaxSequenceLength * DmaDwordsPerChannel];
	volatile uint16_t results[MaxSequenceLength];
	volatile uint16_t resultsByChannel[NumAdcChannels];		// must be large enough to handle PTAT and CTAT temperature sensor inputs
};

AdcClass::AdcClass(Adc * const p_device, DmaChannel p_dmaChan, DmaPriority txPriority, DmaPriority rxPriority, DmaTrigSource p_trigSrc) noexcept
	: device(p_device), dmaChan(p_dmaChan), dmaTxPrio(txPriority), dmaRxPrio(rxPriority), trigSrc(p_trigSrc),
	  numChannelsEnabled(0), numChannelsConverting(0), channelsEnabled(0),
	  taskToWake(nullptr), whenLastConversionStarted(0), state(State::noChannels)
{
	for (size_t i = 0; i < MaxSequenceLength; ++i)
	{
		callbackFunctions[i] = nullptr;
		callbackParams[i].u32 = 0;
	}
	for (volatile uint16_t& r : resultsByChannel)
	{
		r = 0;
	}
}

// Shut down the ADC, making it safe to terminate the AnalogIn task
void AdcClass::Exit() noexcept
{
	taskToWake = nullptr;
	DmacManager::DisableCompletedInterrupt(dmaChan + 1);		// disable the reader completed interrupt
	DmacManager::DisableChannel(dmaChan);						// disable the sequencer DMA
	DmacManager::DisableChannel(dmaChan + 1);					// disable the reader DMA too
}

// Try to enable this ADC on the specified channel returning true if successful
// Only single ended mode with gain x1 is supported
// There is no check to avoid adding the same channel twice. If you do that it will be converted twice.
bool AdcClass::EnableChannel(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept
{
	if (numChannelsEnabled == MaxSequenceLength || chan >= NumAdcChannels)
	{
		return false;
	}

	return InternalEnableChannel(chan, fn, param, p_ticksPerCall);
}

bool AdcClass::SetCallback(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept
{
	for (size_t i = 0; i < numChannelsEnabled; ++i)
	{
		if (GetChannel(i) == chan)
		{
			AtomicCriticalSectionLocker lock;

			callbackFunctions[i] = fn;
			callbackParams[i] = param;
			ticksPerCall[i] = p_ticksPerCall;
			ticksAtLastCall[i] = millis();
			return true;
		}
	}
	return false;
}

bool AdcClass::IsChannelEnabled(unsigned int chan) const noexcept
{
	return (channelsEnabled & (1ul << chan)) != 0;
}

bool AdcClass::EnableTemperatureSensor(unsigned int sensorNumber, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept
{
	if (numChannelsEnabled == MaxSequenceLength || sensorNumber >= 2)
	{
		return false;
	}

	return InternalEnableChannel(sensorNumber + ADC_INPUTCTRL_MUXPOS_PTAT_Val, fn, param, p_ticksPerCall);
}

// Initialise or re-initialise the ADC and DMA channels. The ADC clock has already been enabled.
void AdcClass::ReInit() noexcept
{
	if (!hri_adc_is_syncing(device, ADC_SYNCBUSY_SWRST))
	{
		if (hri_adc_get_CTRLA_reg(device, ADC_CTRLA_ENABLE))
		{
			hri_adc_clear_CTRLA_ENABLE_bit(device);
			hri_adc_wait_for_sync(device, ADC_SYNCBUSY_ENABLE);
		}
		hri_adc_write_CTRLA_reg(device, ADC_CTRLA_SWRST);
	}
	hri_adc_wait_for_sync(device, ADC_SYNCBUSY_SWRST);

	// From the SAME5x errata:
	// 2.1.4 DMA Sequencing
	//	ADC DMA Sequencing with prescaler>8 (ADC->CTRLA.bit.PRESCALER>2) does not produce the expected channel sequence.
	// Workaround
	//  Keep the prescaler setting to a maximum of 8, and use the GCLK Generator divider if more prescaling is needed.
	// 2.1.5 DMA Sequencing
	//  ADC DMA Sequencing with averaging enabled (AVGCTRL.SAMPLENUM>1) without the AVGCTRL bit set (DSEQCTRL.AVGCTRL=0) in the update sequence
	//  does not produce the expected channel sequence.
	// Workaround
	//  Add the AVGCTRL register in the register update list (DSEQCTRL.AVGCTRL=1) and set the desired value in this list.
	hri_adc_write_CTRLA_reg(device, ADC_CTRLA_PRESCALER_DIV8);			// GCLK1 is 60MHz, divided by 8 is 7.5MHz
	hri_adc_write_CTRLB_reg(device, CtrlB);
	hri_adc_write_REFCTRL_reg(device,  RefCtrl);
	hri_adc_write_EVCTRL_reg(device, ADC_EVCTRL_RESRDYEO);
	hri_adc_write_INPUTCTRL_reg(device, ADC_INPUTCTRL_MUXNEG_GND);
	hri_adc_write_AVGCTRL_reg(device, AvgCtrl);
	hri_adc_write_SAMPCTRL_reg(device, SampCtrl);						// this also extends the sample time to 4 ADC clocks
	hri_adc_write_WINLT_reg(device, 0);
	hri_adc_write_WINUT_reg(device, 0xFFFF);
	hri_adc_write_GAINCORR_reg(device, 1u << 11);
	hri_adc_write_OFFSETCORR_reg(device, 0);
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

	// Enable DMA sequencing, updating the input, reference control and average control registers.
	hri_adc_write_DSEQCTRL_reg(device, DmaSeqVal);
	hri_adc_set_CTRLA_ENABLE_bit(device);

	// Set the supply controller to on-demand mode so that we can get at both temperature sensors
	hri_supc_set_VREF_ONDEMAND_bit(SUPC);
	hri_supc_set_VREF_TSEN_bit(SUPC);
	hri_supc_clear_VREF_VREFOE_bit(SUPC);

	// Initialise the DMAC. First the sequencer
	DmacManager::DisableChannel(dmaChan);
	DmacManager::SetDestinationAddress(dmaChan, &device->DSEQDATA.reg);
	DmacManager::SetBtctrl(dmaChan, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_WORD
								| DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_STEPSIZE_X1);
	DmacManager::SetTriggerSource(dmaChan, (DmaTrigSource)((uint8_t)trigSrc + 1));

	// Now the result reader
	DmacManager::DisableChannel(dmaChan + 1);
	DmacManager::SetSourceAddress(dmaChan + 1, const_cast<uint16_t *>(&device->RESULT.reg));
	DmacManager::SetInterruptCallback(dmaChan + 1, DmaCompleteCallback, this);
	DmacManager::SetBtctrl(dmaChan + 1, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_HWORD
								| DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_STEPSIZE_X1);
	DmacManager::SetTriggerSource(dmaChan + 1, trigSrc);
	state = State::starting;
}

bool AdcClass::InternalEnableChannel(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept
{
	if (chan < NumAdcChannels)
	{
		TaskCriticalSectionLocker lock;

		// Set up the ADC
		const size_t newChannelNumber = numChannelsEnabled;
		callbackFunctions[newChannelNumber] = fn;
		callbackParams[newChannelNumber] = param;
		ticksPerCall[newChannelNumber] = p_ticksPerCall;
		ticksAtLastCall[newChannelNumber] = millis();

		// Set up the input registers in the DMA area
		inputRegisters[newChannelNumber * DmaDwordsPerChannel] = (ADC_INPUTCTRL_MUXNEG_GND | (uint32_t)chan) | (CtrlB << 16);
		inputRegisters[newChannelNumber * DmaDwordsPerChannel + 1] = RefCtrl | (AvgCtrl << 16) | (SampCtrl << 24);

		resultsByChannel[chan] = 0;
		channelsEnabled |= 1ul << chan;
		numChannelsEnabled = newChannelNumber + 1;

		if (newChannelNumber == 0)
		{
			// First channel is being enabled, so initialise the ADC
			ReInit();
		}

		return true;
	}

	return false;
}

// If no conversion is already in progress and there are channels to convert, start a conversion and return true; else return false
bool AdcClass::StartConversion() noexcept
{
	numChannelsConverting = numChannelsEnabled;			// capture volatile variable to ensure we use a consistent value
	if (numChannelsConverting == 0)
	{
		return false;
	}

	if (state == State::converting)
	{
		if (millis() - whenLastConversionStarted < AdcConversionTimeout)
		{
			return false;
		}
		++conversionTimeouts;
		ReInit();
	}

	taskToWake = TaskBase::GetCallerTaskHandle();

	// Set up DMA sequencing of the ADC
	DmacManager::DisableChannel(dmaChan + 1);
	DmacManager::DisableChannel(dmaChan);

	(void)device->RESULT.reg;							// make sure no result pending (this is necessary to make it work!)

	DmacManager::SetDestinationAddress(dmaChan + 1, results);
	DmacManager::SetDataLength(dmaChan + 1, numChannelsConverting);

	DmacManager::SetSourceAddress(dmaChan, inputRegisters);
	DmacManager::SetDataLength(dmaChan, numChannelsConverting * DmaDwordsPerChannel);

	{
		InterruptCriticalSectionLocker lock;

		dmaFinishedReason = DmaCallbackReason::none;
		DmacManager::EnableCompletedInterrupt(dmaChan + 1);

		DmacManager::EnableChannel(dmaChan + 1, dmaRxPrio);
		DmacManager::EnableChannel(dmaChan, dmaTxPrio);

		state = State::converting;
		++conversionsStarted;
	}

	whenLastConversionStarted = millis();
	return true;
}

void AdcClass::ExecuteCallbacks() noexcept
{
	Cache::InvalidateAfterDMAReceive(results, sizeof(results));
	TaskCriticalSectionLocker lock;
	const uint32_t now = millis();
	for (size_t i = 0; i < numChannelsConverting; ++i)
	{
		const uint16_t currentResult = results[i];
		resultsByChannel[GetChannel(i)] = currentResult;
		if (now - ticksAtLastCall[i] >= ticksPerCall[i])
		{
			ticksAtLastCall[i] = now;
			if (callbackFunctions[i] != nullptr)
			{
				callbackFunctions[i](callbackParams[i], currentResult);
			}
		}
	}
}

// Indirect callback from the DMA controller ISR
void AdcClass::ResultReadyCallback(DmaCallbackReason reason) noexcept
{
	dmaFinishedReason = reason;
	state = State::ready;
	++conversionsCompleted;
	DmacManager::DisableChannel(dmaChan);			// disable the sequencer DMA, just in case it is out of sync
	DmacManager::DisableChannel(dmaChan + 1);		// disable the reader DMA too
	TaskBase::GiveFromISR(taskToWake);
}

// Callback from the DMA controller ISR
/*static*/ void AdcClass::DmaCompleteCallback(CallbackParameter cp, DmaCallbackReason reason) noexcept
{
	static_cast<AdcClass *>(cp.vp)->ResultReadyCallback(reason);
}

// ADC instances
static AdcClass *adcs[2];

// Main loop executed by the AIN task
void AnalogIn::TaskLoop(void *) noexcept
{
	// Loop taking readings and processing them
	for (;;)
	{
		// Loop through ADCs
		bool conversionStarted = false;
		for (AdcClass* adc : adcs)
		{
			if (adc->GetState() == AdcClass::State::ready)
			{
				adc->ExecuteCallbacks();
			}

			if (adc->StartConversion())
			{
				conversionStarted = true;
			}
		}

		if (conversionStarted)
		{
			TaskBase::Take(100);
			delay(2);
		}
		else
		{
			// No ADCs enabled yet, or all converting
			delay(10);
		}
	}
}

// Initialise the analog input subsystem. Call this just once.
void AnalogIn::Init(DmaChannel dmaChan, DmaPriority txPriority, DmaPriority rxPriority) noexcept
{
	// Enable ADC clocks
	hri_mclk_set_APBDMASK_ADC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, GCLK_PCHCTRL_GEN(AdcGclkNum) | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_APBDMASK_ADC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC1_GCLK_ID, GCLK_PCHCTRL_GEN(AdcGclkNum) | GCLK_PCHCTRL_CHEN);

	// Create the device instances
	adcs[0] = new AdcClass(ADC0, dmaChan, txPriority, rxPriority, DmaTrigSource::adc0_resrdy);
	adcs[1] = new AdcClass(ADC1, dmaChan + 2, txPriority, rxPriority, DmaTrigSource::adc1_resrdy);
}

// Shut down the analog system. making it safe to terminate the AnalogIn task
void AnalogIn::Exit() noexcept
{
	adcs[0]->Exit();
	adcs[1]->Exit();
}

// Enable analog input on a pin.
// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
// Set ticksPerCall to 0 to get a callback on every reading.
bool AnalogIn::EnableChannel(AdcInput adcin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall, bool useAlternateAdc) noexcept
{
	const unsigned int deviceNumber = GetDeviceNumber(adcin);
	if (deviceNumber < ARRAY_SIZE(adcs))				// this test handles AdcInput::none as well as out-of-range ADC numbers
	{
		return adcs[deviceNumber]->EnableChannel(GetInputNumber(adcin), fn, param, ticksPerCall);
	}
	return false;
}

// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
// Set ticksPerCall to 0 to get a callback on every reading.
bool AnalogIn::SetCallback(AdcInput adcin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall, bool useAlternateAdc) noexcept
{
	const unsigned int deviceNumber = GetDeviceNumber(adcin);
	if (deviceNumber < ARRAY_SIZE(adcs))				// this test handles AdcInput::none as well as out-of-range ADC numbers
	{
		return adcs[deviceNumber]->SetCallback(GetInputNumber(adcin), fn, param, ticksPerCall);
	}
	return false;
}

// Return whether or not the channel is enabled
bool AnalogIn::IsChannelEnabled(AdcInput adcin, bool useAlternateAdc) noexcept
{
	const unsigned int deviceNumber = GetDeviceNumber(adcin);
	if (deviceNumber < ARRAY_SIZE(adcs))				// this test handles AdcInput::none as well as out-of-range ADC numbers
	{
		return adcs[deviceNumber]->IsChannelEnabled(GetInputNumber(adcin));
	}

	return false;
}

// Disable a previously-enabled channel
void AnalogIn::DisableChannel(AdcInput adcin, bool useAlternateAdc) noexcept
{
	//TODO not implemented yet (do we need it?)
}

uint16_t AnalogIn::ReadChannel(AdcInput adcin) noexcept
{
	const unsigned int deviceNumber = GetDeviceNumber(adcin);
	return (deviceNumber < ARRAY_SIZE(adcs)) ? adcs[deviceNumber]->ReadChannel(GetInputNumber(adcin)) : 0;
}

// Enable an on-chip MCU temperature sensor
// From the SAME5x-E5x errata document version K:
// 2.23.1 Temperature Sensor
// Both internal temperature sensors, TSENSP and TSENSC, are not supported and should not be used.
// Workaround: None
// Affected Silicon Revisions: A, D
bool AnalogIn::EnableTemperatureSensor(unsigned int sensorNumber, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall, unsigned int adcnum) noexcept
{
	if (adcnum < ARRAY_SIZE(adcs))
	{
		// Set the supply controller to on-demand mode so that we can get at both temperature sensors
		SUPC->VREF.reg |= (SUPC_VREF_ONDEMAND | SUPC_VREF_TSEN | SUPC_VREF_VREFOE);

		return adcs[adcnum]->EnableTemperatureSensor(sensorNumber, fn, param, ticksPerCall);
	}
	return false;
}

// Return debug information
void AnalogIn::GetDebugInfo(uint32_t &convsStarted, uint32_t &convsCompleted, uint32_t &convTimeouts) noexcept
{
	convsStarted = conversionsStarted;
	convsCompleted = conversionsCompleted;
	convTimeouts = conversionTimeouts;
}

#endif

// End
