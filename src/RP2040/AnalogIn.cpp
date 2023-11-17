/*
 * AnalogIn_SAMC21.cpp
 *
 *  Created on: 20 Aug 2019
 *      Author: David
 */

#ifdef RTOS

#include <CoreIO.h>

#if !RP2040
# error Wrong processor!
#endif

#include "AnalogIn.h"
#include <RTOSIface/RTOSIface.h>
#include <DmacManager.h>
#include <Cache.h>
#include <General/Bitmap.h>

#include <hardware/adc.h>
#include <hardware/dma.h>
#include <hardware/structs/adc.h>

constexpr uint32_t AdcConversionTimeout = 5;		// milliseconds

static uint32_t conversionsStarted = 0;
static uint32_t conversionsCompleted = 0;
static uint32_t conversionTimeouts = 0;
static uint32_t errors = 0;

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

	AdcClass(DmaChannel p_dmaChan, DmaPriority p_dmaPrio) noexcept;

	bool ConversionDone() noexcept;
	bool EnableChannel(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept;
	bool SetCallback(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept;
	bool IsChannelEnabled(unsigned int chan) const noexcept;
	uint16_t ReadChannel(unsigned int chan) const noexcept { return resultsByChannel[chan]; }
	void Exit() noexcept;

	void EnableTemperatureSensor() noexcept;

	void ResultReadyCallback(DmaCallbackReason reason) noexcept;

	bool StartConversion() noexcept;
	void ExecuteCallbacks() noexcept;

protected:
	bool InternalEnableChannel(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept;
	void ReInit() noexcept;

	static void DmaCompleteCallback(CallbackParameter cp, DmaCallbackReason reason) noexcept;

	static constexpr size_t NumAdcChannels = 5;			// number of channels per ADC
	static constexpr size_t MaxSequenceLength = 5;		// the maximum length of the read sequence

	volatile uint32_t channelsEnabled;
	volatile TaskHandle taskToWake;
	uint32_t whenLastConversionStarted;

	const DmaChannel dmaChan;
	const DmaPriority dmaPrio;
	volatile uint8_t numChannelsEnabled;
	volatile State state;
	volatile DmaCallbackReason dmaFinishedReason;

	AnalogInCallbackFunction callbackFunctions[NumAdcChannels];
	CallbackParameter callbackParams[NumAdcChannels];
	uint32_t ticksPerCall[NumAdcChannels];
	uint32_t ticksAtLastCall[NumAdcChannels];
	volatile uint16_t results[MaxSequenceLength];
	volatile uint16_t resultsByChannel[NumAdcChannels];
};

AdcClass::AdcClass(DmaChannel p_dmaChan, DmaPriority p_dmaPrio) noexcept
	: channelsEnabled(0), taskToWake(nullptr), whenLastConversionStarted(0),
	  dmaChan(p_dmaChan), dmaPrio(p_dmaPrio), numChannelsEnabled(0), state(State::noChannels)
{
	dma_channel_claim(dmaChan);
	for (size_t i = 0; i < NumAdcChannels; ++i)
	{
		callbackFunctions[i] = nullptr;
		callbackParams[i].u32 = 0;
		resultsByChannel[i] = 0;
	}
}

// Shut down the ADC, making it safe to terminate the AnalogIn task
void AdcClass::Exit() noexcept
{
	taskToWake = nullptr;
	DmacManager::DisableCompletedInterrupt(dmaChan);		// disable the reader completed interrupt
	DmacManager::DisableChannel(dmaChan);					// disable the reader DMA
	dma_channel_unclaim(dmaChan);
}

// Try to enable this ADC on the specified pin returning true if successful
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
	if (IsChannelEnabled(chan))
	{
		AtomicCriticalSectionLocker lock;

		callbackFunctions[chan] = fn;
		callbackParams[chan] = param;
		ticksPerCall[chan] = p_ticksPerCall;
		ticksAtLastCall[chan] = millis();
		return true;
	}
	return false;
}

bool AdcClass::IsChannelEnabled(unsigned int chan) const noexcept
{
	return (channelsEnabled & (1ul << chan)) != 0;
}

// Indirect callback from the DMA controller ISR
void AdcClass::ResultReadyCallback(DmaCallbackReason reason) noexcept
{
    hw_clear_bits(&adc_hw->cs, ADC_CS_START_MANY_BITS);		// stop the ADC (but it will already have started doing another conversion)
	dmaFinishedReason = reason;
	state = State::ready;
	++conversionsCompleted;
	DmacManager::DisableChannel(dmaChan);					// disable the sequencer DMA, just in case it is out of sync
	if (taskToWake != nullptr)
	{
		TaskBase::GiveFromISR(taskToWake);
	}
}

// Callback from the DMA controller ISR
/*static*/ void AdcClass::DmaCompleteCallback(CallbackParameter cp, DmaCallbackReason reason) noexcept
{
	static_cast<AdcClass *>(cp.vp)->ResultReadyCallback(reason);
}

// Check whether the conversion was successful
bool AdcClass::ConversionDone() noexcept
{
	if (state == State::ready)
	{
		if (dmaFinishedReason == DmaCallbackReason::complete)
		{
			return true;
		}

		++errors;
	}
	return false;
}


void AdcClass::ReInit() noexcept
{
	// Initialise the ADC hardware
	adc_init();
	adc_set_temp_sensor_enabled(true);				// need to do this here because the call to adc_init disables it
	adc_fifo_setup(true /*fifo enabled*/, true /*dreq enabled*/, 1 /*dreq threshold*/, true /*set bit 15 in results if error*/, false /*don't reduce to 8 bits*/);

	// Initialise the DMAC to read the result
	DmacManager::DisableChannel(dmaChan);
	DmacManager::SetBtctrl(dmaChan, (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS);
	DmacManager::SetSourceAddress(dmaChan, &adc_hw->fifo);
	DmacManager::SetInterruptCallback(dmaChan, DmaCompleteCallback, CallbackParameter(this));
	DmacManager::SetTriggerSource(dmaChan, DmaTrigSource::adc);
}

// A note on ADC timings.
// The maximum clock frequency of the ADC is 16MHz. We could probably generate this by running the DPLL at 96MHz, but for simplicity we run the ADC at 48MHz/4 = 12MHz.
// Each conversion takes 17 clocks with comparator offset compensation enabled. That's 1.5167us per conversion.
// We have a maximum of 6 input channels: 3 temperature, Vref, Vssa, and the Z probe. If we enable all of them, that's 8.5us per set of conversions.
// If we average 128 readings in hardware, a full set of conversions takes 1.088ms, a little longer than the time between tick interrupts.
// Averaging 64 readings seems to give us lower noise.
bool AdcClass::InternalEnableChannel(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept
{
	if (chan < NumAdcChannels)
	{
		TaskCriticalSectionLocker lock;

		// Set up the ADC
		callbackFunctions[chan] = fn;
		callbackParams[chan] = param;
		ticksPerCall[chan] = p_ticksPerCall;
		ticksAtLastCall[chan] = millis();
		resultsByChannel[chan] = 0;
		if ((channelsEnabled & 1ul << chan) == 0)
		{
			channelsEnabled |= 1ul << chan;
			++numChannelsEnabled;

			if (numChannelsEnabled == 1)
			{
				// First channel is being enabled, so initialise the ADC
				ReInit();
				state = State::starting;
			}
		}

		return true;
	}

	return false;
}

// Start a conversion if we are not already doing one and have channels to convert, or timeout an existing conversion
bool AdcClass::StartConversion() noexcept
{
	const size_t numChannelsConverting = numChannelsEnabled;			// capture volatile variable to ensure we use a consistent value
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

	DmacManager::DisableChannel(dmaChan);

	while ((adc_hw->cs & ADC_CS_READY_BITS) == 0)	// wait until the extra conversion completes
	{
		delay(1);
	}

	adc_fifo_drain();								// make sure no result pending
	adc_set_round_robin(channelsEnabled);
	adc_select_input(LowestSetBit(channelsEnabled));

	// Set up DMA to read the results our of the ADC into the results array
	DmacManager::SetDestinationAddress(dmaChan, results);
	DmacManager::SetDataLength(dmaChan, numChannelsConverting);

	dmaFinishedReason = DmaCallbackReason::none;
	DmacManager::EnableCompletedInterrupt(dmaChan);
	DmacManager::EnableChannel(dmaChan, dmaPrio);

	state = State::converting;
    hw_set_bits(&adc_hw->cs, ADC_CS_START_MANY_BITS);
	++conversionsStarted;
	whenLastConversionStarted = millis();
	return true;
}

void AdcClass::ExecuteCallbacks() noexcept
{
	Cache::InvalidateAfterDMAReceive(results, sizeof(results));
	TaskCriticalSectionLocker lock;

	const uint32_t now = millis();
	const volatile uint16_t *p = results;
	uint32_t channelsPreviouslyEnabled = (adc_hw->cs & ADC_CS_RROBIN_BITS) >> ADC_CS_RROBIN_LSB;
	for (size_t i = 0; i < NumAdcChannels; ++i)
	{
		if ((channelsPreviouslyEnabled & 1u) != 0)
		{
			const uint16_t currentResult = *p++;
			resultsByChannel[i] = currentResult;
			if (now - ticksAtLastCall[i] >= ticksPerCall[i])
			{
				ticksAtLastCall[i] = now;
				const AnalogInCallbackFunction fn = callbackFunctions[i];
				if (fn != nullptr)
				{
					fn(callbackParams[i], currentResult);
				}
			}
		}
		channelsPreviouslyEnabled >>= 1;
	}
}

// ADC instances
static AdcClass *adc;

// Main loop executed by the AIN task
void AnalogIn::TaskLoop(void*) noexcept
{
	// Loop taking readings and processing them
	for (;;)
	{
		bool conversionStarted = false;
		if (adc->ConversionDone())
		{
			adc->ExecuteCallbacks();
		}
		if (adc->StartConversion())
		{
			conversionStarted = true;
		}

		if (conversionStarted)
		{
			TaskBase::Take(100);
			delay(2);
		}
		else
		{
			// ADC not enabled yet, or converting
			delay(10);
		}
	}
}

// Initialise the analog input subsystem. Call this just once.
void AnalogIn::Init(DmaChannel dmaChan, DmaPriority dmaPrio) noexcept
{
	// Create the device instances
	adc = new AdcClass(dmaChan, dmaPrio);
}

// Shut down the analog system. making it safe to terminate the AnalogIn task
void AnalogIn::Exit() noexcept
{
	adc->Exit();
}

// Enable analog input on a pin.
// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
// Set ticksPerCall to 0 to get a callback on every reading.
bool AnalogIn::EnableChannel(AdcInput adcin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall) noexcept
{
	return adc->EnableChannel(GetInputNumber(adcin), fn, param, ticksPerCall);
}

// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
// Set ticksPerCall to 0 to get a callback on every reading.
bool AnalogIn::SetCallback(AdcInput adcin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall) noexcept
{
	return adc->SetCallback(GetInputNumber(adcin), fn, param, ticksPerCall);
}

// Return whether or not the channel is enabled
bool AnalogIn::IsChannelEnabled(AdcInput adcin) noexcept
{
	return adc->IsChannelEnabled(GetInputNumber(adcin));
}

// Disable a previously-enabled channel
void AnalogIn::DisableChannel(AdcInput adcin) noexcept
{
	//TODO not implemented yet (do we need it?)
}

uint16_t AnalogIn::ReadChannel(AdcInput adcin) noexcept
{
	return adc->ReadChannel(GetInputNumber(adcin));
}

// Enable an on-chip MCU temperature sensor
void AnalogIn::EnableTemperatureSensor(AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall) noexcept
{
	adc->EnableChannel(GetInputNumber(AdcInput::adc0_tempSense), fn, param, ticksPerCall);
}

void AnalogIn::GetDebugInfo(uint32_t &convsStarted, uint32_t &convsCompleted, uint32_t &convTimeouts, uint32_t& errs) noexcept
{
	convsStarted = conversionsStarted;
	convsCompleted = conversionsCompleted;
	convTimeouts = conversionTimeouts;
	errs = errors;
}

#endif

// End
