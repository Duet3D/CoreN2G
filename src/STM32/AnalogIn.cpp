/*
 * AnalogIn.cpp
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifdef RTOS

#include <CoreIO.h>

#if !STM32H5					//TODO extend thius to support ATM32H7
# error Wrong processor!
#endif

#include <AnalogIn.h>
#include <RTOSIface/RTOSIface.h>
#include <CoreNotifyIndices.h>

#include <stm32h5xx_hal_adc.h>
#include <stm32h5xx_hal_rcc.h>

#if STM32H5
constexpr uint8_t TempSenseDevice = 0;				// temp sense is ADC1 (aka device 0) channel 16
constexpr uint8_t TempSenseChannel = 16;			// temp sense is ADC1 channel 16
#endif

constexpr uint32_t AdcPrescaler = 1;				// prescaler used to get the ADC conversion clock from the ADC kernel clock (which is 60MHz for a 240MHz ATM32H5)
constexpr uint32_t AdcConversionTimeout = 5;		// milliseconds

static uint32_t conversionsStarted = 0;
static uint32_t conversionsCompleted = 0;
static uint32_t conversionTimeouts = 0;
static uint32_t errors = 0;
static AnalogIn::AdcTaskHookFunction *taskHookFunction = nullptr;

class AdcClass
{
public:
	enum class State : uint8_t
	{
		noChannels = 0,
		starting,
		converting,
		ready
	};

	AdcClass(ADC_TypeDef * const p_device, IRQn irqn, NvicPriority irqPriority) noexcept;

	bool ConversionDone() noexcept;
	bool EnableChannel(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept;
	bool SetCallback(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept;
	bool IsChannelEnabled(unsigned int chan) const noexcept;
	bool StartConversion() noexcept;
	uint16_t ReadChannel(unsigned int chan) const noexcept { return resultsByChannel[chan]; }
	State GetState() const noexcept { return state; }

	void ExecuteCallbacks() noexcept;

	void Exit() noexcept;

	void ResultReadyInterrupt() noexcept;

private:
	bool InternalEnableChannel(unsigned int chan, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t p_ticksPerCall) noexcept;
	size_t GetChannel(size_t slot) noexcept { return channelNumbers[slot]; }
	void ReInit() noexcept;
	void SetupChannels(size_t numChannels) noexcept;

	static constexpr size_t NumAdcChannels = 20;				// number of channels per ADC including temperature sensor inputs etc.
	static constexpr size_t MaxSequenceLength = 16;				// the maximum length of the read sequence (STM32H5 is limited to 16)

	struct InputRegisters
	{
		uint16_t inputCtrl;
		uint16_t avgCtrl;
	};

	ADC_TypeDef * const device;
	const IRQn irqNumber;
	volatile size_t numChannelsEnabled = 0;						// volatile because multiple tasks access it
	size_t numChannelsConverting = 0;
	volatile uint32_t channelsEnabled = 0;
	volatile TaskHandle taskToWake = nullptr;
	uint32_t whenLastConversionStarted = 0;
	uint32_t currentChannel = 0;
	volatile State state = State::noChannels;
	volatile AnalogInCallbackFunction callbackFunctions[MaxSequenceLength];
	CallbackParameter callbackParams[MaxSequenceLength];
	uint32_t ticksPerCall[MaxSequenceLength];
	uint32_t ticksAtLastCall[MaxSequenceLength];
	uint8_t channelNumbers[MaxSequenceLength];
	volatile uint16_t results[MaxSequenceLength];				// results are written to this buffer
	volatile uint16_t resultsByChannel[NumAdcChannels];			// must be large enough to handle temperature sensor input
};

AdcClass::AdcClass(ADC_TypeDef * const p_device, IRQn irqn, NvicPriority irqPriority) noexcept
	: device(p_device), irqNumber(irqn)
{
	NVIC_DisableIRQ(irqn);
	NVIC_SetPriority(irqn, irqPriority);

	for (size_t i = 0; i < MaxSequenceLength; ++i)
	{
		channelNumbers[i] = 0;
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
	device->IER = 0;								// disable interrupts from the ADC
	NVIC_DisableIRQ(irqNumber);
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

// Initialise or re-initialise the ADC. The ADC clock has already been enabled.
void AdcClass::ReInit() noexcept
{
	device->IER = 0;										// disable all interrupts
	device->CR = ADC_CR_ADVREGEN;							// clear DEEPPWD, set ADVREGEN
	delayMicroseconds(10);									// allow Vref to stabilise
	device->CR = ADC_CR_ADVREGEN | ADC_CR_ADCAL;			// start single-ended calibration
	while (device->CR & ADC_CR_ADCAL) { }					// wait for calibration to complete

	device->CFGR = ADC_CFGR_JQDIS;	//TODO enable DMA when we change this driver to use it
	device->CFGR2 = (2u << ADC_CFGR2_OVSS_Pos) | (5u << ADC_CFGR2_OVSR_Pos) | ADC_CFGR2_ROVSE;	// 64x oversampling, shift result right 2 to get 16-bit result
	device->OR = ADC_OR_OP0;								// channel 0 is not connected to its pin unless this bit is set, see the ADC_OR description in DS14540

	SetupChannels(numChannelsEnabled);

	device->CR |= ADC_CR_ADEN;
	while ((device->ISR & ADC_ISR_ADRDY) == 0) { }			// I am assuming this wait will be quick so not worth waiting on an interrupt or delaying

	NVIC_ClearPendingIRQ(irqNumber);
	NVIC_EnableIRQ(irqNumber);

	currentChannel = 0;
	state = State::starting;
}

// program the sequence registers to convert the required channels.
// This must only be called when conversions have stopped.
void AdcClass::SetupChannels(size_t numChannels) noexcept
{
	device->SQR1 = (numChannels == 0) ? 0
					: (numChannels - 1)
					  | (channelNumbers[0] << ADC_SQR1_SQ1_Pos) | (channelNumbers[1] << ADC_SQR1_SQ2_Pos) | (channelNumbers[2] << ADC_SQR1_SQ3_Pos) | (channelNumbers[3] << ADC_SQR1_SQ4_Pos);
	device->SQR2 = (channelNumbers[4] << ADC_SQR2_SQ5_Pos) | (channelNumbers[5] << ADC_SQR2_SQ6_Pos) | (channelNumbers[6] << ADC_SQR2_SQ7_Pos)
					| (channelNumbers[7] << ADC_SQR2_SQ8_Pos) | (channelNumbers[8] << ADC_SQR2_SQ9_Pos);
	device->SQR3 = (channelNumbers[9] << ADC_SQR3_SQ10_Pos) | (channelNumbers[10] << ADC_SQR3_SQ11_Pos) | (channelNumbers[11] << ADC_SQR3_SQ12_Pos)
					| (channelNumbers[12] << ADC_SQR3_SQ13_Pos) | ( (channelNumbers[13] << ADC_SQR3_SQ14_Pos));
	device->SQR4 =  (channelNumbers[14] << ADC_SQR4_SQ15_Pos) | (channelNumbers[15] << ADC_SQR4_SQ16_Pos);
	numChannelsConverting = numChannels;
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
		channelNumbers[newChannelNumber] = chan;

		resultsByChannel[chan] = 0;
		channelsEnabled |= 1ul << chan;
		numChannelsEnabled = newChannelNumber + 1;

		if (newChannelNumber == 0)
		{
			ReInit();										// first channel is being enabled, so initialise the ADC
		}
		return true;
	}

	return false;
}

// If no conversion is already in progress and there are channels to convert, start a conversion and return true; else return false
bool AdcClass::StartConversion() noexcept
{
	const size_t numEnabled = numChannelsEnabled;			// capture volatile variable to ensure we use a consistent value
	if (numEnabled == 0)
	{
		return false;
	}

	if (state == State::converting)
	{
		if (millis() - whenLastConversionStarted < AdcConversionTimeout)
		{
			return false;									// let the current conversion continue
		}

		++conversionTimeouts;
		ReInit();
	}
	else if (numChannelsConverting < numEnabled)
	{
		SetupChannels(numEnabled);
		numChannelsConverting = numEnabled;					// this is the number of channels we will convert this time
	}

	taskToWake = TaskBase::GetCallerTaskHandle();

	state = State::converting;
	(void)device->DR;										// make sure no result pending
	currentChannel = 0;
	device->IER = ADC_IER_EOCIE;							// enable result ready interrupt
	device->CR |= ADC_CR_ADSTART;

	++conversionsStarted;
	whenLastConversionStarted = millis();
	return true;
}

void AdcClass::ExecuteCallbacks() noexcept
{
	TaskCriticalSectionLocker lock;
	const uint32_t now = millis();
	for (size_t i = 0; i < numChannelsConverting; ++i)
	{
		const uint16_t currentResult = results[i];
		resultsByChannel[GetChannel(i)] = currentResult;
		if (now - ticksAtLastCall[i] >= ticksPerCall[i])
		{
			ticksAtLastCall[i] = now;
			const AnalogInCallbackFunction fn = callbackFunctions[i];
			if (fn != nullptr)
			{
				fn(callbackParams[i], (uint32_t)currentResult);
			}
		}
	}
}

// Interrupt from the ADC when a result is ready
void AdcClass::ResultReadyInterrupt() noexcept
{
#if 0	// for timing the ISR speed
	fastDigitalWriteHigh(PortDPin(10));
#endif
	const uint16_t result = device->DR;
	if (state == State::converting)
	{
		results[currentChannel++] = result;
		if (currentChannel == numChannelsConverting)
		{
			state = State::ready;
			++conversionsCompleted;
			TaskBase::GiveFromISR(taskToWake, NotifyIndices::AnalogIn);
		}
	}
#if 0	// for timing the ISR speed
	fastDigitalWriteLow(PortDPin(10));
#endif
}

// Check whether the conversion was successful
bool AdcClass::ConversionDone() noexcept
{
	return state == State::ready;
}

// ADC instances
static AdcClass *adcs[2];

// ADC ISRs
extern "C" void ADC1_IRQHandler() noexcept
{
	adcs[0]->ResultReadyInterrupt();
}

extern "C" void ADC2_IRQHandler() noexcept
{
	adcs[1]->ResultReadyInterrupt();
}

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
			if (adc->GetState() != AdcClass::State::noChannels)
			{
				if (adc->ConversionDone())
				{
					adc->ExecuteCallbacks();
				}

				if (adc->StartConversion())
				{
					conversionStarted = true;
				}
			}
		}

		if (taskHookFunction != nullptr)
		{
			taskHookFunction();
		}

		if (conversionStarted)
		{
			TaskBase::TakeIndexed(NotifyIndices::AnalogIn, 100);
			delay(2);			//TEMPORARY! without this the AIN task blocks the MAIN task.
		}
		else
		{
			// No ADCs enabled yet, or all converting. This should only happens during startup, before we enable the voltage monitor(s).
			delay(2);							// don't delay too long here, we want the ADCs to start up quickly once channels are enabled
		}
	}
}

// Initialise the analog input subsystem. Call this just once.
void AnalogIn::Init(NvicPriority interruptPriority) noexcept
{
	// Enable ADC clocks
	__HAL_RCC_ADC_CLK_ENABLE();
	ADC12_COMMON->CCR = ADC_CCR_TSEN | (AdcPrescaler << ADC_CCR_PRESC_Pos) | (0x03 << ADC_CCR_CKMODE_Pos);

	// Create the device instances
	adcs[0] = new AdcClass(ADC1, ADC1_IRQn, interruptPriority);
	adcs[1] = new AdcClass(ADC2, ADC2_IRQn, interruptPriority);
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
bool AnalogIn::EnableChannel(AdcInput adcin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall) noexcept
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
bool AnalogIn::SetCallback(AdcInput adcin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall) noexcept
{
	const unsigned int deviceNumber = GetDeviceNumber(adcin);
	if (deviceNumber < ARRAY_SIZE(adcs))				// this test handles AdcInput::none as well as out-of-range ADC numbers
	{
		return adcs[deviceNumber]->SetCallback(GetInputNumber(adcin), fn, param, ticksPerCall);
	}
	return false;
}

// Return whether or not the channel is enabled
bool AnalogIn::IsChannelEnabled(AdcInput adcin) noexcept
{
	const unsigned int deviceNumber = GetDeviceNumber(adcin);
	if (deviceNumber < ARRAY_SIZE(adcs))				// this test handles AdcInput::none as well as out-of-range ADC numbers
	{
		return adcs[deviceNumber]->IsChannelEnabled(GetInputNumber(adcin));
	}

	return false;
}

// Disable a previously-enabled channel
void AnalogIn::DisableChannel(AdcInput adcin) noexcept
{
	//TODO not implemented yet (do we need it?)
}

uint16_t AnalogIn::ReadChannel(AdcInput adcin) noexcept
{
	const unsigned int deviceNumber = GetDeviceNumber(adcin);
	return (deviceNumber < ARRAY_SIZE(adcs)) ? adcs[deviceNumber]->ReadChannel(GetInputNumber(adcin)) : 0;
}

// Enable the on-chip MCU temperature sensor
bool AnalogIn::EnableTemperatureSensor(AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall) noexcept
{
#if STM32H5
	return adcs[TempSenseDevice]->EnableChannel(TempSenseChannel, fn, param, ticksPerCall);
#else
# error Unsupportewd MCU
#endif
	return false;
}

// Return debug information
void AnalogIn::GetDebugInfo(uint32_t &convsStarted, uint32_t &convsCompleted, uint32_t &convTimeouts, uint32_t& errs) noexcept
{
	convsStarted = conversionsStarted;
	convsCompleted = conversionsCompleted;
	convTimeouts = conversionTimeouts;
	errs = errors;
}

AnalogIn::AdcTaskHookFunction *AnalogIn::SetTaskHook(AdcTaskHookFunction *func) noexcept
{
	const AdcTaskHookFunction *oldFunc = taskHookFunction;
	taskHookFunction = func;
	return oldFunc;
}

#endif

// End
