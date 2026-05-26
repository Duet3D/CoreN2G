/*
 * Povide PWM based output using either Hardware or Software PWM.
 * GA 14/8/2020
 */
#include <CoreImp.h> 
#include "HybridPWM.h"
#define PWM_MAX_DUTY_CYCLE          4095
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

static HardwarePWM PWMChans[MaxPWMChannels];

// Create the timers we can use
HardwareTimer Timer2(TIM2);
HardwareTimer Timer3(TIM3);
HardwareTimer Timer4(TIM4);
HardwareTimer Timer8(TIM8);
HardwareTimer Timer12(TIM12);
HardwareTimer Timer13(TIM13);
HardwareTimer Timer14(TIM14);
#if STM32H7
HardwareTimer Timer15(TIM15);
HardwareTimer Timer16(TIM16);
HardwareTimer Timer17(TIM17);
#else
HardwareTimer Timer9(TIM9);
HardwareTimer Timer10(TIM10);
HardwareTimer Timer11(TIM11);
#endif
static HardwareTimer* PWMTimers[] = {
                                        &Timer2, &Timer3, &Timer4, &Timer8, &Timer12, &Timer13, &Timer14,
#if STM32H7
                                        &Timer15, &Timer16, &Timer17
#else
                                        &Timer9, &Timer10, &Timer11
#endif
};

HardwarePWM::HardwarePWM() noexcept : timer(nullptr), channel(0)
{
} 

void HardwarePWM::free() noexcept
{
    //debugPrintf("Free timer chan %d\n", channel);
    if (timer) 
    {
        timer->pause();
        timer->setMode(channel, TIMER_DISABLED);
        timer->resume();
    }
    timer = nullptr;
    channel = 0;
}

HybridPWMBase *HardwarePWM::allocate(Pin pin, uint32_t freq, float value) noexcept
{
    //debugPrintf("HWPWM allocate pin %x, freq %d\n", static_cast<int>(pin), static_cast<int>(freq));
    // first find out if we have a timer for this pin
    TIM_TypeDef *instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
    if (instance == nullptr) return nullptr;
    // Find the timer for this pin
    HardwareTimer *t = nullptr;
    for(uint32_t i = 0; i < ARRAY_SIZE(PWMTimers); i++)
    {
        if (PWMTimers[i]->getHandle()->Instance == instance)
        {
            t = PWMTimers[i];
            break;
        }
    }
    if (t == nullptr)
    {
        debugPrintf("Unable to get hardware timer for pin %x\n", static_cast<int>(pin));
        return nullptr;
    }
    // Get the channel we need
    uint32_t chan = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
    // Now search to see if the timer is already in use
    int free = -1;
    for(uint32_t i = 0; i < MaxPWMChannels; i++)
    {
        if (PWMChans[i].timer == t)
        {
            // channel already in use?
            if (PWMChans[i].channel == chan)
                return nullptr;
            // wrong frequency already being used
            if (PWMChans[i].pwmPin->freq != freq)
                return nullptr;
        }
        if (free < 0 && PWMChans[i].timer == nullptr)
            free = (int)i;
    }
    if (free < 0) return nullptr;
    //debugPrintf("Allocated slot %d timer index %d id %d chan %d to pin %x\n", free, index, get_timer_id((timer_index_t)index), chan, pin);
    // If we get here then we can use the hardware
    PWMChans[free].timer = t;
    PWMChans[free].channel = chan;
    if (freq != 0)
    {
        // set the hardware up ready to go
        t->pause();
        t->setMode(chan, TIMER_OUTPUT_COMPARE_PWM1, pin);
        t->setOverflow(freq, HERTZ_FORMAT);
        t->setCaptureCompare(chan, (uint32_t)(value*PWM_MAX_DUTY_CYCLE), RESOLUTION_12B_COMPARE_FORMAT);
        t->resume();
    }
    return &PWMChans[free];
}

void HardwarePWM::setValue(float value) noexcept
{
    timer->setCaptureCompare(channel, (uint32_t)(value*PWM_MAX_DUTY_CYCLE), RESOLUTION_12B_COMPARE_FORMAT);
}

void HardwarePWM::appendStatus(const StringRef& reply) noexcept
{
    reply.catf(" Tim %d chan %d", static_cast<int>(timer->getTimerId()), static_cast<int>(channel));
}
