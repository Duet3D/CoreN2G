/*
 * Interrupts.cpp
 *
 *  Created on: 6 Jul 2019
 *      Author: David
 *  Adapted for STM32 by DC, 2026-07-08
 */

#include "Interrupts.h"

struct InterruptCallback
{
	StandardCallbackFunction func;
	CallbackParameter param;

	InterruptCallback() noexcept : func(nullptr) { }
};

// On the STM32H5 we have 16 external interrupts shared between multiple pins. Only one of those pins may be programmed to generate an interrupt.
// Therefore we will have a clash if we try to attach an interrupt to two pins that use the same EXINT.
// The pin table ensures that only one pin is flagged as able to use each EXINT.

static InterruptCallback exintCallbacks[16];
static InterruptMode exintModes[16];

static void DisableExint(unsigned int exint) noexcept
{
	// Clear the enable bits
	const uint32_t thisExintMask = 1ul << exint;
	const uint32_t otherExintMask = ~thisExintMask;
	EXTI->RTSR1 &= otherExintMask;
	EXTI->FTSR1 &= otherExintMask;

	// Clear the pending bits
	EXTI->RPR1 &= otherExintMask;
	EXTI->FPR1 &= otherExintMask;
}

static void EnableExint(unsigned int exint) noexcept
{
	// STM32 doesn't support level-triggered interrupts
	// TODO to implement modes 'low' and 'high' we should read the pin
	const uint32_t mask = 1ul << exint;
	switch (exintModes[exint])
	{
	case InterruptMode::falling:
		EXTI->FPR1 |= mask;						// clear any existing pending interrupt
		[[fallthrough]];
	case InterruptMode::low:
		EXTI->FTSR1 |= mask;
		break;

	case InterruptMode::rising:
		EXTI->RPR1 |= mask;						// clear any existing pending interrupt
		[[fallthrough]];
	case InterruptMode::high:
		EXTI->RTSR1 |= mask;
		break;

	case InterruptMode::change:
		EXTI->RPR1 |= mask;						// clear any existing pending interrupt
		EXTI->FPR1 |= mask;						// clear any existing pending interrupt
		EXTI->RTSR1 |= mask;
		EXTI->FTSR1 |= mask;
		break;

	default:
		break;
	}
}

void InitialiseExints() noexcept
{
	for (InterruptMode& m : exintModes)
	{
		m = InterruptMode::none;
	}

	for (unsigned int i = 0; i < 16; ++i)
	{
		DisableExint(i);
	}
}

// Attach an interrupt to the specified pin returning true if successful
bool AttachPinInterrupt(Pin pin, StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param, bool enable) noexcept
{
	const PinDescriptionBase * const pinDesc = AppGetPinDescription(pin);
	if (pinDesc == nullptr)
	{
		return false;
	}

	const ExintNumber exint = pinDesc->exintNumber;
	if (exint >= 16)
	{
		return false;
	}

	// Clear any existing selection
	DisableExint(exint);

	// Enable the interrupt in the NVIC
	const IRQn irqn = (IRQn)(EXTI0_IRQn + exint);
	NVIC_ClearPendingIRQ(irqn);
	NVIC_EnableIRQ(irqn);

	// Configure the input multiplexer
	const uint32_t regNumber = exint >> 2;
	volatile uint32_t *const intSelReg = reinterpret_cast<volatile uint32_t*>(EXTI_BASE + 0x60 + (regNumber << 2));
	const unsigned int shift = (exint & 3) << 3;
	const uint32_t mask = 0x000F << shift;
	*intSelReg = (*intSelReg & ~mask) | (GpioPortNumber(pin) << shift);

	exintModes[exint] = mode;
	exintCallbacks[exint].func = callback;
	exintCallbacks[exint].param = param;

	if (enable)
	{
		EnableExint(exint);
	}

	return true;
}

void DetachPinInterrupt(Pin pin) noexcept
{
	const PinDescriptionBase * const pinDesc = AppGetPinDescription(pin);
	if (pinDesc != nullptr)
	{
		const ExintNumber exint = pinDesc->exintNumber;
		if (exint < 16)
		{
			DisableExint(exint);
			exintCallbacks[exint].func = nullptr;
		}
	}
}

// Enable an interrupt that has already been attached.
// We also clear the interrupt flag, otherwise a previous level may be remembered even after the level has returned to the non-interrupting value.
void EnablePinInterrupt(Pin pin) noexcept
{
	const PinDescriptionBase * const pinDesc = AppGetPinDescription(pin);
	if (pinDesc != nullptr)
	{
		const ExintNumber exint = pinDesc->exintNumber;
		if (exint < 16)
		{
			EnableExint(exint);
		}
	}
}

// Disable an interrupt that has already been attached. Also clears any pending interrupt.
void DisablePinInterrupt(Pin pin) noexcept
{
	const PinDescriptionBase * const pinDesc = AppGetPinDescription(pin);
	if (pinDesc != nullptr)
	{
		const ExintNumber exint = pinDesc->exintNumber;
		if (exint < 16)
		{
			DisableExint(exint);
		}
	}
}

// Common EXINT handler
static inline void CommonExintHandler(size_t exintNumber) noexcept
{
	const uint32_t mask = 1ul << exintNumber;
	switch (exintModes[exintNumber])
	{
	case InterruptMode::falling:
	case InterruptMode::low:
		EXTI->FPR1 |= mask;
		break;

	case InterruptMode::rising:
	case InterruptMode::high:
		EXTI->RPR1 |= mask;
		break;

	case InterruptMode::change:
		EXTI->RPR1 |= mask;
		EXTI->FPR1 |= mask;
		break;

	default:
		break;
	}

	const InterruptCallback& cb = exintCallbacks[exintNumber];
	if (cb.func != nullptr)
	{
		cb.func(cb.param);
	}
}

extern "C" void EXTI0_IRQHandler() noexcept
{
	CommonExintHandler(0);
}

extern "C" void EXTI1_IRQHandler() noexcept
{
	CommonExintHandler(1);
}

extern "C" void EXTI2_IRQHandler() noexcept
{
	CommonExintHandler(2);
}

extern "C" void EXTI3_IRQHandler() noexcept
{
	CommonExintHandler(3);
}

extern "C" void EXTI4_IRQHandler() noexcept
{
	CommonExintHandler(4);
}

extern "C" void EXTI5_IRQHandler() noexcept
{
	CommonExintHandler(5);
}

extern "C" void EXTI6_IRQHandler() noexcept
{
	CommonExintHandler(6);
}

extern "C" void EXTI7_IRQHandler() noexcept
{
	CommonExintHandler(7);
}

extern "C" void EXTI8_IRQHandler() noexcept
{
	CommonExintHandler(8);
}

extern "C" void EXTI9_IRQHandler() noexcept
{
	CommonExintHandler(9);
}

extern "C" void EXTI10_IRQHandler() noexcept
{
	CommonExintHandler(10);
}

extern "C" void EXTI11_IRQHandler() noexcept
{
	CommonExintHandler(11);
}

extern "C" void EXTI12_IRQHandler() noexcept
{
	CommonExintHandler(12);
}

extern "C" void EXTI13_IRQHandler() noexcept
{
	CommonExintHandler(13);
}

extern "C" void EXTI14_IRQHandler() noexcept
{
	CommonExintHandler(14);
}

extern "C" void EXTI15_IRQHandler() noexcept
{
	CommonExintHandler(15);
}

// End
