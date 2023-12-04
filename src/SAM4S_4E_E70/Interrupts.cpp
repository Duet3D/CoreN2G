/*
  Copyright (c) 2011-2012 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Interrupts.h>
#include <pmc/pmc.h>

struct InterruptCallback
{
	StandardCallbackFunction _ecv_null func;
	CallbackParameter param;

	// Constructor
	InterruptCallback() noexcept : func(nullptr) { }

	// Execute the callback if the function pointer is not null
	void Execute() noexcept
	{
		if (func != nullptr)
		{
			func(param);
		}
	}
};

#if SAM4E || SAME70
static constexpr unsigned int NumPins = (4 * 32) + 6;		// SAM4E and SAM4S both have only the first 6 pins of PIOE
#elif SAM4S
static constexpr unsigned int NumPins = (3 * 32);			// SAM4S has PIOA thru PIOC only
#endif

static InterruptCallback pinCallbacks[NumPins];

/* Configure PIO interrupt sources */
static void __initialize() noexcept
{
	pmc_enable_periph_clk(ID_PIOA);
	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_EnableIRQ(PIOA_IRQn);

	pmc_enable_periph_clk(ID_PIOB);
	NVIC_DisableIRQ(PIOB_IRQn);
	NVIC_ClearPendingIRQ(PIOB_IRQn);
	NVIC_EnableIRQ(PIOB_IRQn);

	pmc_enable_periph_clk(ID_PIOC);
	NVIC_DisableIRQ(PIOC_IRQn);
	NVIC_ClearPendingIRQ(PIOC_IRQn);
	NVIC_EnableIRQ(PIOC_IRQn);

#ifdef ID_PIOD
	pmc_enable_periph_clk(ID_PIOD);
	NVIC_DisableIRQ(PIOD_IRQn);
	NVIC_ClearPendingIRQ(PIOD_IRQn);
	NVIC_EnableIRQ(PIOD_IRQn);
#endif

#ifdef ID_PIOE
	pmc_enable_periph_clk(ID_PIOE);
	NVIC_DisableIRQ(PIOE_IRQn);
	NVIC_ClearPendingIRQ(PIOE_IRQn);
	NVIC_EnableIRQ(PIOE_IRQn);
#endif
}

// Attach an interrupt to the specified pin returning true if successful
bool attachInterrupt(Pin pin, StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param) noexcept
{
	const PinDescriptionBase * const pinDesc = AppGetPinDescription(pin);
	if (pinDesc == nullptr || pin >= NumPins)
	{
		return false;
	}

	static bool enabled = false;
	if (!enabled)
	{
		__initialize();
		enabled = true;
	}

	// Retrieve pin information
	Pio * const pio = GpioPort(pin);
	const uint32_t mask = GpioMask(pin);
	pio->PIO_IDR = mask;									// ensure the interrupt is disabled before we start changing the tables

	// Set callback function and parameter
	pinCallbacks[pin].func = callback;
	pinCallbacks[pin].param = param;

	// Configure the interrupt mode
	if (mode == InterruptMode::change)
	{
		// Disable additional interrupt mode (detects both rising and falling edges)
		pio->PIO_AIMDR = mask;
	}
	else
	{
		// Enable additional interrupt mode
		pio->PIO_AIMER = mask;

		// Select mode of operation
		switch(mode)
		{
		case InterruptMode::low:
			pio->PIO_LSR = mask;    // "Level" Select Register
			pio->PIO_FELLSR = mask; // "Falling Edge / Low Level" Select Register
			break;

		case InterruptMode::high:
			pio->PIO_LSR = mask;    // "Level" Select Register
			pio->PIO_REHLSR = mask; // "Rising Edge / High Level" Select Register
			break;

		case InterruptMode::falling:
			pio->PIO_ESR = mask;    // "Edge" Select Register
			pio->PIO_FELLSR = mask; // "Falling Edge / Low Level" Select Register
			break;

		case InterruptMode::rising:
			pio->PIO_ESR = mask;    // "Edge" Select Register
			pio->PIO_REHLSR = mask; // "Rising Edge / High Level" Select Register
			break;

		default:
			break;
		}
	}

	// Enable interrupt
	if (mode != InterruptMode::none)
	{
		pio->PIO_IFER = mask;		// enable glitch filter on this pin
		pio->PIO_IER = mask;		// enable interrupt on this pin
	}

	return true;
}

void detachInterrupt(Pin pin) noexcept
{
	const PinDescriptionBase * const pinDesc = AppGetPinDescription(pin);
	if (pinDesc != nullptr && pin < NumPins)
	{
		// Retrieve pin information
		Pio * const pio = GpioPort(pin);
		const uint32_t mask = GpioMask(pin);

		// Disable interrupt
		pio->PIO_IDR = mask;
	}
}

// Common PIO interrupt handler
static void CommonPioHandler(Pio *pio, unsigned int firstPin) noexcept
{
	uint32_t isr = pio->PIO_ISR & pio->PIO_IMR;
	while (isr != 0)
	{
		const unsigned int pos = LowestSetBitNumber(isr);
		if (pos + firstPin < NumPins)
		{
			pinCallbacks[pos + firstPin].Execute();
		}
		isr &= ~(1u << pos);
	}
}

extern "C" void PIOA_Handler(void) noexcept
{
	CommonPioHandler(PIOA, 0);
}

extern "C" void PIOB_Handler(void) noexcept
{
	CommonPioHandler(PIOB, 32);
}

extern "C" void PIOC_Handler(void) noexcept
{
	CommonPioHandler(PIOC, 2 * 32);
}

#ifdef ID_PIOD
extern "C" void PIOD_Handler(void) noexcept
{
	CommonPioHandler(PIOD, 3 * 32);
}
#endif

#ifdef ID_PIOE
extern "C" void PIOE_Handler(void) noexcept
{
	CommonPioHandler(PIOE, 4 * 32);
}
#endif

// End
