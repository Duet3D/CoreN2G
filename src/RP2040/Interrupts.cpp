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
#include <hardware/irq.h>
#include <hardware/structs/iobank0.h>

struct InterruptCallback
{
	StandardCallbackFunction func;
	CallbackParameter param;

	InterruptCallback() : func(nullptr) { }
};

constexpr unsigned int NumCallbacks = NumTotalPins;

static InterruptCallback pinCallbacks[NumCallbacks];

extern "C" void PioHandler() noexcept;

/* Configure PIO interrupt sources */
static void __initialize()
{
	irq_set_exclusive_handler(IO_IRQ_BANK0, PioHandler);
	NVIC_DisableIRQ(IO_IRQ_BANK0_IRQn);
	NVIC_ClearPendingIRQ(IO_IRQ_BANK0_IRQn);
	NVIC_EnableIRQ(IO_IRQ_BANK0_IRQn);
}

// Attach an interrupt to the specified pin returning true if successful
bool attachInterrupt(Pin pin, StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param) noexcept
{
	if (pin >= NumCallbacks)
	{
		return false;
	}

	static bool enabled = false;
	if (!enabled)
	{
		__initialize();
		enabled = true;
	}

	gpio_set_irq_enabled(pin, 0x0F, false);					// ensure the interrupt is disabled before we start changing the tables

	// Set callback function and parameter
	pinCallbacks[pin].func = callback;
	pinCallbacks[pin].param = param;

	// Configure the interrupt mode
	switch(mode)
	{
	case InterruptMode::change:
		gpio_set_irq_enabled(pin, 0x0c, true);
		break;

	case InterruptMode::low:
		gpio_set_irq_enabled(pin, 0x01, true);
		break;

	case InterruptMode::high:
		gpio_set_irq_enabled(pin, 0x02, true);
		break;

	case InterruptMode::falling:
		gpio_set_irq_enabled(pin, 0x04, true);
		break;

	case InterruptMode::rising:
		gpio_set_irq_enabled(pin, 0x08, true);
		break;

	default:
		break;
	}

	return true;
}

void detachInterrupt(Pin pin) noexcept
{
	if (pin < NumCallbacks)
	{
		gpio_set_irq_enabled(pin, 0x0F, false);
		pinCallbacks[pin].func = nullptr;
	}
}

// Common PIO interrupt handler
void PioHandler() noexcept
{
	for (unsigned int bank = 0; bank < 4; ++bank)
	{
		uint32_t status = iobank0_hw->proc0_irq_ctrl.ints[bank];		// get the pending interrupts
		iobank0_hw->intr[bank] = status;								// clear the edge triggered interrupts
		unsigned int pin = bank << 3;
		while (status != 0)
		{
			if ((status & 0x0F) && pinCallbacks[pin].func != nullptr && pin < NumCallbacks)
			{
				pinCallbacks[pin].func(pinCallbacks[pin].param);
			}
			status >>= 4;
			++pin;
		}
	}
}

// End
