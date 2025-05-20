/*
 * Interrupts.h
 *
 *  Created on: 6 Jul 2019
 *      Author: David

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 3 as published by the Free Software Foundation.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SRC_HARDWARE_PININTERRUPTS_H_
#define SRC_HARDWARE_PININTERRUPTS_H_

#include <CoreIO.h>

// Pin change interrupt support

enum class InterruptMode : uint8_t
{
	none = 0,
	low,
	high,
	change,
	falling,
	rising,

#if SAME5x
	debounce = 0x80,
	lowWithDebounce = low | debounce,
	highWithDebounce = high | debounce,
	changeWithDebounce = change | debounce,
	fallingWithDebounce = falling | debounce,
	risingWithDebounce = rising | debounce,
#endif
};

void InitialiseExints() noexcept;
bool AttachPinInterrupt(Pin pin, StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param, bool enable = true) noexcept;
void DetachPinInterrupt(Pin pin) noexcept;
void EnablePinInterrupt(Pin pin) noexcept;
void DisablePinInterrupt(Pin pin) noexcept;
ExintNumber AttachEvent(Pin pin, InterruptMode mode, bool enableFilter) noexcept;
void DetachEvent(Pin pin) noexcept;

#if SAME5x
bool ReadDebouncedPin(Pin pin) noexcept;			// read the pin state after it has passed through the interrupt debouncer
#endif

#endif /* SRC_HARDWARE_PININTERRUPTS_H_ */
