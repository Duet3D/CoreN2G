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
	rising
};

// Definitions for backwards compatibility with RRF
constexpr InterruptMode INTERRUPT_MODE_NONE = InterruptMode::none;
constexpr InterruptMode INTERRUPT_MODE_LOW = InterruptMode::low;
constexpr InterruptMode INTERRUPT_MODE_HIGH = InterruptMode::high;
constexpr InterruptMode INTERRUPT_MODE_CHANGE = InterruptMode::change;
constexpr InterruptMode INTERRUPT_MODE_FALLING = InterruptMode::falling;
constexpr InterruptMode INTERRUPT_MODE_RISING = InterruptMode::rising;

void InitialiseExints() noexcept;
bool attachInterrupt(Pin pin, StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param) noexcept;
void detachInterrupt(Pin pin) noexcept;
ExintNumber AttachEvent(Pin pin, InterruptMode mode, bool enableFilter) noexcept;
void DetachEvent(Pin pin) noexcept;

#endif /* SRC_HARDWARE_PININTERRUPTS_H_ */
