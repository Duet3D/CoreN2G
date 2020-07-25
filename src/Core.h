/*
 * Core.h
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 *
 * This file contains basic CPU and I/O pin support for the SAME5x (also works with SAMD5x) and SAMC21
 * Use it where we can't include the full CoreIO.h file, for example in C source files
 */

#ifndef SRC_CORE_H_
#define SRC_CORE_H_

// Basic CPU and I/O pin support

#include "ecv.h"
#undef array
#undef assert
#undef result
#undef value

#if defined(__SAME54P20A__) || defined(__SAMD51N19A__)
# include <same54.h>
# define __ARM_ARCH_7EM__	1
# define SAME5x				1
# define SAMC21				0
#elif defined(__SAMC21G18A__)
# include <samc21.h>
# define __ARM_ARCH_6M__	1
# define SAME5x				0
# define SAMC21				1
# define SUPPORT_SDHC		0			// SAMC21 doesn't support SDHC
#else
# error unsupported processor
#endif

#define SAM4E	0
#define SAM4S	0
#define SAM3XA	0
#define SAME70	0

#include <inttypes.h>				// for PRIu32 etc.
#include <ctype.h>

// Types
typedef uint8_t DmaChannel;
typedef uint8_t DmaPriority;
typedef uint8_t Pin;
typedef uint8_t ExintNumber;
typedef uint16_t PwmFrequency;		// type used to represent a PWM frequency. 0 sometimes means "default".
typedef uint32_t NvicPriority;

static const Pin NoPin = 0xFF;

// Standard GCLK numbers
#if SAME5x

static const uint32_t SystemCoreClockFreq = 120000000;

static const unsigned int GclkNum120MHz = 0;
static const unsigned int GclkNum32KHz = 1;		// frequency is approx. 32768Hz
static const unsigned int GclkNum25MHz = 2;		// only on 5LC board
static const unsigned int GclkNum60MHz = 3;
static const unsigned int GclkNum48MHz = 4;

#elif SAMC21

static const uint32_t SystemCoreClockFreq = 48000000;

static const unsigned int GclkNum48MHz = 0;
static const unsigned int GclkNum32KHz = 3;		// frequency is approx. 32768Hz

#endif

// Pin mode enumeration. Would ideally be a C++ scoped enum, but we need to use it from C library functions.
enum PinMode
{
	PIN_MODE_NOT_CONFIGURED = -1,	// used in Platform class to record that the mode for a pin has not been set yet
	INPUT = 0,						// pin is a digital input
	INPUT_PULLUP,					// pin is a digital input with pullup enabled
	INPUT_PULLDOWN,					// pin is a digital input with pulldown enabled
	OUTPUT_LOW,						// pin is an output with initial state LOW
	OUTPUT_HIGH,					// pin is an output with initial state HIGH
	AIN,							// pin is an analog input, digital input buffer is disabled if possible
	SPECIAL,						// pin is used for the special function defined for it in the variant.cpp file
	OUTPUT_PWM_LOW,					// PWM output mode, initially low
	OUTPUT_PWM_HIGH,				// PWM output mode, initially high
};

#ifndef __cplusplus
# include <stdbool.h>
#endif

#define UNUSED(_x)	(void)_x
#define Assert(expr) ((void) 0)

// Macro to give us the number of elements in an array
#ifndef ARRAY_SIZE
# define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof((_x)[0]))
#endif

#ifdef __cplusplus
extern "C" {
#endif

uint32_t millis() noexcept;
uint64_t millis64() noexcept;
void delay(uint32_t millis) noexcept;

void pinMode(Pin pin, enum PinMode mode) noexcept;
bool digitalRead(Pin pin) noexcept;
void digitalWrite(Pin pin, bool high) noexcept;

uint32_t random32(void) noexcept;		// needed by lwip

// Delay for a specified number of CPU clock cycles from the starting time. Return the time at which we actually stopped waiting.
uint32_t DelayCycles(uint32_t start, uint32_t cycles) noexcept;

static inline void delayMicroseconds(uint32_t) noexcept __attribute__((always_inline, unused));
static inline void delayMicroseconds(uint32_t usec) noexcept
{
	(void)DelayCycles(SysTick->VAL & 0x00FFFFFF, usec * (SystemCoreClockFreq/1000000));
}

// Functions and macros to enable/disable interrupts
static inline void cpu_irq_enable() noexcept
{
	__DMB();
	__enable_irq();
}

static inline void cpu_irq_disable() noexcept
{
	__disable_irq();
	__DMB();
}

typedef bool irqflags_t;

static inline bool cpu_irq_is_enabled() noexcept
{
	return __get_PRIMASK() == 0;
}

static inline irqflags_t cpu_irq_save() noexcept
{
	const irqflags_t flags = cpu_irq_is_enabled();
	cpu_irq_disable();
	return flags;
}

static inline bool cpu_irq_is_enabled_flags(irqflags_t flags) noexcept
{
	return flags;
}

static inline void cpu_irq_restore(irqflags_t flags) noexcept
{
	if (cpu_irq_is_enabled_flags(flags))
	{
		cpu_irq_enable();
	}
}

static inline bool isDigit(char c) noexcept
{
	return isdigit(c) != 0;
}

// Return true if we are in any interrupt service routine
static inline bool inInterrupt() noexcept
{
	return (__get_IPSR() & 0x01FF) != 0;
}

#ifdef __cplusplus
}		// end extern "C"
#endif

#endif /* SRC_CORE_H_ */
