/**
 * @file Core.h
 * @brief Header file declaring types, functions etc. usable from both C and C++
 *
 * This file contains basic CPU and I/O pin support for the SAME5x (also works with SAMD5x) and SAMC21
 * Use it where we can't include the full CoreIO.h file, for example in C source files
 */

/*  Created on: 19 Jun 2020
 *      Author: David
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

typedef uint8_t DmaChannel;			///< A type that represents a DMA channel number
typedef uint8_t DmaPriority;		///< A type that represents a DMA priority
typedef uint8_t Pin;				///< A type that represents an I/O pin on the microcontroller
typedef uint8_t ExintNumber;		///< A type that represents an EXINT number. Used in pin tables.
typedef uint16_t PwmFrequency;		///< A type that represents a PWM frequency. 0 sometimes means "default".
typedef uint32_t NvicPriority;		///< A type that represents an interrupt priority

static const Pin NoPin = 0xFF;		///< A number that represents no I/O pin

// Standard GCLK numbers and frequencies
#if SAME5x

static const uint32_t SystemCoreClockFreq = 120000000;		///< The processor clock frequency after initialisation

static const unsigned int GclkNum120MHz = 0;
static const unsigned int GclkNum32KHz = 1;		// frequency is approx. 32768Hz
static const unsigned int GclkNum60MHz = 3;
static const unsigned int GclkNum48MHz = 4;
// There may be other GCLKs defined by the client application, e.g. on Duet 3 Mini:
//static const unsigned int GclkNum25MHz = 2;	// for Ethernet PHY
//static const unsigned int GclkNum90MHz = 5;	// for SDHC

#elif SAMC21

static const uint32_t SystemCoreClockFreq = 48000000;

static const unsigned int GclkNum48MHz = 0;
static const unsigned int GclkNum32KHz = 3;		// frequency is approx. 32768Hz

#endif

/// Pin mode enumeration
enum PinMode
{
	PIN_MODE_NOT_CONFIGURED = -1,	///< do not use
	INPUT = 0,						///< pin is a digital input
	INPUT_PULLUP,					///< pin is a digital input with pullup enabled
	INPUT_PULLDOWN,					///< pin is a digital input with pulldown enabled
	OUTPUT_LOW,						///< pin is an output with initial state LOW
	OUTPUT_HIGH,					///< pin is an output with initial state HIGH
	AIN,							///< pin is an analog input, digital input buffer is disabled if possible
	SPECIAL,						///< do not use
	OUTPUT_PWM_LOW,					///< PWM output mode, initially low
	OUTPUT_PWM_HIGH,				///< PWM output mode, initially high
};

#ifndef __cplusplus
# include <stdbool.h>
#endif

/// Macro to indicate that a function parameter is unused
#define UNUSED(_x)	(void)_x

/// Assert macro, normally defined to do nothing
#define Assert(expr) ((void) 0)

#ifndef ARRAY_SIZE
/// Macro to give us the number of elements in an array
# define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof((_x)[0]))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the time since startup
 * @return Number of milliseconds since startup modulo 2^32
 */
uint32_t millis() noexcept;

/**
 * @brief Get the time since startup
 * @return Number of milliseconds since startup modulo 2^64
 */
uint64_t millis64() noexcept;

/**
 * @brief Delay for a number of 1ms clock ticks.
 * @param millis Number of 1ms system ticks to delay for. The actual delay may be up to 1 millisecond less than this, or arbitrarily greater.
 */
void delay(uint32_t millis) noexcept;

/**
 * @brief Set the required mode fore an I/O pin for
 * @param pin The pin number to set the mode for. If the pin number is not valid (e.g. NoPin), the call will be ignored.
 * @param mode The mode to set the pin to.
 */
void pinMode(Pin pin, enum PinMode mode) noexcept;

/**
 * @brief Read from an I/O pin
 * @param pin The pin to read from
 * @return true if the pin was valid and its state is high, false if it was not valid (e.g. NoPin) or it is valid and its state is low
 */
bool digitalRead(Pin pin) noexcept;

/**
 * @brief Write to an I/O pin
 * @param pin The pin to write to. If the pin number is not valid (e.g. NoPin), the call will be ignored.
 * @param high True to set the pin high, false to set it low
 */
void digitalWrite(Pin pin, bool high) noexcept;

/**
 * @brief Return a random or pseudo-random unsigned 32-bit integer
 * @return The random number
 */
uint32_t random32(void) noexcept;		// needed by lwip

/**
 * @brief Delay for a specified number of CPU clock cycles from the starting time
 *
 * @param start The time we want to start waiting from, which must be in the recent past
 * @param cycles The number of processor clock cycles to delay from the start time
 * @return The time at which we actually stopped waiting.
 */
uint32_t DelayCycles(uint32_t start, uint32_t cycles) noexcept;

/**
 * @brief Delay or at least the specified number of microseconds
 *
 * @param How many microseconds to delay for
 */
static inline void delayMicroseconds(uint32_t) noexcept __attribute__((always_inline, unused));
static inline void delayMicroseconds(uint32_t usec) noexcept
{
	(void)DelayCycles(SysTick->VAL & 0x00FFFFFF, usec * (SystemCoreClockFreq/1000000));
}

// Functions and macros to enable/disable interrupts

/**
 * @brief Enable interrupts unconditionally
 *
 */
static inline void cpu_irq_enable() noexcept
{
	__DMB();
	__enable_irq();
}

/**
 * @brief Disable interrupts unconditionally
 *
 */
static inline void cpu_irq_disable() noexcept
{
	__disable_irq();
	__DMB();
}

typedef bool irqflags_t;	///< Type used to indicate whether interrupts were enabled/should be enabled

/**
 * @brief Test whether interrupts are enabled
 *
 * @return True iff interrupts are enabled
 */
static inline bool cpu_irq_is_enabled() noexcept
{
	return __get_PRIMASK() == 0;
}

/**
 * @brief Disable interrupts and return the initial enabled state. C++ clients should use class AtomicCriticalSectionLocker instead.
 *
 * @return The initial interrupts enabled state
 */
static inline irqflags_t cpu_irq_save() noexcept
{
	const irqflags_t flags = cpu_irq_is_enabled();
	cpu_irq_disable();
	return flags;
}

/**
 * @brief convert irqflags_t to Boolean
 *
 * @param flags Value to convert
 * @return Converted value
 */
static inline bool cpu_irq_is_enabled_flags(irqflags_t flags) noexcept
{
	return flags;
}

/**
 * @brief Re-enable interrupts if they were enabled originally
 *
 * @param flags The original interrupts enabled state returned by a call to cpu_irq_save
 */
static inline void cpu_irq_restore(irqflags_t flags) noexcept
{
	if (cpu_irq_is_enabled_flags(flags))
	{
		cpu_irq_enable();
	}
}

/**
 * @brief Return true of a character is one of the digits 0 thru 9
 *
 * @param c the character to test
 * @return True iff the character is a digit
 */
static inline bool isDigit(char c) noexcept
{
	return isdigit(c) != 0;
}

/**
 * @brief Test whether we are in an interrupt service routine or exception handler
 *
 * @return true if we are in an interrupt service routine or exception handler
 */
static inline bool inInterrupt() noexcept
{
	return (__get_IPSR() & 0x01FF) != 0;
}

#ifdef __cplusplus
}		// end extern "C"
#endif

#endif /* SRC_CORE_H_ */
