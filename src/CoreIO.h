/**
 * @file CoreIO.h
 * @brief Header file declaring types, functions etc. usable from both C and C++
 *
 * This file contains basic CPU and I/O pin support for the SAME5x (also works with SAMD5x) and SAMC21
 * Use it where we can't include the full CoreIO.h file, for example in C source files
 */

/*
 *  Created on: 28 May 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_COREIO_H_
#define SRC_HARDWARE_SAME5X_COREIO_H_

#include "Core.h"

#include <General/SimpleMath.h>
#include <hal_gpio.h>

// Define NumTotalPins as pin number at and beyond which it is not safe to access the corresponding port registers on this processor family.
// This may be greater than the number of I/O pins actually on the particular device we are running on.
#if SAME5x
#include <hri_port_e54.h>
constexpr unsigned int NumTotalPins = (3 * 32) + 22;	// SAME54P20A goes up to PD21
#elif SAMC21
#include <hri_port_c21.h>
constexpr unsigned int NumTotalPins = 2 * 32;			// SAMC21J goes up to PB31. We don't support the SAMC21N.
#else
# error Unsupported processor
#endif

/**
 * @brief Return the global pin number for a Port A pin
 *
 * @param n The bit number of the pin on Port A
 * @return The global pin number
 */
inline constexpr Pin PortAPin(unsigned int n) noexcept { return n; }

/**
 * @brief Return the global pin number for a Port B pin
 *
 * @param n The bit number of the pin on Port B
 * @return The global pin number
 */
inline constexpr Pin PortBPin(unsigned int n) noexcept { return 32+n; }

#if SAME5x

/**
 * @brief Return the global pin number for a Port C pin
 *
 * @param n The bit number of the pin on Port C
 * @return The global pin number
 */
inline constexpr Pin PortCPin(unsigned int n) noexcept { return 64+n; }

/**
 * @brief Return the global pin number for a Port D pin
 *
 * @param n The bit number of the pin on Port D
 * @return The global pin number
 */
inline constexpr Pin PortDPin(unsigned int n) noexcept { return 96+n; }

#endif

/**
 * @brief Pin function numbers for calls to gpio_set_pin_function
 *
 */
enum class GpioPinFunction : uint32_t { A = 0, B, C, D, E, F, G, H, I, J, K, L, M, N, off = 0xFFFFFFFF };

/**
 * @brief Set the function of an I/O pin
 *
 * @param p The pin number
 * @param f The required pin function
 */
inline void SetPinFunction(Pin p, GpioPinFunction f) noexcept
{
	gpio_set_pin_function(p, (uint32_t)f);
}

/**
 * @brief Set a pin back top ordinary digital I/O
 *
 * @param p The pin number
 */
inline void ClearPinFunction(Pin p) noexcept
{
	gpio_set_pin_function(p, GPIO_PIN_FUNCTION_OFF);
}

// GCLK management

/**
 * @brief Possible sources for a GCLK
 *
 */
enum class GclkSource : uint8_t
{
#if SAME5x
	xosc0 = 0,
	xosc1,
	gclkIn,
	gclk1,
	oscUlp32k,
	xosc32k,
	dfll,
	dpll0,
	dpll1
#elif SAMC21
	xosc = 0, /**< xosc */
	gclkIn,   /**< gclkIn */
	gclk1,    /**< gclk1 */
	oscUlp32k,/**< oscUlp32k */
	osc32k,   /**< osc32k */
	xosc32k,  /**< xosc32k */
	osc48m,   /**< osc48m */
	dpll      /**< dpll */
#else
# error Unsupported processor
#endif
};

/**
 * @brief Configure and enable a GCLK
 *
 * @param index The GCLK number
 * @param source The source oscillator
 * @param divisor The division factor
 * @param enableOutput Whether we want to enable output to a pin
 */
void ConfigureGclk(unsigned int index, GclkSource source, uint16_t divisor, bool enableOutput = false) noexcept;

// Atomic section locker, alternative to InterruptCriticalSectionLocker (is safe to call from within an ISR, and may be faster)
/**
 * @brief This class is an alternative to InterruptCriticalSectionLocker. It is safe to call from within an ISR, and may be faster.
 *
 */
class AtomicCriticalSectionLocker
{
public:
	AtomicCriticalSectionLocker() : flags(cpu_irq_save())
	{
	}

	~AtomicCriticalSectionLocker()
	{
		cpu_irq_restore(flags);
	}

private:
	irqflags_t flags;
};

#if SAME5x

// Functions to change the base priority, to shut out interrupts up to a priority level

// Get the base priority and shut out interrupts lower than or equal to a specified priority
inline uint32_t ChangeBasePriority(uint32_t prio)
{
	const uint32_t oldPrio = __get_BASEPRI();
	__set_BASEPRI_MAX(prio << (8 - __NVIC_PRIO_BITS));
	return oldPrio;
}

// Restore the base priority following a call to ChangeBasePriority
inline void RestoreBasePriority(uint32_t prio)
{
	__set_BASEPRI(prio);
}

// Set the base priority when we are not interested in the existing value i.e. definitely in non-interrupt code
inline void SetBasePriority(uint32_t prio)
{
	__set_BASEPRI(prio << (8 - __NVIC_PRIO_BITS));
}

#endif

/**
 * @brief Type used as the parameter to a standard callback function
 *
 */
union CallbackParameter
{
	void *vp;
	uint32_t u32;
	int32_t i32;

	CallbackParameter(void *pp) noexcept : vp(pp) { }
	CallbackParameter(uint32_t pp) noexcept : u32(pp) { }
	CallbackParameter(unsigned int p) noexcept { u32 = p; }
	CallbackParameter(int32_t pp) noexcept : i32(pp) { }
	CallbackParameter(int p) noexcept { i32 = p; }
	CallbackParameter() noexcept : u32(0) { }
};

/**
 * @brief Standard callback function type
 *
 * @param The parameter to the callback function
 */
typedef void (*StandardCallbackFunction)(CallbackParameter) noexcept;

/**
 * @brief Initialise the watchdog
 *
 */
void WatchdogInit() noexcept;

/**
 * @brief Kick the watchdog. This should be called form within the tick ISR.
 *
 */
void WatchdogReset() noexcept;

/**
 * @brief Timekeeping function. Call this from within the tick ISR.
 *
 */
void CoreSysTick() noexcept;

/**
 * @brief Call this from the application to initialise the DMA controller, interrupt system etc.
 *
 */
void CoreInit() noexcept;

// Random numbers

/**
 * @brief Return a random or pseudo-random number
 *
 * @param howbig Upper limit
 * @return The number, in the range 0 to (howbig - 1)
 */
static inline int32_t random(uint32_t howbig) noexcept
{
	return (howbig == 0) ? 0 : random32() % howbig;
}

/**
 * @brief  Return a random or pseudo-random number
 *
 * @param howsmall Lower limit
 * @param howbig Upper limit
 * @return The number, in the range howsmall to (howbig - 1)
 */
static inline uint32_t random(uint32_t howsmall, uint32_t howbig) noexcept
{
	return random(howbig - howsmall) + howsmall;
}

/**
 * @brief Set a pin high with no error checking
 *
 * @param pin The pin to set high
 */
inline void fastDigitalWriteHigh(uint32_t pin) noexcept
{
	PORT->Group[GPIO_PORT(pin)].OUTSET.reg = 1ul << GPIO_PIN(pin);
}

/**
 * @brief Set a pin low with no error checking
 *
 * @param pin The pin to set low
 */
inline void fastDigitalWriteLow(uint32_t pin) noexcept
{
	PORT->Group[GPIO_PORT(pin)].OUTCLR.reg = 1ul << GPIO_PIN(pin);
}

/**
 * @brief Read a pin with no error checking
 *
 * @param pin The pin to read
 */
inline bool fastDigitalRead(uint32_t pin) noexcept
{
	return (PORT->Group[GPIO_PORT(pin)].IN.reg & (1ul << GPIO_PIN(pin))) != 0;
}

/**
 * @brief Reset the microcontroller
 *
 */
[[noreturn]] void Reset() noexcept;

/**
 * @brief TC output identifiers used in pin tables
 * These encode the TC number, the output number from that TC, and the peripheral number
 */
enum class TcOutput : uint8_t
{
	// TC devices, on peripheral E for both SAME5x and SAMC21
	tc0_0 = 0, tc0_1,
	tc1_0, tc1_1,
	tc2_0, tc2_1,
	tc3_0, tc3_1,
	tc4_0, tc4_1,
#if SAME5x
	tc5_0, tc5_1,
	tc6_0, tc6_1,
	tc7_0, tc7_1,
#endif

	none = 0xFF,
};

/**
 * @brief Extract the TC number
 *
 * @param tc The TcOutput value
 * @return The TC number of tc
 */
static inline constexpr unsigned int GetDeviceNumber(TcOutput tc) noexcept { return (uint8_t)tc >> 1; }

/**
 * @brief Extract the output number
 *
 * @param tc The TcOutput value
 * @return The output number of tc
 */
static inline constexpr unsigned int GetOutputNumber(TcOutput tc) noexcept { return (uint8_t)tc & 1; }

/**
 * @brief Get the peripheral function that a TC output is on
 *
 * @param tc the TC output
 * @return The peripheral function identifier
 */
static inline constexpr GpioPinFunction GetPeriNumber(TcOutput tc) noexcept
{
	return GpioPinFunction::E;		// all TCs are on peripheral E for both the SAME5x and the SAMC21
}

/**
 * @brief Initialise a TC clock
 *
 * @param tcNumber The TC number that needs a clock
 * @param gclkNum The GCLK number to use
 */
void EnableTcClock(unsigned int tcNumber, unsigned int gclkNum) noexcept;

/**
 * @brief TCC output identifiers used in pin tables
 * These encode the TCC number, the output number from that TCC, and the peripheral number that the output is on
 */
enum class TccOutput : uint8_t
{
#if SAME5x
	// TCC devices on peripheral F
	tcc0_0F = 0x00, tcc0_1F, tcc0_2F, tcc0_3F, tcc0_4F, tcc0_5F,
	tcc1_0F = 0x08, tcc1_1F, tcc1_2F, tcc1_3F, tcc1_4F, tcc1_5F, tcc1_6F,
	tcc2_0F = 0x10, tcc2_1F, tcc2_2F, tcc2_3F, tcc2_4F, tcc2_5F,
	tcc3_0F = 0x18, tcc3_1F, tcc3_2F, tcc3_3F, tcc3_4F, tcc3_5F,
	tcc4_0F = 0x20, tcc4_1F, tcc4_2F, tcc4_3F, tcc4_4F, tcc4_5F,
	tcc5_0F = 0x28, tcc5_1F, tcc5_2F, tcc5_3F, tcc5_4F, tcc5_5F,

	// TCC devices on peripheral G
	tcc0_0G = 0x80, tcc0_1G, tcc0_2G, tcc0_3G, tcc0_4G, tcc0_5G, tcc0_6G, tcc0_7G,
	tcc1_0G = 0x88, tcc1_1G, tcc1_2G, tcc1_3G, tcc1_4G, tcc1_5G,
	tcc2_0G = 0x90, tcc2_1G, tcc2_2G, tcc2_3G, tcc2_4G, tcc2_5G,
	tcc3_0G = 0x98, tcc3_1G, tcc3_2G, tcc3_3G, tcc3_4G, tcc3_5G,
	tcc4_0G = 0xA0, tcc4_1G, tcc4_2G, tcc4_3G, tcc4_4G, tcc4_5G,
	tcc5_0G = 0xA8, tcc5_1G, tcc5_2G, tcc5_3G, tcc5_4G, tcc5_5G,
#endif

#if SAMC21
	// TCC devices on peripheral E
	tcc0_0E = 0x00, tcc0_1E, tcc0_2E, tcc0_3E, tcc0_4E, tcc0_5E,
	tcc1_0E = 0x08, tcc1_1E, tcc1_2E, tcc1_3E, tcc1_4E, tcc1_5E,
	tcc2_0E = 0x10, tcc2_1E, tcc2_2E, tcc2_3E, tcc2_4E, tcc2_5E,
	// TCC devices on peripheral F
	tcc0_0F = 0x80, tcc0_1F, tcc0_2F, tcc0_3F, tcc0_4F, tcc0_5F, tcc0_6F, tcc0_7F,
	tcc1_0F = 0x88, tcc1_1F, tcc1_2F, tcc1_3F, tcc1_4F, tcc1_5F,
#endif

	none = 0xFF
};

/**
 * @brief Extract the TCC number
 *
 * @param tcc The TcOutput value
 * @return The TCC number of tcc
 */
static inline constexpr unsigned int GetDeviceNumber(TccOutput tcc) noexcept { return ((uint8_t)tcc & 0x7F) >> 3; }

/**
 * @brief Extract the output number
 *
 * @param tcc The TccOutput value
 * @return The output number of tcc
 */
static inline constexpr unsigned int GetOutputNumber(TccOutput tcc) noexcept { return (uint8_t)tcc & 7; }

/**
 * @brief Get the peripheral function that a TCC output is on
 *
 * @param tcc the TCC output
 * @return The peripheral function identifier
 */
static inline constexpr GpioPinFunction GetPeriNumber(TccOutput tcc) noexcept
{
#if SAME5x
	return ((uint8_t)tcc >= 0x80) ? GpioPinFunction::G : GpioPinFunction::F;		// peripheral G or F
#elif SAMC21
	return ((uint8_t)tcc >= 0x80) ? GpioPinFunction::F : GpioPinFunction::E;		// peripheral F or E
#endif
}

/**
 * @brief Initialise a TCC clock
 *
 * @param tccNumber The TCC number that needs a clock
 * @param gclkNumThe GCLK number to use
 */
void EnableTccClock(unsigned int tccNumber, unsigned int gclkNum) noexcept;

/**
 * @brief ADC input identifiers, encoding both the ADC device and the ADC input number within the device.
 * On the SAMC21 we only support the first ADC and the SDADC. On the SAME5x we support both ADCs.
 *
 */
enum class AdcInput : uint8_t
{
	adc0_0 = 0x00, adc0_1, adc0_2, adc0_3, adc0_4, adc0_5, adc0_6, adc0_7, adc0_8, adc0_9, adc0_10, adc0_11,
#if SAME5x
	adc0_12, adc0_13, adc0_14, adc0_15,
	adc1_0 = 0x10, adc1_1, adc1_2, adc1_3, adc1_4, adc1_5, adc1_6, adc1_7, adc1_8, adc1_9, adc1_10, adc1_11,
#elif SAMC21
	sdadc_0 = 0x10, sdadc_1,
#endif
	none = 0xFF
};

typedef AdcInput AnalogChannelNumber;						///< for backwards compatibility
constexpr AnalogChannelNumber NO_ADC = AdcInput::none;		///< for backwards compatibility

/**
 * @brief Get the ADC number that an ADC input is on
 *
 * @param ain The AdcInput value
 * @return The ADC number
 */
static inline constexpr unsigned int GetDeviceNumber(AdcInput ain) noexcept { return (uint8_t)ain >> 4; }

/**
 * @brief Get the ADC input number that an ADC input is on
 *
 * @param ain The AdcInput
 * @return The input number within the ADC
 */
static inline constexpr unsigned int GetInputNumber(AdcInput ain) noexcept { return (uint8_t)ain & 0x0F; }

/**
 * @brief Return the AdcInput that is attached to a pin
 *
 * @param p The pin number
 * @return The AdcInput, or AdcInput::none
 */
AdcInput PinToAdcChannel(Pin p) noexcept;

/**
 * @brief SERCOM identifier. This encodes a SERCOM number and the peripheral that it is on.
 *
 */
enum class SercomIo : uint8_t
{
	// SERCOM pins on peripheral C
	sercom0c = 0x00,
	sercom1c, sercom2c, sercom3c, sercom4c, sercom5c,
#if SAME5x
	sercom6c, sercom7c,
#endif

	// SERCOM pins on peripheral D
	sercom0d = 0x80,
	sercom1d, sercom2d, sercom3d, sercom4d, sercom5d,
#if SAME5x
	sercom6d, sercom7d,
#endif

	none = 0xFF
};

/**
 * @brief get the SERCOM number
 *
 * @param sercom The SercomIo
 * @return the SERCOM number
 */
static inline constexpr unsigned int GetDeviceNumber(SercomIo sercom) noexcept { return (uint8_t)sercom & 7; }

/**
 * @brief Get the peripheral ID
 *
 * @param sercom The SercomIo
 * @return The peripheral ID
 */
static inline constexpr GpioPinFunction GetPeriNumber(SercomIo sercom) noexcept { return ((uint8_t)sercom >= 0x80) ? GpioPinFunction::D : GpioPinFunction::C; }

// Addresses of unique ID dwords
#if SAME5x
constexpr uint32_t SerialNumberAddresses[4] = { 0x008061FC, 0x00806010, 0x00806014, 0x00806018 };
#elif SAMC21
constexpr uint32_t SerialNumberAddresses[4] = { 0x0080A00C, 0x0080A040, 0x0080A044, 0x0080A048 };
#endif

/**
 * @section AppInterface Functions that must be provided by the application project
 */

/**
 * @brief Layout of an entry in the pin table. The client project may add additional fields by deriving from this.
 */
struct PinDescriptionBase
{
	TcOutput tc;					///< The TC output that is connected to this pin and available for PWM generation, or TcOutput::none
	TccOutput tcc;					///< The TCC output that is connected to this pin and available for PWM generation, or TccOutput::none
	AdcInput adc;					///< The ADC input that is connected to this pin and available, or AdcInput::none
#if SAMC21
	AdcInput sdadc;					///< The SDADC input that is connected to this pin and available, or AdcInput::none
#endif
	SercomIo sercomIn;				///< The Sercom input that is connected to this pin and available, or SercomIo::none
	SercomIo sercomOut;				///< The Sercom output that is connected to this pin and available, or SercomIo::none
	ExintNumber exintNumber;		///< The EXINT number that is allocated exclusively for use by this pin, or Nx if none available
};

/**
 * @brief Initialise the application. Called after the main clocks have been set up.
 * You can use delayMicroseconds() in this function but not delay().
 */
extern void AppInit() noexcept;

/**
 * @brief Run the application. Must not return.
 */
[[noreturn]] extern void AppMain() noexcept;

/**
 * @brief Get the frequency in MHz of the crystal connected to the MCU. Should be 12, 16 or 25.
 * @return Frequency in MHz
 */
extern unsigned int AppGetXoscFrequency() noexcept;

#if SAME5x

/**
 * @brief Get the MCU oscillator number whose pins the crystal is connected to
 * @return XOSC number, 0 or 1
 */
extern unsigned int AppGetXoscNumber() noexcept;

#endif

/**
 * @brief Get a pin table entry
 * @param p Pin number
 * @return Pointer to the pin table entry for that pin, or nullptr if the pin does not exist
 */
extern const PinDescriptionBase *AppGetPinDescription(Pin p) noexcept;

#if SAME5x

/**
 * @brief Return the frequency of the SDHC peripheral clock. Only needs to be provided if the SDHC subsystem is used.
 * @return Frequency in Hz
 */
extern uint32_t AppGetSdhcClockSpeed() noexcept;

#endif

#endif /* SRC_HARDWARE_SAME5X_COREIO_H_ */
