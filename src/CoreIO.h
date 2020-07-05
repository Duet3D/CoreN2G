/*
 * Core.h
 *
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
constexpr unsigned int NumTotalPins = (3 * 32) + 22;	// SAME54P20A goes up to PD21
#elif SAMC21
constexpr unsigned int NumTotalPins = 2 * 32;			// SAMC21J goes up to PB31. We don't support the SAMC21N.
#else
# error Unsupported processor
#endif

inline constexpr Pin PortAPin(unsigned int n) noexcept { return n; }
inline constexpr Pin PortBPin(unsigned int n) noexcept { return 32+n; }

#if SAME5x
inline constexpr Pin PortCPin(unsigned int n) noexcept { return 64+n; }
inline constexpr Pin PortDPin(unsigned int n) noexcept { return 96+n; }
#endif

// Pin function numbers for calls to gpio_set_pin_function
enum class GpioPinFunction : uint32_t { A = 0, B, C, D, E, F, G, H, I, J, K, L, M, N };

inline void SetPinFunction(Pin p, GpioPinFunction f) noexcept
{
	gpio_set_pin_function(p, (uint32_t)f);
}

inline void ClearPinFunction(Pin p) noexcept
{
	gpio_set_pin_function(p, GPIO_PIN_FUNCTION_OFF);
}

// GCLK management
enum class GclkSource : uint8_t
{
#if SAME5x
	xosc0 = 1,
	xosc1,
	gclkIn,
	gclk1,
	oscUlp32k,
	xosc32k,
	dfll,
	dpll0,
	dpll1
#elif SAMC21
	xosc = 0,
	gclkIn,
	gclk1,
	oscUlp32k,
	osc32k,
	xosc32k,
	osc48m,
	dpll
#else
# error Unsupported processor
#endif
};

void EnableGclk(unsigned int index, GclkSource source, uint16_t divisor, bool enableOutput = false) noexcept;

// Atomic section locker, alternative to InterruptCriticalSectionLocker (is safe to call from within an ISR, and may be faster)
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

union CallbackParameter
{
	void *vp;
	uint32_t u32;
	int32_t i32;

	CallbackParameter(void *pp) noexcept : vp(pp) { }
	CallbackParameter(uint32_t pp) noexcept : u32(pp) { }
	CallbackParameter(int32_t pp) noexcept : i32(pp) { }
	CallbackParameter() noexcept : u32(0) { }
};

typedef void (*StandardCallbackFunction)(CallbackParameter) noexcept;

extern "C" uint32_t SystemCoreClock;			// in system_samxxx.c
extern "C" uint32_t SystemPeripheralClock;		// in system_samxxx.c

void WatchdogInit() noexcept;
void watchdogReset() noexcept;
void CoreSysTick() noexcept;
void CoreInit(DmaChannel firstAdcDmaChannel) noexcept;

int32_t random(uint32_t howbig) noexcept;

static inline uint32_t random(uint32_t howsmall, uint32_t howbig) noexcept
{
	return random(howbig - howsmall) + howsmall;
}

// Set a pin high with no error checking
inline void fastDigitalWriteHigh(uint32_t pin) noexcept
{
	hri_port_set_OUT_reg(PORT, GPIO_PORT(pin), 1U << GPIO_PIN(pin));
}

// Set a pin low with no error checking
inline void fastDigitalWriteLow(uint32_t pin) noexcept
{
	hri_port_clear_OUT_reg(PORT, GPIO_PORT(pin), 1U << GPIO_PIN(pin));
}

[[noreturn]] void Reset() noexcept;

// Timer identifiers used in assigning PWM control devices
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

static inline constexpr unsigned int GetDeviceNumber(TcOutput tc) noexcept { return (uint8_t)tc >> 1; }
static inline constexpr unsigned int GetOutputNumber(TcOutput tc) noexcept { return (uint8_t)tc & 1; }

// Initialise a TC clock
void EnableTcClock(unsigned int tcNumber, uint32_t gclkVal) noexcept;
void EnableTccClock(unsigned int tccNumber, uint32_t gclkVal) noexcept;

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

static inline constexpr unsigned int GetDeviceNumber(TccOutput tcc) noexcept { return ((uint8_t)tcc & 0x7F) >> 3; }
static inline constexpr unsigned int GetOutputNumber(TccOutput tcc) noexcept { return (uint8_t)tcc & 7; }

static inline constexpr GpioPinFunction GetPeriNumber(TccOutput tcc) noexcept
{
	return ((uint8_t)tcc >= 0x80) ? GpioPinFunction::G : GpioPinFunction::F;		// peripheral G or F
}

// ADC input identifiers. On the SAMC21 we only support the first ADC and the SDADC. On the SAME5x we support both ADCs.
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

typedef AdcInput AnalogChannelNumber;			// for backwards compatibility
constexpr AnalogChannelNumber NO_ADC = AdcInput::none;

static inline constexpr unsigned int GetDeviceNumber(AdcInput ain) noexcept { return (uint8_t)ain >> 4; }
static inline constexpr unsigned int GetInputNumber(AdcInput ain) noexcept { return (uint8_t)ain & 0x0F; }

AnalogChannelNumber PinToAdcChannel(Pin p) noexcept;

// SERCOM identifiers
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

static inline constexpr unsigned int GetDeviceNumber(SercomIo sercom) noexcept { return (uint8_t)sercom & 7; }
static inline constexpr unsigned int GetPeriNumber(SercomIo sercom) noexcept { return ((uint8_t)sercom >= 0x80) ? 3 : 2; }	// peripheral D or C

// Addresses of unique ID dwords
#if SAME5x
constexpr uint32_t SerialNumberAddresses[4] = { 0x008061FC, 0x00806010, 0x00806014, 0x00806018 };
#elif SAMC21
constexpr uint32_t SerialNumberAddresses[4] = { 0x0080A00C, 0x0080A040, 0x0080A044, 0x0080A048 };
#endif

// Pin table format. A client of this library may inherit it in order to define additional fields at the end.
//TODO check that we can still brace-initialise such an inherited strict
struct PinDescriptionBase
{
	TcOutput tc;
	TccOutput tcc;
	AdcInput adc;
#if SAMC21
	AdcInput sdadc;
#endif
	SercomIo sercomIn;
	SercomIo sercomOut;
	uint8_t exintNumber;
};

// External function to get a pin table entry. This must be provided by the client project.
const PinDescriptionBase *GetPinDescription(Pin p) noexcept;

#endif /* SRC_HARDWARE_SAME5X_COREIO_H_ */
