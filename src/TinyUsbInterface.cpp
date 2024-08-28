/*
    Shared USB for the Raspberry Pi Pico RP2040
    Allows for multiple endpoints to share the USB controller

    Copyright (c) 2021 Earle F. Philhower, III <earlephilhower@yahoo.com>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <TinyUsbInterface.h>

#if SUPPORT_USB && CORE_USES_TINYUSB

#include <CoreIO.h>

#include "tusb.h"
#include "class/hid/hid_device.h"
#include "class/audio/audio.h"
#include "class/midi/midi.h"

#if SAME70

#include "pmc.h"

# define __nocache		__attribute__((section(".ram_nocache")))

#else

# define __nocache		// nothing

# if SAME5x
#  include <hal_gpio.h>
#  include <hal_usb_device.h>
#  include <peripheral_clk_config.h>
# endif

#endif

#define DUAL_CDC		(0)		// dual CDC does not work yet!

#define USBD_MAX_POWER_MA (250)

// USB VID/PID (note that PID can change depending on the add'l interfaces)
#define USBD_VID		(0x1d50)		// Duet 3D
#define USBD_PID		(0x60ee)		// Duet 3

#define USB_BCD   		(0x0200)

// SAMG & SAME70 don't support a same endpoint number with different direction IN and OUT
//    e.g EP1 OUT & EP1 IN cannot exist together
#define EPNUM_CDC_0_NOTIF   0x81
#define EPNUM_CDC_0_OUT     0x02
#define EPNUM_CDC_0_IN      0x83

#if DUAL_CDC
#define EPNUM_CDC_1_NOTIF   0x84
#define EPNUM_CDC_1_OUT     0x05
#define EPNUM_CDC_1_IN      0x86
#endif

#define USBD_CDC_CMD_MAX_SIZE (8)
#define USBD_CDC_IN_OUT_MAX_SIZE (64)

#define USBD_STR_0 (0x00)
#define USBD_STR_MANUF (0x01)
#define USBD_STR_PRODUCT (0x02)
#define USBD_STR_SERIAL (0x03)
#define USBD_STR_CDC (0x04)

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
static tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USBD_VID,
    .idProduct          = USBD_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = USBD_STR_MANUF,
    .iProduct           = USBD_STR_PRODUCT,
    .iSerialNumber      = USBD_STR_SERIAL,

    .bNumConfigurations = 1
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
extern "C" uint8_t const * tud_descriptor_device_cb(void) noexcept
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum
{
  ITF_NUM_CDC_0 = 0,
  ITF_NUM_CDC_0_DATA,
#if DUAL_CDC
  ITF_NUM_CDC_1,
  ITF_NUM_CDC_1_DATA,
#endif
  ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN)

static uint8_t const desc_fs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00,
#if SAME70
  200
#else
  100
#endif
  ),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),

#if DUAL_CDC
  // 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, EPNUM_CDC_1_NOTIF, 8, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 64),
#endif
};

#if TUD_OPT_HIGH_SPEED
// Per USB specs: high speed capable device must report device_qualifier and other_speed_configuration

static uint8_t const desc_hs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00,
#if SAME70
  200
#else
  100
#endif
  ),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 512),

#if DUAL_CDC
  // 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, EPNUM_CDC_1_NOTIF, 8, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 512),
#endif
};

// device qualifier is mostly similar to device descriptor since we don't change configuration based on speed
static tusb_desc_device_qualifier_t const desc_device_qualifier =
{
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = USB_BCD,

  .bDeviceClass       = TUSB_CLASS_CDC,
  .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,

  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .bNumConfigurations = 0x01,
  .bReserved          = 0x00
};

// Invoked when received GET DEVICE QUALIFIER DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete.
// device_qualifier descriptor describes information about a high-speed capable device that would
// change if the device were operating at the other speed. If not highspeed capable stall this request.
extern "C" uint8_t const* tud_descriptor_device_qualifier_cb(void) noexcept
{
  return (uint8_t const*) &desc_device_qualifier;
}

// Invoked when received GET OTHER SEED CONFIGURATION DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
// Configuration descriptor in the other speed e.g if high speed then this is for full speed and vice versa
extern "C" uint8_t const* tud_descriptor_other_speed_configuration_cb(uint8_t index) noexcept
{
  (void) index; // for multiple configurations

  // if link speed is high return fullspeed config, and vice versa
  return (tud_speed_get() == TUSB_SPEED_HIGH) ?  desc_fs_configuration : desc_hs_configuration;
}

#endif // highspeed

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
extern "C" uint8_t const * tud_descriptor_configuration_cb(uint8_t index) noexcept
{
  (void) index; // for multiple configurations

#if TUD_OPT_HIGH_SPEED
  // Although we are highspeed, host may be fullspeed.
  return (tud_speed_get() == TUSB_SPEED_HIGH) ?  desc_hs_configuration : desc_fs_configuration;
#else
  return desc_fs_configuration;
#endif
}

#define DESC_STR_MAX (20)
static uint16_t desc_str[DESC_STR_MAX];

extern "C" const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) noexcept
{
	(void) langid;

	static const char *const usbd_desc_str[] =
	{
		[USBD_STR_0] = "",
		[USBD_STR_MANUF] = "Duet 3D",
		[USBD_STR_PRODUCT] = "Duet",
		[USBD_STR_SERIAL] = "",
		[USBD_STR_CDC] = "Board CDC",
	};

	uint8_t len;
	if (index == 0)
	{
		desc_str[1] = 0x0409; // supported language is English
		len = 1;
	}
	else
	{
		if (index >= sizeof(usbd_desc_str) / sizeof(usbd_desc_str[0]))
		{
			return nullptr;
		}
		const char *str = usbd_desc_str[index];
		for (len = 0; len < DESC_STR_MAX - 1 && str[len]; ++len)
		{
			desc_str[1 + len] = str[len];
		}
	}

	// first byte is length (including header), second byte is string type
	desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * len + 2);

	return desc_str;
}

// Call this to initialise the hardware
void CoreUsbInit(NvicPriority priority) noexcept
{
#if SAME70

	// Set the USB interrupt priority to a level that is allowed to make FreeRTOS calls
	NVIC_SetPriority(USBHS_IRQn, priority);

	// Start the UPLL clock. The default divider is 40 which is correct for 12MHz crystal.
	pmc_enable_upll_clock();

# if TUD_OPT_HIGH_SPEED
	pmc_switch_udpck_to_upllck(1 - 1);
# else
	pmc_switch_udpck_to_upllck(10 - 1);					// when high speed is disabled, tinyusb uses low power mode, which requires a 48MHz clock
# endif
	pmc_enable_udpck();

	pmc_set_fast_startup_input(PMC_FSMR_USBAL);
	// Enable peripheral clock for USBHS
	pmc_enable_periph_clk(ID_USBHS);

#elif SAME5x

	// Set the USB interrupt priority to a level that is allowed to make FreeRTOS calls
	NVIC_SetPriority(USB_0_IRQn, priority);
	NVIC_SetPriority(USB_1_IRQn, priority);
	NVIC_SetPriority(USB_2_IRQn, priority);
	NVIC_SetPriority(USB_3_IRQn, priority);

	// Set up USB clock
	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNum48MHz) | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_AHBMASK_USB_bit(MCLK);
	hri_mclk_set_APBBMASK_USB_bit(MCLK);

	// Set up USB pins
	// This is the code generated by Atmel Start. I don't know whether it is all necessary.
	gpio_set_pin_direction(PortAPin(24), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortAPin(24), false);
	gpio_set_pin_pull_mode(PortAPin(24), GPIO_PULL_OFF);
	gpio_set_pin_function(PortAPin(24), PINMUX_PA24H_USB_DM);

	gpio_set_pin_direction(PortAPin(25), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortAPin(25), false);
	gpio_set_pin_pull_mode(PortAPin(25), GPIO_PULL_OFF);
	gpio_set_pin_function(PortAPin(25), PINMUX_PA25H_USB_DP);

#elif RP2040

	// Set the USB interrupt priority to a suitable level
	NVIC_SetPriority((IRQn_Type)USBCTRL_IRQ, priority);

#else
# error Unsupported processor
#endif
}

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
extern "C" void CoreUsbDeviceTask(void* param) noexcept
{
	(void)param;

	// This should be called after scheduler/kernel is started.
	// Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
	tusb_init();

	// RTOS forever loop
	while (1)
	{
		// tinyusb device task
		tud_task();
//	    tud_cdc_write_flush();
	}
}

#if RP2040		// RP2040 USB configuration has HID enabled by default

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
extern "C" uint8_t const * tud_hid_descriptor_report_cb(uint8_t instance) noexcept
{
    (void)instance;
    return nullptr;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
extern "C" uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) noexcept
{
	// TODO not implemented
	(void) instance;
	(void) report_id;
	(void) report_type;
	(void) buffer;
	(void) reqlen;

	return 0;
}

// Invoked when received SET_REPORT control request or received data on OUT endpoint ( Report ID = 0, Type = 0 )
extern "C" void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) noexcept
{
	// TODO set LED based on CAPLOCK, NUMLOCK etc...
	(void) instance;
	(void) report_id;
	(void) report_type;
	(void) buffer;
	(void) bufsize;
}

#endif

uint32_t numUsbInterrupts = 0;

// USB interrupt handler

#if SAME70

extern "C" void USBHS_Handler() noexcept
{
	++numUsbInterrupts;
	tud_int_handler(0);
}

#elif SAME5x

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
extern "C" void USB_0_Handler(void)
{
	++numUsbInterrupts;
	tud_int_handler(0);
}

extern "C" void USB_1_Handler(void)
{
	++numUsbInterrupts;
	tud_int_handler(0);
}

extern "C" void USB_2_Handler(void)
{
	++numUsbInterrupts;
	tud_int_handler(0);
}

extern "C" void USB_3_Handler(void)
{
	++numUsbInterrupts;
	tud_int_handler(0);
}

#endif

#endif

// End
