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

#include "tusb.h"
#include "class/hid/hid_device.h"
#include "class/audio/audio.h"
#include "class/midi/midi.h"

#if SAME70

#include "pmc.h"
#include "sysclk.h"
//#include <sam/drivers/usbhs/usbhs_otg.h>
//#include <sam/drivers/usbhs/usbhs_device.h>

# define __nocache		__attribute__((section(".ram_nocache")))

#else

# define __nocache		// nothing

#endif


#define USBD_MAX_POWER_MA (250)

// USB VID/PID (note that PID can change depending on the add'l interfaces)
#define USBD_VID (0x2E8A) // Raspberry Pi

#ifdef SERIALUSB_PID
#define USBD_PID (SERIALUSB_PID)
#else
#define USBD_PID (0x000a) // Raspberry Pi Pico SDK CDC
#endif

#define USB_BCD   0x0200

#define USBD_DESC_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

#define USBD_ITF_CDC (0) // needs 2 interfaces
#define USBD_ITF_MAX (2)

#define USBD_CDC_EP_CMD (0x81)
#define USBD_CDC_EP_OUT (0x02)
#define USBD_CDC_EP_IN (0x82)
#define USBD_CDC_CMD_MAX_SIZE (8)
#define USBD_CDC_IN_OUT_MAX_SIZE (64)

#define USBD_STR_0 (0x00)
#define USBD_STR_MANUF (0x01)
#define USBD_STR_PRODUCT (0x02)
#define USBD_STR_SERIAL (0x03)
#define USBD_STR_CDC (0x04)

#if 1

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
static __nocache tusb_desc_device_t /*const*/ desc_device =
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

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
extern "C" uint8_t const * tud_descriptor_device_cb(void) noexcept
{
  return (uint8_t const *) &desc_device;
}

#else

extern "C" const uint8_t *tud_descriptor_device_cb(void) noexcept
{
    static __nocache tusb_desc_device_t usbd_desc_device =
    {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = TUSB_CLASS_CDC,
        .bDeviceSubClass = MISC_SUBCLASS_COMMON,
        .bDeviceProtocol = MISC_PROTOCOL_IAD,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor = USBD_VID,
        .idProduct = USBD_PID,
        .bcdDevice = 0x0100,
        .iManufacturer = USBD_STR_MANUF,
        .iProduct = USBD_STR_PRODUCT,
        .iSerialNumber = USBD_STR_SERIAL,
        .bNumConfigurations = 1
    };
	// Can use as-is, this is the default USB case
	return (const uint8_t *)&usbd_desc_device;
}

#endif

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum
{
  ITF_NUM_CDC_0 = 0,
  ITF_NUM_CDC_0_DATA,
  ITF_NUM_CDC_1,
  ITF_NUM_CDC_1_DATA,
  ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN)

// SAMG & SAME70 don't support a same endpoint number with different direction IN and OUT
//    e.g EP1 OUT & EP1 IN cannot exist together
#define EPNUM_CDC_0_NOTIF   0x81
#define EPNUM_CDC_0_OUT     0x02
#define EPNUM_CDC_0_IN      0x83

#define EPNUM_CDC_1_NOTIF   0x84
#define EPNUM_CDC_1_OUT     0x05
#define EPNUM_CDC_1_IN      0x86

static __nocache uint8_t /*const*/ desc_fs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),

  // 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, EPNUM_CDC_1_NOTIF, 8, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 64),
};

#if TUD_OPT_HIGH_SPEED
// Per USB specs: high speed capable device must report device_qualifier and other_speed_configuration

static __nocache uint8_t /*const*/ desc_hs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 512),

  // 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, EPNUM_CDC_1_NOTIF, 8, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 512),
};

// device qualifier is mostly similar to device descriptor since we don't change configuration based on speed
static __nocache tusb_desc_device_qualifier_t /*const*/ desc_device_qualifier =
{
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = USB_BCD,

  .bDeviceClass       = TUSB_CLASS_MISC,
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

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
extern "C" uint8_t const * tud_hid_descriptor_report_cb(uint8_t instance) noexcept
{
    (void)instance;
    return nullptr;
}

#define DESC_STR_MAX (20)
static __nocache uint16_t desc_str[DESC_STR_MAX];

extern "C" const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) noexcept
{
	(void) langid;

	static const char *const usbd_desc_str[] =
	{
		[USBD_STR_0] = "",
		[USBD_STR_MANUF] = "Raspberry Pi",
		[USBD_STR_PRODUCT] = "PicoArduino",
		[USBD_STR_SERIAL] = "1234123412341234",
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

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
extern "C" void UsbDeviceTask(void* param) noexcept
{
	(void)param;

#if SAME70

	/* Enable peripheral clock for USBHS */
	pmc_enable_periph_clk(ID_USBHS);
	pmc_enable_upll_clock();

	// From the datasheet:
	// "Before enabling the USB clock in the Power Management Controller, the USBHS must be enabled
	// (by writing a one to the USBHS_CTRL.USBE bit and a zero to the USBHS_CTRL.FRZCLK bit)"
# if 0
	pmc_switch_udpck_to_upllck(1 - 1);
	USBHS->USBHS_DEVCTRL = USBHS_DEVCTRL_DETACH | USBHS_DEVCTRL_SPDCONF_FORCED_FS;
# elif TUD_OPT_HIGH_SPEED
	pmc_switch_udpck_to_upllck(1 - 1);
	USBHS->USBHS_DEVCTRL = USBHS_DEVCTRL_DETACH;
# else
	pmc_switch_udpck_to_upllck(10 - 1);					// when high speed is disabled, tinyusb uses low power mode, which requires a 48MHz clock
	USBHS->USBHS_DEVCTRL = USBHS_DEVCTRL_SPDCONF_LOW_POWER | USBHS_DEVCTRL_DETACH;
# endif
	USBHS->USBHS_CTRL = USBHS_CTRL_UIMOD_DEVICE | USBHS_CTRL_USBE;
	pmc_enable_udpck();

	pmc_set_fast_startup_input(PMC_FSMR_USBAL);

# if 0
	// Enable USB hardware
	otg_enable();

	// Set the USB speed requested by configuration file
#  ifdef USB_DEVICE_LOW_SPEED
	udd_low_speed_enable();
#  else
	udd_low_speed_disable();
#  ifdef USB_DEVICE_HS_SUPPORT
	udd_high_speed_enable();
#else
	udd_high_speed_disable();
#endif
#endif // USB_DEVICE_LOW_SPEED

	otg_unfreeze_clock();
	// Check USB clock
	while (!Is_otg_clock_usable());
# endif

#else
# error Unsupported processor
#endif

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

uint32_t numUsbInterrupts = 0;

// USB interrupt handler
extern "C" void USBHS_Handler() noexcept
{
	++numUsbInterrupts;
	tud_int_handler(0);
}

#endif

// End
