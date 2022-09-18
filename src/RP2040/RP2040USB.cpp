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

#include <cstdlib>
#include "RP2040USB.h"

#include "tusb.h"
#include "class/hid/hid_device.h"
#include "class/audio/audio.h"
#include "class/midi/midi.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "pico/mutex.h"
#include "pico/unique_id.h"

#define USBD_MAX_POWER_MA (250)

// Big, global USB mutex, shared with all USB devices to make sure we don't
// have multiple cores updating the TUSB state in parallel
mutex_t __usb_mutex;

// USB VID/PID (note that PID can change depending on the add'l interfaces)
#define USBD_VID (0x2E8A) // Raspberry Pi

#ifdef SERIALUSB_PID
#define USBD_PID (SERIALUSB_PID)
#else
#define USBD_PID (0x000a) // Raspberry Pi Pico SDK CDC
#endif

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

extern "C" const uint8_t *tud_descriptor_device_cb(void) noexcept
{
    static tusb_desc_device_t usbd_desc_device =
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

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_hid_descriptor_report_cb(uint8_t instance) noexcept
{
    (void)instance;
    return nullptr;
}

static uint8_t *usbd_desc_cfg = nullptr;

extern "C" const uint8_t *tud_descriptor_configuration_cb(uint8_t index) noexcept
{
    (void)index;
    return usbd_desc_cfg;
}

void __SetupUSBDescriptor() noexcept
{
	if (!usbd_desc_cfg)
	{
		uint8_t interface_count = 2;		// just serial

		uint8_t cdc_desc[TUD_CDC_DESC_LEN] =
		{
			// Interface number, string index, protocol, report descriptor len, EP In & Out address, size & polling interval
			TUD_CDC_DESCRIPTOR(USBD_ITF_CDC, USBD_STR_CDC, USBD_CDC_EP_CMD, USBD_CDC_CMD_MAX_SIZE, USBD_CDC_EP_OUT, USBD_CDC_EP_IN, USBD_CDC_IN_OUT_MAX_SIZE)
		};

		int usbd_desc_len = TUD_CONFIG_DESC_LEN + sizeof(cdc_desc);

		uint8_t tud_cfg_desc[TUD_CONFIG_DESC_LEN] =
		{
			// Config number, interface count, string index, total length, attribute, power in mA
			TUD_CONFIG_DESCRIPTOR(1, interface_count, USBD_STR_0, usbd_desc_len, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, USBD_MAX_POWER_MA)
		};

		// Combine to one descriptor
		usbd_desc_cfg = (uint8_t *)malloc(usbd_desc_len);
		if (usbd_desc_cfg)
		{
			bzero(usbd_desc_cfg, usbd_desc_len);
			uint8_t *ptr = usbd_desc_cfg;
			memcpy(ptr, tud_cfg_desc, sizeof(tud_cfg_desc));
			ptr += sizeof(tud_cfg_desc);
			memcpy(ptr, cdc_desc, sizeof(cdc_desc));
			ptr += sizeof(cdc_desc);
		}
	}
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) noexcept
{
	(void) langid;
#define DESC_STR_MAX (20)
	static uint16_t desc_str[DESC_STR_MAX];

	static char idString[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];

	static const char *const usbd_desc_str[] =
	{
		[USBD_STR_0] = "",
		[USBD_STR_MANUF] = "Raspberry Pi",
		[USBD_STR_PRODUCT] = "PicoArduino",
		[USBD_STR_SERIAL] = idString,
		[USBD_STR_CDC] = "Board CDC",
	};

	if (!idString[0])
	{
		pico_get_unique_board_id_string(idString, sizeof(idString));
	}

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
			return NULL;
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

void __USBStart()
{
	if (tusb_inited())
	{
		// Already called
		return;
	}

	__SetupUSBDescriptor();

	mutex_init(&__usb_mutex);
}

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
void UsbDeviceTask(void* param) noexcept
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

// End
