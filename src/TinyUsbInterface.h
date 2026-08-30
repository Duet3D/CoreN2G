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

#ifndef TINYUSBINTERFACE_H_INCLUDED
#define TINYUSBINTERFACE_H_INCLUDED

#include <Core.h>
#include <General/StringRef.h>

#if SUPPORT_USB && CORE_USES_TINYUSB
#include "tusb_option.h"

void CoreUsbInit(NvicPriority priority, Pin usbVbusDetect = NoPin, Pin usbVbusOn = NoPin, Pin usbModeSwitch = NoPin, Pin usbModeDetect = NoPin) noexcept;	// call this to initialise the hardware
extern "C" [[noreturn]] void CoreUsbDeviceTask(void* param) noexcept;		// this must be called by the USB task
void CoreUsbStop();

#if CFG_TUH_ENABLED
bool CoreUsbSetHostMode(bool hostMode, const StringRef& reply);
bool CoreUsbIsHostMode();
#endif

#else
#define CFG_TUH_ENABLED 0
#endif


#endif	// TINYUSBINTERFACE_H_INCLUDED
