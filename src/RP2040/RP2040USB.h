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

#ifndef RP2040USB_H_INCLUDED
#define RP2040USB_H_INCLUDED

#define PICO_MUTEX_ENABLE_SDK120_COMPATIBILITY		0
#undef from
#include <pico/mutex.h>

// Big, global USB mutex, shared with all USB devices to make sure we don't
// have multiple cores updating the TUSB state in parallel
extern mutex_t __usb_mutex;

// Called by main() to init the USB subsystem
void __USBStart();

[[noreturn]] void UsbDeviceTask(void* param) noexcept;						// this must be called by the USB task

#endif	// RP2040USB_H_INCLUDED
