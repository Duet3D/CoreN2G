/*
 * UsbSerial.cpp
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

#if SUPPORT_USB

#include "SerialCDC.h"

#ifdef RTOS
# include <RTOSIface/RTOSIface.h>
#endif

#undef from

#define PICO_MUTEX_ENABLE_SDK120_COMPATIBILITY		0

extern "C" {
#include "tusb.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "pico/mutex.h"
#include "hardware/watchdog.h"
#include "pico/unique_id.h"
}

#include "RP2040USB.h"

class CoreMutex
{
public:
	CoreMutex(mutex_t *mutex) noexcept
	{
		uint32_t owner;
		_mutex = mutex;
		_acquired = false;
		if (!mutex_try_enter(_mutex, &owner))
		{
			if (owner == get_core_num())
			{	// Deadlock!
				return;
			}
			mutex_enter_blocking(_mutex);
		}
		_acquired = true;
	}

	~CoreMutex()
	{
		if (_acquired)
		{
			mutex_exit(_mutex);
		}
	}

	operator bool() noexcept
	{
		return _acquired;
	}

private:
    mutex_t *_mutex;
    bool _acquired;
};

SerialCDC::SerialCDC(Pin p, size_t numTxSlots, size_t numRxSlots) noexcept
{
}

void SerialCDC::Start() noexcept
{
	running = true;
}

void SerialCDC::end() noexcept
{
	running = false;
}

bool SerialCDC::IsConnected() const noexcept
{
    CoreMutex m(&__usb_mutex);
    if (!running || !m) {
        return false;
    }

    tud_task();
	return tud_cdc_connected();
}

// Overridden virtual functions

// Non-blocking read, return -1 if no character available
// Note, we must either check tud_cdc_connected() in both of available() and read(), or in neither.
// The original code only checked it in read() which meant that if a character is received when DTR is low,
// available() returned nonzero bit read() never read it. Now we check neither when reading.
int SerialCDC::read() noexcept
{
    CoreMutex m(&__usb_mutex);
    if (!running || !m)
    {
          return -1;
    }

    if (tud_cdc_available())
    {
        return tud_cdc_read_char();
    }
    return -1;
}

int SerialCDC::available() noexcept
{
    CoreMutex m(&__usb_mutex);
    if (!running || !m)
    {
          return 0;
    }

    return tud_cdc_available();
}

void SerialCDC::flush() noexcept
{
	CoreMutex m(&__usb_mutex);
	if (!running || !m)
	{
		return;
	}

	tud_cdc_write_flush();
}

size_t SerialCDC::canWrite() noexcept
{
	CoreMutex m(&__usb_mutex);
	if (!running || !m)
	{
		return 0;
	}

	return tud_cdc_write_available();
}

// Write single character, blocking
size_t SerialCDC::write(uint8_t c) noexcept
{
	return write(&c, 1);
}

// Blocking write block
size_t SerialCDC::write(const uint8_t *buf, size_t length) noexcept
{
	CoreMutex m(&__usb_mutex);
	if (!running || !m)
	{
		  return 0;
	}

	static uint64_t last_avail_time;
	int written = 0;
	if (tud_cdc_connected())
	{
		for (size_t i = 0; i < length;)
		{
			unsigned int n = length - i;
			unsigned int avail = tud_cdc_write_available();
			if (n > avail)
			{
				n = avail;
			}
			if (n != 0)
			{
				unsigned int n2 = tud_cdc_write(buf + i, n);
				tud_task();
				tud_cdc_write_flush();
				i += n2;
				written += n2;
				last_avail_time = time_us_64();
			}
			else
			{
				tud_task();
				tud_cdc_write_flush();
				if (!tud_cdc_connected() || (!tud_cdc_write_available() && time_us_64() > last_avail_time + 1000000 /* 1 second */))
				{
					break;
				}
			}
		}
	}
	else
	{
		// reset our timeout
		last_avail_time = 0;
	}
	return written;
}

void SerialCDC::Spin() noexcept
{
#if DONT_USE_TIMER
    // if the mutex is already owned, then we are in user code
    // in this file which will do a tud_task itself, so we'll just do nothing
    // until the next tick; we won't starve
    if (mutex_try_enter(&__usb_mutex, NULL))
    {
        tud_task();
        mutex_exit(&__usb_mutex);
    }
#endif
}

#endif	// SUPPORT_USB

// End
