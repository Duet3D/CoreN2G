/*
 * SerialCDC.cpp
 *
 *  Created on: 18 Mar 2016
 *      Author: David
 */

#include <Core.h>

#if SUPPORT_USB

#include "TinyUsbInterface.h"
#include "SerialCDC_tusb.h"

#if CORE_USES_TINYUSB

#if RP2040
# define PICO_MUTEX_ENABLE_SDK120_COMPATIBILITY		0
#endif

extern "C" {
#include "tusb.h"
}

SerialCDC::SerialCDC() noexcept
{
}

void SerialCDC::Start(Pin p) noexcept
{
#if CFG_TUH_ENABLED
	if (CoreUsbIsHostMode())
	{
		return;
	}
#endif
	vBusPin = p;
	while (!tud_inited()) { delay(10); }
	running = true;
}

void SerialCDC::end() noexcept
{
#if CFG_TUH_ENABLED
	if (CoreUsbIsHostMode())
	{
		return;
	}
#endif
	running = false;
}

bool SerialCDC::IsConnected() const noexcept
{
	return
#if CFG_TUH_ENABLED
	!CoreUsbIsHostMode() &&
#endif
	tud_cdc_connected();
}

// Overridden virtual functions

// Non-blocking read, return -1 if no character available
// Note, we must either check tud_cdc_connected() in both of available() and read(), or in neither.
// The original code only checked it in read() which meant that if a character is received when DTR is low,
// available() returned nonzero bit read() never read it. Now we check neither when reading.
int SerialCDC::read() noexcept
{
	if (!running
#if CFG_TUH_ENABLED
	|| CoreUsbIsHostMode()
#endif
	)
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
	if (!running
#if CFG_TUH_ENABLED
	|| CoreUsbIsHostMode()
#endif
	)
	{
		return 0;
	}

	return tud_cdc_available();
}

size_t SerialCDC::readBytes(char * _ecv_array buffer, size_t length) noexcept
{
	if (!running
#if CFG_TUH_ENABLED
	|| CoreUsbIsHostMode()
#endif
	)
	{
		return 0;
	}

	return tud_cdc_read (buffer, length);
}

void SerialCDC::flush() noexcept
{
	if (!running
#if CFG_TUH_ENABLED
	|| CoreUsbIsHostMode()
#endif
	)
	{
		return;
	}

	tud_cdc_write_flush();
}

size_t SerialCDC::canWrite() noexcept
{
	if (!running
#if CFG_TUH_ENABLED
	|| CoreUsbIsHostMode()
#endif
	)
	{
		return 0;
	}

	return tud_cdc_write_available();
}

// Write single character, blocking
size_t SerialCDC::write(uint8_t c) noexcept
{
#if CFG_TUH_ENABLED
	if (CoreUsbIsHostMode())
	{
		return 0;
	}
#endif
	return write(&c, 1);
}

// Blocking write block
size_t SerialCDC::write(const uint8_t *buf, size_t length) noexcept
{
	if (!running
#if CFG_TUH_ENABLED
	|| CoreUsbIsHostMode()
#endif
	)
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
				const unsigned int n2 = tud_cdc_write(buf + i, n);
				tud_cdc_write_flush();
				i += n2;
				written += n2;
				last_avail_time = millis();
			}
			else
			{
				tud_cdc_write_flush();
				if (!tud_cdc_connected() || (!tud_cdc_write_available() && millis() > last_avail_time + 1000 /* 1 second */))
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

// USB Device callbacks
// Invoked when cdc when line state changed e.g connected/disconnected
extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void) itf;
	(void) rts;
	(void) dtr;
}

// Invoked when CDC interface received data from host
extern "C" void tud_cdc_rx_cb(uint8_t itf)
{
	(void) itf;
}

#endif

#endif

// End
