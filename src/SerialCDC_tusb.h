/*
 * SerialCDC.h
 *
 *  Created on: 18 Mar 2016
 *      Author: David
 */

#ifndef SRC_SERIALCDC__TUSB_H_
#define SRC_SERIALCDC__TUSB_H_

#include <Core.h>

#if SUPPORT_USB && CORE_USES_TINYUSB

#include "Stream.h"
#include <General/RingBuffer.h>
#include <RTOSIface/RTOSIface.h>

// Serial over CDC

class SerialCDC : public Stream
{
public:
	SerialCDC() noexcept;

	void Start(Pin p_vBusPin) noexcept;
	void end(void) noexcept;

	int available() noexcept override;
	int read() noexcept override;
	size_t readBytes(char * _ecv_array buffer, size_t length) noexcept override;
	void flush() noexcept override;
	size_t write(uint8_t) noexcept override;
	size_t write(const uint8_t *_ecv_array buffer, size_t size) noexcept override;

	size_t canWrite() noexcept override;	// Function added by DC42 so that we can tell how many characters we can write without blocking (for Duet)
	bool IsConnected() const noexcept;

private:
	volatile TaskHandle txWaitingTask;
    bool running = false;
	Pin vBusPin;
};

#endif

#endif /* SRC_SERIALCDC__TUSB_H_ */
