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
	explicit SerialCDC(size_t interface_index = 0) noexcept;

	void Start(Pin p_vBusPin) noexcept;
	void end() noexcept;

	int available() noexcept override;
	int read() noexcept override;
	size_t readBytes(char * _ecv_array buffer, size_t length) noexcept override;
	void flush() noexcept override;
	size_t write(uint8_t) noexcept override;
	size_t write(const uint8_t *_ecv_array buffer, size_t size) noexcept override;

	size_t canWrite() noexcept override;	// Function added by DC42 so that we can tell how many characters we can write without blocking (for Duet)
	bool IsConnected() const noexcept;
	void WaitForTxEmpty(uint32_t timeoutMs) noexcept;	// Wait until the CDC TX FIFO is fully drained
	void ClearTxBuffer() noexcept;		// Discard any pending data in the CDC TX FIFO

#ifdef RTOS
	// Zero-copy direct transfers (bypass CDC FIFO, use USB hardware directly)
	void BeginDirectMode() noexcept;	// Take over endpoints from CDC stream (waits for handover)
	void EndDirectMode() noexcept;		// Release endpoints back to CDC stream
	size_t readDirect(uint8_t *buffer, size_t maxLength, uint32_t timeoutMs) noexcept;
	bool writeDirect(const uint8_t *buffer, size_t length, uint32_t timeoutMs) noexcept;

	// Called from tud_cdc_xfer_cb hook to notify completion of a direct transfer
	void DirectXferComplete(uint32_t xferred_bytes) noexcept;

	volatile bool directMode = false;	// checked by tud_cdc_xfer_cb hook
	volatile uint8_t directWaitingEp = 0;	// endpoint we're waiting for completion on (checked by hook)
	volatile uint8_t lastDirectError = 0;	// 0=ok, 1=claim failed, 2=xfer failed, 3=timeout
#endif

private:
	bool running = false;
	Pin vBusPin = NoPin;
	size_t interfaceIndex;

#ifdef RTOS
	volatile TaskHandle directWaitingTask = nullptr;
	volatile uint32_t directXferredBytes = 0;
	uint8_t epIn = 0;		// bulk IN endpoint address
	uint8_t epOut = 0;		// bulk OUT endpoint address
	uint16_t maxPacketSize = 64;	// USB max packet size for bulk endpoints (64 FS, 512 HS)
#endif
};

#endif

#endif /* SRC_SERIALCDC__TUSB_H_ */
