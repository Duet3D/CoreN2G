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
	void end() noexcept;					// Tear down this CDC interface; detaches the whole device once the last interface ends

	int available() noexcept override;
	int read() noexcept override;
	size_t readBytes(char * _ecv_array buffer, size_t length) noexcept override;
	void flush() noexcept override;
	size_t write(uint8_t) noexcept override;
	size_t write(const uint8_t *_ecv_array buffer, size_t size) noexcept override;

	size_t canWrite() noexcept override;	// Function added by DC42 so that we can tell how many characters we can write without blocking (for Duet)
	bool IsConnected() const noexcept;
	uint32_t GetDisconnectCount() const noexcept { return disconnectCount; }	// Changes each time the host drops DTR or the device is configured again
	void NoteDisconnect() noexcept { disconnectCount = disconnectCount + 1; }	// Called from the TinyUSB callbacks
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

	// Non-blocking zero-copy receive, for a task that must not wait. Data received before the endpoint was taken stays in the CDC FIFO
	bool TakeReceiveEndpoint() noexcept;	// Take the OUT endpoint from the CDC stream, possible once a full FIFO has stopped the stream
	bool IsReceiveEndpointTaken() const noexcept { return rxTaken; }
	bool StartDirectRead(uint8_t *buffer, size_t length) noexcept;	// Start receiving, length must be a multiple of the packet size
	bool IsDirectReadPending() const noexcept { return rxPending; }
	bool GetDirectReadProgress(size_t& bytesReceived) const noexcept;	// Get how much the pending read has received so far, false if that cannot be told
	bool IsDirectReadComplete(size_t& bytesRead) noexcept;	// Return true once when a read has ended, on a short packet or with the buffer full
	void ReturnReceiveEndpoint() noexcept;	// Give the endpoint back to the CDC stream, only when no read is pending
	size_t GetMaxPacketSize() const noexcept;	// Bulk packet size at the current bus speed

	void DeviceConfigured() noexcept;	// Called from tud_mount_cb
	bool DirectReadXferComplete(uint8_t ep_addr, uint32_t xferred_bytes) noexcept;	// Called from the tud_cdc_xfer_cb hook

	volatile bool directMode = false;	// checked by tud_cdc_xfer_cb hook
	volatile uint8_t directWaitingEp = 0;	// endpoint we're waiting for completion on (checked by hook)
	volatile uint8_t lastDirectError = 0;	// 0=ok, 1=claim failed, 2=xfer failed, 3=timeout
#endif

private:
	volatile uint32_t disconnectCount = 0;
	bool running = false;
	Pin vBusPin = NoPin;
	size_t interfaceIndex;
	uint32_t lastAvailTime = 0;				// when the blocking write last found space in the transmit buffer

#ifdef RTOS
	volatile TaskHandle directWaitingTask = nullptr;
	volatile uint32_t directXferredBytes = 0;
	volatile uint32_t rxBytes = 0;
	const uint8_t *rxBuffer = nullptr;
	volatile bool rxTaken = false;	// the OUT endpoint is ours, the tud_cdc_xfer_cb hook keeps its completions away from the CDC stream
	volatile bool rxPending = false;
	volatile bool rxDone = false;
	uint8_t epIn = 0;		// bulk IN endpoint address
	uint8_t epOut = 0;		// bulk OUT endpoint address
	uint16_t maxPacketSize = 64;	// USB max packet size for bulk endpoints (64 FS, 512 HS)
#endif
};

#endif

#endif /* SRC_SERIALCDC__TUSB_H_ */
