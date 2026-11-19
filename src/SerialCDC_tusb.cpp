/*
 * SerialCDC.cpp
 *
 *  Created on: 18 Mar 2016
 *      Author: David
 */

#include <Core.h>

#if SUPPORT_USB

#include "SerialCDC_tusb.h"

#if CORE_USES_TINYUSB

#if RP2040
# define PICO_MUTEX_ENABLE_SDK120_COMPATIBILITY		0
#endif

extern "C" {
#include "tusb.h"
#ifdef RTOS
#include "device/usbd_pvt.h"
#endif
}

#ifdef RTOS
# include <CoreNotifyIndices.h>
#endif

// Array of SerialCDC instances for lookup by interface index in callbacks
static SerialCDC *serialCDCInstances[CFG_TUD_CDC] = { nullptr };

SerialCDC::SerialCDC(size_t interface_index) noexcept
	: interfaceIndex(interface_index)
{
	if (interface_index < CFG_TUD_CDC)
	{
		serialCDCInstances[interface_index] = this;
	}

#ifdef RTOS
	// Endpoint addresses must match EPNUM_CDC_x defines in TinyUsbInterface.cpp
	if (interface_index == 0)
	{
		epIn = 0x83;		// EPNUM_CDC_0_IN
		epOut = 0x02;		// EPNUM_CDC_0_OUT
	}
	else
	{
		epIn = 0x86;		// EPNUM_CDC_1_IN
		epOut = 0x05;		// EPNUM_CDC_1_OUT
	}
#endif
}

// All CDC interfaces share one TinyUSB device and tud_disconnect() acts on the whole device, so count
// the running interfaces and only detach once the last one ends. That gives the host a clean disconnect
// on reboot without one interface tearing the bus out from under the other. The initial attach is left
// to tusb_init(); calling tud_connect() here as well races the USB task mid-enumeration and breaks it.
static unsigned int numRunningCdcInterfaces = 0;

void SerialCDC::Start(Pin p) noexcept
{
	while (!tud_inited()) { delay(10); }
#ifdef RTOS
	maxPacketSize = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512 : 64;
#endif
	if (!running)
	{
		running = true;
		++numRunningCdcInterfaces;
	}
}

void SerialCDC::end() noexcept
{
	if (running)
	{
		running = false;
		if (--numRunningCdcInterfaces == 0)
		{
			tud_disconnect();				// detach the device now that the last interface has ended
		}
	}
}

bool SerialCDC::IsConnected() const noexcept
{
	return tud_cdc_n_connected(interfaceIndex);
}

// Overridden virtual functions

// Non-blocking read, return -1 if no character available
// Note, we must either check tud_cdc_connected() in both of available() and read(), or in neither.
// The original code only checked it in read() which meant that if a character is received when DTR is low,
// available() returned nonzero bit read() never read it. Now we check neither when reading.
int SerialCDC::read() noexcept
{
    if (!running || directMode)
    {
    	return -1;
    }

    if (tud_cdc_n_available(interfaceIndex))
    {
        return tud_cdc_n_read_char(interfaceIndex);
    }
    return -1;
}

int SerialCDC::available() noexcept
{
    if (!running || directMode)
    {
    	return 0;
    }

    return tud_cdc_n_available(interfaceIndex);
}

size_t SerialCDC::readBytes(char * _ecv_array buffer, size_t length) noexcept
{
	if (!running || directMode) { return 0; }
	return tud_cdc_n_read(interfaceIndex, buffer, length);
}

void SerialCDC::flush() noexcept
{
	if (!running || directMode)
	{
		return;
	}

	tud_cdc_n_write_flush(interfaceIndex);
}

size_t SerialCDC::canWrite() noexcept
{
	if (!running || directMode)
	{
		return 0;
	}

	return tud_cdc_n_write_available(interfaceIndex);
}

// Write single character, blocking
size_t SerialCDC::write(uint8_t c) noexcept
{
	return write(&c, 1);
}

// Blocking write block
size_t SerialCDC::write(const uint8_t *buf, size_t length) noexcept
{
	if (!running || directMode)
	{
		return 0;
	}

	int written = 0;
	if (tud_cdc_n_connected(interfaceIndex))
	{
		for (size_t i = 0; i < length;)
		{
			unsigned int n = length - i;
			unsigned int avail = tud_cdc_n_write_available(interfaceIndex);
			if (n > avail)
			{
				n = avail;
			}
			if (n != 0)
			{
				const unsigned int n2 = tud_cdc_n_write(interfaceIndex, buf + i, n);
				tud_cdc_n_write_flush(interfaceIndex);
				i += n2;
				written += n2;
				lastAvailTime = millis();
			}
			else
			{
				tud_cdc_n_write_flush(interfaceIndex);
				if (!tud_cdc_n_connected(interfaceIndex) || (!tud_cdc_n_write_available(interfaceIndex) && millis() - lastAvailTime > 1000 /* 1 second */))
				{
					break;
				}
			}
		}
	}
	else
	{
		// reset our timeout
		lastAvailTime = 0;
	}
	return written;
}

void SerialCDC::ClearTxBuffer() noexcept
{
	tud_cdc_n_write_clear(interfaceIndex);
}

// Wait until the CDC TX FIFO is fully drained (all data sent to host)
void SerialCDC::WaitForTxEmpty(uint32_t timeoutMs) noexcept
{
	if (!running || directMode) { return; }

	const uint32_t startTime = millis();
	while (millis() - startTime < timeoutMs)
	{
		tud_cdc_n_write_flush(interfaceIndex);
		if (tud_cdc_n_write_available(interfaceIndex) >= CFG_TUD_CDC_TX_BUFSIZE)
		{
			return;		// FIFO is empty
		}
		delay(1);
	}
}

// USB Device callbacks
// Invoked when cdc when line state changed e.g connected/disconnected
extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void) rts;

	if (!dtr && itf < CFG_TUD_CDC)
	{
		SerialCDC *dev = serialCDCInstances[itf];
		if (dev != nullptr)
		{
			dev->NoteDisconnect();

			// In direct mode, wake the blocked task so it can detect the disconnect immediately
			if (dev->directMode && dev->directWaitingEp != 0)
			{
				dev->DirectXferComplete(0);
			}

			// Flush the CDC RX FIFO so stale partial data doesn't interfere with the next connection
			tud_cdc_n_read_flush(itf);
		}
	}
}

// Invoked when CDC interface received data from host
extern "C" void tud_cdc_rx_cb(uint8_t itf)
{
	(void) itf;
}

#ifdef RTOS

// Zero-copy direct write: submit buffer directly to USB DMA, bypassing CDC FIFO.
// Must call BeginDirectMode() first.
bool SerialCDC::writeDirect(const uint8_t *buffer, size_t length, uint32_t timeoutMs) noexcept
{
	if (!running || !directMode)
	{
		return false;
	}

	directWaitingTask = TaskBase::GetCallerTaskHandle();
	directXferredBytes = 0;
	directWaitingEp = epIn;

	if (!usbd_edpt_claim(0, epIn))
	{
		directWaitingEp = 0;
		return false;
	}

	if (!usbd_edpt_xfer(0, epIn, const_cast<uint8_t *>(buffer), (uint16_t)length))
	{
		usbd_edpt_release(0, epIn);
		directWaitingEp = 0;
		return false;
	}

	// Block calling task until completion callback fires
	if (!TaskBase::TakeIndexed(NotifyIndices::UsbDirect, timeoutMs))
	{
		directWaitingEp = 0;
		return false;
	}
	directWaitingEp = 0;

	// If the transfer size is a multiple of the max packet size,
	// the host can't distinguish "transfer complete" from "more data coming".
	// Send a ZLP (zero-length packet) to signal end-of-transfer.
	if (length > 0 && (length % maxPacketSize) == 0)
	{
		directWaitingTask = TaskBase::GetCallerTaskHandle();
		directXferredBytes = 0;
		directWaitingEp = epIn;

		if (usbd_edpt_claim(0, epIn))
		{
			if (usbd_edpt_xfer(0, epIn, nullptr, 0))
			{
				TaskBase::TakeIndexed(NotifyIndices::UsbDirect, timeoutMs);
			}
			else
			{
				usbd_edpt_release(0, epIn);
			}
		}
		directWaitingEp = 0;
	}
	return true;
}

// Enter direct mode: set flag and wait for the CDC stream's current OUT transfer to complete.
// The tud_cdc_xfer_cb hook will see directMode=true and prevent the stream from re-claiming the endpoint.
void SerialCDC::BeginDirectMode() noexcept
{
	directWaitingTask = TaskBase::GetCallerTaskHandle();
	directXferredBytes = 0;
	directWaitingEp = epOut;	// only intercept OUT completions (not TX flush completions)
	directMode = true;

	// The CDC stream currently owns the OUT endpoint. Wait for its pending transfer
	// to complete. Our hook (tud_cdc_xfer_cb) will intercept the completion, notify us,
	// and prevent the stream from re-claiming the endpoint.
	// Note: the first packet from the host is consumed by this handover and not returned
	// to the caller. The protocol must account for this.
	TaskBase::TakeIndexed(NotifyIndices::UsbDirect, 2000);
	directWaitingEp = 0;
}

// Exit direct mode: cancel any pending direct transfers and re-prepare the CDC stream
void SerialCDC::EndDirectMode() noexcept
{
	directMode = false;
	directWaitingEp = 0;

	// Cancel any pending hardware transfers on both endpoints.
	// Only stall if the endpoint is busy (has a pending transfer that needs cancelling).
	// Stalling an idle endpoint is unnecessary and visible to the host.
	if (usbd_edpt_busy(0, epOut))
	{
		usbd_edpt_stall(0, epOut);
		usbd_edpt_clear_stall(0, epOut);
	}
	usbd_edpt_release(0, epOut);
	if (usbd_edpt_busy(0, epIn))
	{
		usbd_edpt_stall(0, epIn);
		usbd_edpt_clear_stall(0, epIn);
	}
	usbd_edpt_release(0, epIn);

	// A direct transfer that completed after its timeout was handed to the CDC stream, so discard what the stream holds
	tud_cdc_n_read_flush(interfaceIndex);

	// Re-prepare the CDC stream's OUT endpoint for FIFO reads.
	// tud_cdc_n_read triggers tu_edpt_stream_read_xfer which re-submits an OUT transfer
	// so the CDC stream can receive data again.
	uint8_t dummy;
	tud_cdc_n_read(interfaceIndex, &dummy, 1);
}

// Zero-copy direct read: submit buffer directly to USB DMA, bypassing CDC FIFO.
// Must call BeginDirectMode() first to take over the endpoint from the CDC stream.
size_t SerialCDC::readDirect(uint8_t *buffer, size_t maxLength, uint32_t timeoutMs) noexcept
{
	if (!running || !directMode)
	{
		return 0;
	}

	directWaitingTask = TaskBase::GetCallerTaskHandle();
	directXferredBytes = 0;
	directWaitingEp = epOut;

	if (!usbd_edpt_claim(0, epOut))
	{
		// Caller can check lastDirectError for diagnostics
		lastDirectError = 1;	// claim failed
		directWaitingEp = 0;
		return 0;
	}

	if (!usbd_edpt_xfer(0, epOut, buffer, (uint16_t)maxLength))
	{
		lastDirectError = 2;	// xfer submit failed
		usbd_edpt_release(0, epOut);
		directWaitingEp = 0;
		return 0;
	}

	// Block calling task until completion callback or timeout
	if (!TaskBase::TakeIndexed(NotifyIndices::UsbDirect, timeoutMs))
	{
		lastDirectError = 3;	// timeout
	}
	else
	{
		lastDirectError = 0;
	}
	directWaitingEp = 0;
	return directXferredBytes;
}

size_t SerialCDC::GetMaxPacketSize() const noexcept
{
	return (tud_speed_get() == TUSB_SPEED_HIGH) ? 512 : 64;
}

// The CDC stream only starts an OUT transfer while its FIFO has room for a whole packet, and nothing but reading the FIFO
// makes room. So when the caller is the only reader and finds the FIFO fuller than that, no transfer is in progress and
// everything received so far is in the FIFO. Claiming the endpoint then keeps the stream from starting another one.
bool SerialCDC::TakeReceiveEndpoint() noexcept
{
	if (!running || directMode || rxTaken || CFG_TUD_CDC_RX_BUFSIZE - tud_cdc_n_available(interfaceIndex) >= GetMaxPacketSize() || !usbd_edpt_claim(0, epOut))
	{
		return false;
	}

	rxPending = false;
	rxDone = false;
	rxTaken = true;
	return true;
}

bool SerialCDC::StartDirectRead(uint8_t *buffer, size_t length) noexcept
{
	if (!rxTaken || rxPending)
	{
		return false;
	}

	rxDone = false;
	rxBuffer = buffer;
	rxPending = true;
	if (!usbd_edpt_xfer(0, epOut, buffer, (uint16_t)length))
	{
		// A failed submit leaves the endpoint unclaimed, so hand it back to the stream and let the caller take it again later
		rxPending = false;
		ReturnReceiveEndpoint();
		return false;
	}
	return true;
}

// TinyUSB does not report the progress of a transfer, so this reads it from the USB hardware where we know how.
// On other processors it returns false and the caller has to treat the content of the read as unknown.
// Both sources are read-only views that the device driver does not depend on.
bool SerialCDC::GetDirectReadProgress(size_t& bytesReceived) const noexcept
{
	if (!rxPending)
	{
		return false;
	}

#if SAME5x
	// The endpoint descriptor table counts the bytes of a multi-packet OUT transfer as its packets arrive
	bytesReceived = reinterpret_cast<const UsbDeviceDescriptor *>(USB->DEVICE.DESCADD.reg)[epOut & 0x0F].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT;
	return true;
#elif SAME70
	// The DMA channel advances its address register as it stores data. The status register also has a count, but reading it clears flags
	bytesReceived = USBHS->USBHS_DEVDMA[(epOut & 0x0F) - 1].USBHS_DEVDMAADDRESS - reinterpret_cast<uint32_t>(rxBuffer);
	return true;
#else
	(void)bytesReceived;
	return false;
#endif
}

bool SerialCDC::IsDirectReadComplete(size_t& bytesRead) noexcept
{
	if (!rxDone)
	{
		return false;
	}

	bytesRead = rxBytes;
	rxDone = false;
	return true;
}

void SerialCDC::ReturnReceiveEndpoint() noexcept
{
	if (rxTaken && !rxPending)
	{
		rxTaken = false;				// must be cleared before the stream can start a transfer, or the hook would swallow its completion
		usbd_edpt_release(0, epOut);
		tud_cdc_n_read(interfaceIndex, nullptr, 0);	// reads nothing, but makes the stream start receiving again
	}
}

// Runs in the USB task. The endpoint is claimed again at once, because completing a transfer releases it
// and the stream could otherwise start one of its own, e.g. when the host drops DTR
bool SerialCDC::DirectReadXferComplete(uint8_t ep_addr, uint32_t xferred_bytes) noexcept
{
	if (!rxTaken || ep_addr != epOut)
	{
		return false;
	}

	(void)usbd_edpt_claim(0, epOut);
	rxBytes = xferred_bytes;
	rxPending = false;
	rxDone = true;
	return true;
}

// Runs in the USB task after a bus reset and enumeration, which closed the endpoints and gave them back to the stream
void SerialCDC::DeviceConfigured() noexcept
{
	rxTaken = false;
	rxPending = false;
	rxDone = false;
	NoteDisconnect();
}

void SerialCDC::DirectXferComplete(uint32_t xferred_bytes) noexcept
{
	if (directMode)
	{
		directXferredBytes = xferred_bytes;
		TaskBase::GiveFromISR(directWaitingTask, NotifyIndices::UsbDirect);
	}
}

// Transfer completion hook - called from cdcd_xfer_cb before normal CDC FIFO processing.
// Returns true to skip FIFO processing (used for zero-copy direct transfers).
extern "C" bool tud_cdc_xfer_cb(uint8_t itf, uint8_t ep_addr,
                                 xfer_result_t result, uint32_t xferred_bytes)
{
	(void)result;
	if (itf < CFG_TUD_CDC)
	{
		SerialCDC *dev = serialCDCInstances[itf];
		if (dev != nullptr && dev->directMode && ep_addr == dev->directWaitingEp)
		{
			dev->DirectXferComplete(xferred_bytes);
			return true;   // skip normal CDC FIFO processing
		}
		if (dev != nullptr && dev->DirectReadXferComplete(ep_addr, xferred_bytes))
		{
			return true;
		}
	}
	return false;  // normal CDC processing
}

// Invoked when the host has configured the device, which follows every bus reset. A reset clears the CDC line state
// without a call to tud_cdc_line_state_cb, so this is where a replugged cable or a restarted host shows up
extern "C" void tud_mount_cb(void)
{
	for (SerialCDC *dev : serialCDCInstances)
	{
		if (dev != nullptr)
		{
			dev->DeviceConfigured();
		}
	}
}

#endif // RTOS

#endif

#endif

// End
