/*
 * UsbSerial.cpp
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

#if SUPPORT_USB

#include "SerialCDC.h"
#include <CoreNotifyIndices.h>

#ifdef RTOS
# include <RTOSIface/RTOSIface.h>
#endif

extern "C" {
#include "usb/class/cdc/device/cdcdf_acm.h"
#include "usb/class/cdc/device/cdcdf_acm_desc.h"
}

#if CONF_USBD_HS_SP
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_LS_FS};
static uint8_t single_desc_bytes_hs[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_HS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS
#else
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_DESCES_LS_FS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#endif

static struct usbd_descriptors single_desc[]
    = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
       ,
       {single_desc_bytes_hs, single_desc_bytes_hs + sizeof(single_desc_bytes_hs)}
#endif
};

static SerialCDC *device;
static volatile bool isConnected = false;
static volatile bool sending = false;
static volatile bool receiving = false;

/** Ctrl endpoint buffer */
alignas(4) static uint8_t ctrl_buffer[64];

// Buffers to receive and send data. I am not sure that they need to be aligned.
alignas(4) static uint8_t rxTempBuffer[64];
alignas(4) static uint8_t txTempBuffer[64];

/**
 * \brief Callback invoked when bulk data received
 */
static bool usb_device_cb_bulk_rx(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	receiving = false;
	device->DataReceived(count);
	return false;						// no error
}

/**
 * \brief Callback invoked when bulk data has been sent
 */
static bool usb_device_cb_bulk_tx(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	sending = false;
	device->StartSending();
	return false;						// no error
}

/**
 * \brief Callback invoked when Line State Change
 */
static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
	if (!isConnected)
	{
		/* Clear pending data when the CDC device is opened */
		device->ClearBuffers();

		isConnected = true;

		/* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_bulk_rx);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_bulk_tx);

		/* Start Rx */
		device->StartReceiving();
	}

	/* No error. */
	return false;
}

SerialCDC::SerialCDC(Pin p, size_t numTxSlots, size_t numRxSlots) noexcept : vbusPin(p), cdcInitialised(false)
{
	txBuffer.Init(numTxSlots);
	rxBuffer.Init(numRxSlots);
}

void SerialCDC::Start() noexcept
{
	device = this;

	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	cdcdf_acm_init();
	usbdc_start(single_desc);
	usbdc_attach();
}

void SerialCDC::end() noexcept
{
	isConnected = sending = receiving = false;
	usbdc_detach();
	usbdc_stop();

	cdcdf_acm_deinit();
	cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, nullptr);
	cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, nullptr);
	cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, nullptr);
	cdcInitialised = false;

	usbdc_deinit();
	ClearBuffers();
}

void SerialCDC::CheckCdc() noexcept
{
	if (!cdcInitialised)
	{
		if (cdcdf_acm_is_enabled())
		{
			cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);
			cdcInitialised = true;
		}
	}
	else if (isConnected)
	{
		StartReceiving();
	}
}

bool SerialCDC::IsConnected() const noexcept
{
	return isConnected;
}

// Overridden virtual functions

// Non-blocking read, return -1 if no character available
int SerialCDC::read() noexcept
{
	if (isConnected)
	{
		uint8_t c;
		if (rxBuffer.GetItem(c))
		{
			StartReceiving();
			return c;
		}
	}
	else
	{
		CheckCdc();
	}
	return -1;
}

int SerialCDC::available() noexcept
{
	CheckCdc();
	return rxBuffer.ItemsPresent();
}

void SerialCDC::flush() noexcept
{
	while (isConnected && (sending || !txBuffer.IsEmpty())) { }
}

size_t SerialCDC::canWrite() noexcept
{
	CheckCdc();
	return (isConnected) ? txBuffer.SpaceLeft() : 0;
}

// Write single character, blocking
size_t SerialCDC::write(uint8_t c) noexcept
{
	CheckCdc();
	while (isConnected)
	{
		if (txBuffer.PutItem(c))
		{
			StartSending();
			break;
		}
#ifdef RTOS
		txWaitingTask = RTOSIface::GetCurrentTask();
#endif
		StartSending();
#ifdef RTOS
		TaskBase::TakeIndexed(NotifyIndices::Usb, 50);
#endif
	}
	return 1;
}

// Blocking write block
size_t SerialCDC::write(const uint8_t *buffer, size_t buflen) noexcept
{
	CheckCdc();
	const size_t ret = buflen;
	while (isConnected)
	{
		buflen -= txBuffer.PutBlock(buffer, buflen);
		if (buflen == 0)
		{
			StartSending();
			break;
		}
#ifdef RTOS
		txWaitingTask = RTOSIface::GetCurrentTask();
#endif
		StartSending();
#ifdef RTOS
		TaskBase::TakeIndexed(NotifyIndices::Usb, 50);
#endif
	}
	return ret;
}

void SerialCDC::ClearBuffers() noexcept
{
	txBuffer.Clear();
	rxBuffer.Clear();
}

void SerialCDC::StartSending() noexcept
{
	if (isConnected && !sending)
	{
		sending = true;						// set this first to prevent a race condition between the task and the ISR
		const size_t count = txBuffer.GetBlock(txTempBuffer, sizeof(txTempBuffer));
		if (count == 0)
		{
			/* Finished sending data */
			sending = false;
		}
		else
		{
			const int32_t rc = cdcdf_acm_write(txTempBuffer, count);
#if 0
			(void)rc;
#else
			if (rc != ERR_NONE)
			{
				sending = false;
				if (rc != USB_BUSY)			//TODO currently we will lose data if we had status USB_BUSY
				{
					/* Unregister callbacks again */
					cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, nullptr);
					cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, nullptr);

					/* Stop communication */
					isConnected = receiving = false;
				}
			}
#endif
		}
	}

#ifdef RTOS
	// Wake up the task waiting for data to go, there may be more to write
	const TaskHandle t = txWaitingTask;
	if (t != nullptr && txBuffer.SpaceLeft() >= txBuffer.GetCapacity()/4)
	{
		txWaitingTask = nullptr;
		TaskBase::GiveFromISR(t, NotifyIndices::Usb);
	}
#endif
}

void SerialCDC::StartReceiving() noexcept
{
	if (isConnected && !receiving)
	{
		receiving = true;						// set this first to prevent a race condition between the task and the ISR
		if (rxBuffer.SpaceLeft() < sizeof(rxTempBuffer))
		{
			receiving = false;
		}
		else
		{
			const int32_t rc = cdcdf_acm_read(rxTempBuffer, sizeof(rxTempBuffer));
#if 0
			(void)rc;
#else
			if (rc != ERR_NONE)
			{
				receiving = false;
				if (rc != USB_BUSY)
				{
					/* Unregister callbacks again */
					cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, nullptr);
					cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, nullptr);

					/* Stop communication */
					isConnected = sending = false;
				}
			}
#endif
		}
	}
}

void SerialCDC::DataReceived(uint32_t count) noexcept
{
	rxBuffer.PutBlock(rxTempBuffer, count);
	StartReceiving();
}

#endif

// End
