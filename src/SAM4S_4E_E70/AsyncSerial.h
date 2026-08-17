/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SRC_HARDWARE_SAME4S_4E_E70_ASYNC_SERIAL_H_
#define SRC_HARDWARE_SAME4S_4E_E70_ASYNC_SERIAL_H_

#include <CoreIO.h>
#include <Stream.h>
#include <UART/UartParameters.h>
#include <UART/UartMode.h>
#include <General/RingBuffer.h>

#ifdef RTOS
# include <RTOSIface/RTOSIface.h>
#endif

#if SAM4E || SAME70
#include "component/usart.h"
#else
#include "component/component_usart.h"
#endif

#define SERIAL_8N1 UARTClass::Mode_8N1
#define SERIAL_8E1 UARTClass::Mode_8E1
#define SERIAL_8O1 UARTClass::Mode_8O1
#define SERIAL_8M1 UARTClass::Mode_8M1
#define SERIAL_8S1 UARTClass::Mode_8S1


class AsyncSerial : public Stream
{
public:
	typedef void (*InterruptCallbackFn)(AsyncSerial *_ecv_from) noexcept;
	typedef void (*OnBeginFn)(AsyncSerial*_ecv_from) noexcept;
	typedef void (*OnEndFn)(AsyncSerial*_ecv_from) noexcept;
	typedef void (*OnTransmissionEndedFn)(CallbackParameter cp) noexcept;

	union Errors
	{
		uint32_t all;
		struct
		{
			uint32_t uartOverrun : 11,
					 framing : 11,
					 bufferOverrun : 10;
		};

		Errors() noexcept { all = 0; }
	};

	AsyncSerial(const UartParameters& params) noexcept;

	void begin(uint32_t baudRate, UartMode mode = UartMode::Mode8N1) noexcept;
	void end() noexcept;
	int available(void) noexcept override;
	int read() noexcept override;
	void flush() noexcept override;
	size_t write(uint8_t c) noexcept override;
	size_t write(const uint8_t *_ecv_array buffer, size_t buflen) noexcept override;

	size_t canWrite() noexcept override;

	void ClearTransmitBuffer() noexcept;
	void ClearReceiveBuffer() noexcept;
	void DisableTransmit() noexcept;
	void EnableTransmit() noexcept;

	void setInterruptPriority(uint32_t priority) noexcept;
	uint32_t getInterruptPriority() noexcept;

	InterruptCallbackFn _ecv_null SetInterruptCallback(InterruptCallbackFn _ecv_null f) noexcept;
	OnTransmissionEndedFn _ecv_null SetOnTxEndedCallback(OnTransmissionEndedFn _ecv_null f, CallbackParameter cp) noexcept;

	// Get and clear the errors
	Errors GetAndClearErrors() noexcept;

protected:
	void init(const uint32_t dwBaudRate, const uint32_t config) noexcept;

	void IrqHandler() noexcept;
	static void GlobalIrqHandler(void *device) noexcept;

	RingBuffer<uint8_t> txBuffer;
	RingBuffer<uint8_t> rxBuffer;

	Uart* const _pUart;
	const int8_t id;
#ifdef RTOS
	volatile TaskHandle _ecv_null txWaitingTask = nullptr;
#endif
	InterruptCallbackFn _ecv_null interruptCallback = nullptr;
	OnTransmissionEndedFn _ecv_null onTransmissionEndedFn = nullptr;
	CallbackParameter onTransmissionEndedCp;
	Errors errors;

	const uint8_t uartOrUsartInstance;
	const Pin rxPin;
	const Pin txPin;
	const GpioPinFunction pinFunction;

	uint8_t numInterruptBytesMatched = 0;
	bool bufferOverrunPending = false;
    bool txEnabled = false;

	static constexpr uint8_t interruptSeq[2] = { 0xF0, 0x0F };
};

#endif // SRC_HARDWARE_SAME4S_4E_E70_ASYNC_SERIAL_H_
