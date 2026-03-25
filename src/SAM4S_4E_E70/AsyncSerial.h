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

	enum UARTModes {
		Mode_8N1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_NO,
		Mode_8E1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_EVEN,
		Mode_8O1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_ODD,
		Mode_8M1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_MARK,
		Mode_8S1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_SPACE,

		// The following are available for USARTs but possibly not for UARTs, We don't use them so I (DC) haven't checked.
		Mode_5N1 = US_MR_CHRL_5_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_1_BIT,
		Mode_6N1 = US_MR_CHRL_6_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_1_BIT,
		Mode_7N1 = US_MR_CHRL_7_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_1_BIT,
		Mode_5N2 = US_MR_CHRL_5_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_2_BIT,
		Mode_6N2 = US_MR_CHRL_6_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_2_BIT,
		Mode_7N2 = US_MR_CHRL_7_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_2_BIT,
		Mode_8N2 = US_MR_CHRL_8_BIT | US_MR_PAR_NO    | US_MR_NBSTOP_2_BIT,
		Mode_5E1 = US_MR_CHRL_5_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_1_BIT,
		Mode_6E1 = US_MR_CHRL_6_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_1_BIT,
		Mode_7E1 = US_MR_CHRL_7_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_1_BIT,
		Mode_5E2 = US_MR_CHRL_5_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_2_BIT,
		Mode_6E2 = US_MR_CHRL_6_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_2_BIT,
		Mode_7E2 = US_MR_CHRL_7_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_2_BIT,
		Mode_8E2 = US_MR_CHRL_8_BIT | US_MR_PAR_EVEN  | US_MR_NBSTOP_2_BIT,
		Mode_5O1 = US_MR_CHRL_5_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_1_BIT,
		Mode_6O1 = US_MR_CHRL_6_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_1_BIT,
		Mode_7O1 = US_MR_CHRL_7_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_1_BIT,
		Mode_5O2 = US_MR_CHRL_5_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_2_BIT,
		Mode_6O2 = US_MR_CHRL_6_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_2_BIT,
		Mode_7O2 = US_MR_CHRL_7_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_2_BIT,
		Mode_8O2 = US_MR_CHRL_8_BIT | US_MR_PAR_ODD   | US_MR_NBSTOP_2_BIT,
		Mode_5M1 = US_MR_CHRL_5_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_1_BIT,
		Mode_6M1 = US_MR_CHRL_6_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_1_BIT,
		Mode_7M1 = US_MR_CHRL_7_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_1_BIT,
		Mode_5M2 = US_MR_CHRL_5_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_2_BIT,
		Mode_6M2 = US_MR_CHRL_6_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_2_BIT,
		Mode_7M2 = US_MR_CHRL_7_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_2_BIT,
		Mode_8M2 = US_MR_CHRL_8_BIT | US_MR_PAR_MARK  | US_MR_NBSTOP_2_BIT,
		Mode_5S1 = US_MR_CHRL_5_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_1_BIT,
		Mode_6S1 = US_MR_CHRL_6_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_1_BIT,
		Mode_7S1 = US_MR_CHRL_7_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_1_BIT,
		Mode_5S2 = US_MR_CHRL_5_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_2_BIT,
		Mode_6S2 = US_MR_CHRL_6_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_2_BIT,
		Mode_7S2 = US_MR_CHRL_7_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_2_BIT,
		Mode_8S2 = US_MR_CHRL_8_BIT | US_MR_PAR_SPACE | US_MR_NBSTOP_2_BIT,
	};

	AsyncSerial(const UartParameters& params) noexcept;

	virtual void begin(uint32_t dwBaudRate, UARTModes config) noexcept;
	void begin(uint32_t dwBaudRate) noexcept;

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
