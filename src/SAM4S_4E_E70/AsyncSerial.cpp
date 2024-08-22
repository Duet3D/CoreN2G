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

#include "AsyncSerial.h"
#include <CoreNotifyIndices.h>
#include <asf.h>

#include <cstdlib>
#include <cstring>
#include <algorithm>		// for std::swap

// Constructors ////////////////////////////////////////////////////////////////

AsyncSerial::AsyncSerial(Uart* pUart, IRQn_Type p_irqn, uint32_t p_id, size_t numTxSlots, size_t numRxSlots, OnBeginFn p_onBegin, OnEndFn p_onEnd) noexcept
	: _pUart(pUart), irqn(p_irqn), id(p_id),
	  interruptCallback(nullptr), onBegin(p_onBegin), onEnd(p_onEnd), onTransmissionEndedFn(nullptr),
	  numInterruptBytesMatched(0), txEnabled(false)
{
	txBuffer.Init(numTxSlots);
	rxBuffer.Init(numRxSlots);
}

// Public Methods //////////////////////////////////////////////////////////////

void AsyncSerial::begin(uint32_t dwBaudRate) noexcept
{
	begin(dwBaudRate, Mode_8N1);
}

void AsyncSerial::begin(uint32_t dwBaudRate, UARTModes config) noexcept
{
	uint32_t modeReg = static_cast<uint32_t>(config) & 0x00000E00;
	init(dwBaudRate, modeReg | UART_MR_CHMODE_NORMAL);
}

void AsyncSerial::init(const uint32_t dwBaudRate, const uint32_t modeReg) noexcept
{
	// Configure PMC
	pmc_enable_periph_clk(id);
	onBegin(this);

#if !SAME70
	// Disable PDC channel
	_pUart->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
#endif

	// Reset and disable receiver and transmitter
	_pUart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;

	// Configure mode
	_pUart->UART_MR = modeReg;

	// Configure baudrate (asynchronous, no oversampling)
	const uint32_t br16 = dwBaudRate * 16;
	_pUart->UART_BRGR = (SystemPeripheralClock() + (br16/2) - 1) / br16;

	// Make sure both ring buffers are initialized back to empty.
	txBuffer.Clear();
	rxBuffer.Clear();
	bufferOverrunPending = false;

	// Configure interrupts
	_pUart->UART_IDR = 0xFFFFFFFF;
	_pUart->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;

	// Enable UART interrupt in NVIC
	NVIC_EnableIRQ(irqn);

	// Enable receiver and transmitter
	errors.all = 0;
	_pUart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
	txEnabled = true;
}

void AsyncSerial::end( void ) noexcept
{
	// Clear any received data
	rxBuffer.Clear();

	// Wait for any outstanding data to be sent
	flush();
	txEnabled = false;

	// Disable UART interrupt in NVIC
	NVIC_DisableIRQ(irqn);

	pmc_disable_periph_clk(id);
}

void AsyncSerial::setInterruptPriority(uint32_t priority) noexcept
{
	NVIC_SetPriority(irqn, priority & 0x0F);
}

uint32_t AsyncSerial::getInterruptPriority() noexcept
{
	return NVIC_GetPriority(irqn);
}

int AsyncSerial::available() noexcept
{
	return rxBuffer.ItemsPresent();
}

size_t AsyncSerial::canWrite() noexcept
{
	return txBuffer.SpaceLeft();
}

int AsyncSerial::read() noexcept
{
	uint8_t c;
	return (rxBuffer.GetItem(c)) ? c : -1;
}

void AsyncSerial::flush( void ) noexcept
{
	while (!txBuffer.IsEmpty()) { }									// wait for transmit data to be sent from the buffer
	while ((_pUart->UART_SR & UART_SR_TXRDY) != UART_SR_TXRDY) { }	// wait for transmission to complete
}

// Write single character, blocking unless transmission is disabled
size_t AsyncSerial::write(uint8_t uc_data) noexcept
{
	if (txEnabled && txBuffer.IsEmpty() && (_pUart->UART_SR & UART_SR_TXRDY) != 0)
	{
		_pUart->UART_THR = uc_data;									// bypass buffering and send character directly
	}
	else
	{
		for (;;)
		{
			if (txBuffer.PutItem(uc_data))
			{
				if (txEnabled)
				{
					_pUart->UART_IER = UART_IER_TXRDY;
				}
				break;
			}
			if (!txEnabled)
			{
				return 0;
			}
#ifdef RTOS
			txWaitingTask = RTOSIface::GetCurrentTask();
#endif
			_pUart->UART_IER = UART_IER_TXRDY;
#ifdef RTOS
			TaskBase::TakeIndexed(NotifyIndices::UartTx, 50);
#endif
		}
	}
	return 1;
}

// Write block, blocking if transmitter is enabled
size_t AsyncSerial::write(const uint8_t *buffer, size_t buflen) noexcept
{
	size_t ret = 0;
	for (;;)
	{
		const size_t numPut = txBuffer.PutBlock(buffer, buflen);
		buflen -= numPut;
		buffer += numPut;
		ret += numPut;

		if (!txEnabled)
		{
			break;
		}

		if (buflen == 0)
		{
			_pUart->UART_IER = UART_IER_TXRDY;
			break;
		}
#ifdef RTOS
		txWaitingTask = RTOSIface::GetCurrentTask();
#endif
	    _pUart->UART_IER = UART_IER_TXRDY;
#ifdef RTOS
		TaskBase::TakeIndexed(NotifyIndices::UartTx, 50);
#endif
	}
	return ret;
}

void AsyncSerial::ClearTransmitBuffer() noexcept
{
	AtomicCriticalSectionLocker lock;
	txBuffer.Clear();
}

void AsyncSerial::ClearReceiveBuffer() noexcept
{
	AtomicCriticalSectionLocker lock;
	rxBuffer.Clear();
}

void AsyncSerial::DisableTransmit() noexcept
{
	txEnabled = false;
	_pUart->UART_IDR = UART_IDR_TXRDY;
}

void AsyncSerial::EnableTransmit() noexcept
{
	txEnabled = true;
	_pUart->UART_IER = UART_IER_TXRDY;
}

void AsyncSerial::IrqHandler() noexcept
{
	const uint32_t status = _pUart->UART_SR;

	// Did we receive data?
	if ((status & UART_SR_RXRDY) != 0)
	{
		const uint8_t c = _pUart->UART_RHR;
		if (c == interruptSeq[numInterruptBytesMatched])
		{
			++numInterruptBytesMatched;
			if (numInterruptBytesMatched == ARRAY_SIZE(interruptSeq))
			{
				numInterruptBytesMatched = 0;
				if (interruptCallback != nullptr)
				{
					interruptCallback(this);
				}
			}
		}
		else
		{
			numInterruptBytesMatched = 0;
		}

		if (bufferOverrunPending)
		{
			if (rxBuffer.PutItem(0x7F))
			{
				bufferOverrunPending = false;
				(void)rxBuffer.PutItem(c);					// we don't much care whether this succeeds or not
			}
		}
		else if (!rxBuffer.PutItem(c))
		{
			++errors.bufferOverrun;
			bufferOverrunPending = true;
		}
	}

	// Do we need to keep sending data?
	if ((status & UART_SR_TXRDY) != 0)
	{
		uint8_t c;
		if (txBuffer.GetItem(c))
		{
			_pUart->UART_THR = c;
#ifdef RTOS
			if (txWaitingTask != nullptr && txBuffer.SpaceLeft() >= txBuffer.GetCapacity()/2)
			{
				TaskBase::GiveFromISR(txWaitingTask, NotifyIndices::UartTx);
				txWaitingTask = nullptr;
			}
#endif
		}
		else
		{
			_pUart->UART_IDR = UART_IDR_TXRDY;						// mask off transmit interrupt so we don't get it anymore
			if (onTransmissionEndedFn != nullptr)					// if we want callback when the transmitter is empty
			{
				if ((status & UART_SR_TXEMPTY) != 0)				// if it is already empty
				{
					_pUart->UART_IDR = UART_IER_TXEMPTY;			// disable the transmitter empty interrupt
					onTransmissionEndedFn(onTransmissionEndedCp);	// execute the callback
				}
				else
				{
					_pUart->UART_IER = UART_IER_TXEMPTY;			// enable the transmitter empty interrupt
				}
			}
#ifdef RTOS
			if (txWaitingTask != nullptr)
			{
				TaskBase::GiveFromISR(txWaitingTask, NotifyIndices::UartTx);
				txWaitingTask = nullptr;
			}
#endif
		}
	}

	// Acknowledge errors
	if ((status & (UART_SR_OVRE | UART_SR_FRAME)) != 0)
	{
		if (status & UART_SR_OVRE)
		{
			++errors.uartOverrun;
		}
		if (status & UART_SR_FRAME)
		{
			++errors.framing;
		}
		_pUart->UART_CR = UART_CR_RSTSTA;
		rxBuffer.PutItem(0x7F);
	}
}

AsyncSerial::InterruptCallbackFn AsyncSerial::SetInterruptCallback(InterruptCallbackFn f) noexcept
{
	const InterruptCallbackFn ret = interruptCallback;
	interruptCallback = f;
	return ret;
}

AsyncSerial::OnTransmissionEndedFn AsyncSerial::SetOnTxEndedCallback(OnTransmissionEndedFn f, CallbackParameter cp) noexcept
{
	AtomicCriticalSectionLocker lock;
	const OnTransmissionEndedFn ret = onTransmissionEndedFn;
	onTransmissionEndedFn = f;
	onTransmissionEndedCp = cp;
	return ret;
}

// Get and clear the errors
AsyncSerial::Errors AsyncSerial::GetAndClearErrors() noexcept
{
	Errors errs;
	std::swap(errs, errors);
	return errs;
}

// End
