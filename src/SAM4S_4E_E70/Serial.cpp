/*
 * Serial.cpp
 *
 *  Created on: 24 Mar 2026
 *      Author: David
 */

#include "Serial.h"

static void DummyHandler(void*) noexcept
{
	// Maybe we should record an exception instead of just looping?
	while (1) { }
}

static Serial::IrqFunc uartIrq[Serial::NumUarts];
static void *uartParam[Serial::NumUarts];

void Serial::SetUartVector(uint8_t uartNumber, Serial::IrqFunc f, void *param) noexcept
{
	uartParam[uartNumber] = param;
	uartIrq[uartNumber] = f;
}

void Serial::ReleaseUartVector(uint8_t uartNumber) noexcept
{
	uartIrq[uartNumber] = DummyHandler;
}

# define DEFINE_UART_IRQ(_uart) \
	void UART ## _uart ## _Handler() noexcept { uartIrq[_uart](uartParam[_uart]); }

DEFINE_UART_IRQ(0)
DEFINE_UART_IRQ(1)
#if SAME70
DEFINE_UART_IRQ(2)
DEFINE_UART_IRQ(3)
DEFINE_UART_IRQ(4)
#endif

static Serial::IrqFunc usartIrq[Serial::NumUsarts];
static void *usartParam[Serial::NumUsarts];

void Serial::SetUsartVector(uint8_t usartNumber, Serial::IrqFunc f, void *param) noexcept
{
	usartParam[usartNumber] = param;
	usartIrq[usartNumber] = f;
}

void Serial::ReleaseUsartVector(uint8_t usartNumber) noexcept
{
	usartIrq[usartNumber] = DummyHandler;
}

# define DEFINE_USART_IRQ(_usart) \
	void USART ## _usart ## _Handler() noexcept { usartIrq[_usart](usartParam[_usart]); }

DEFINE_USART_IRQ(0)
DEFINE_USART_IRQ(1)
#if SAME70
DEFINE_USART_IRQ(2)
#endif

void Serial::Init() noexcept
{
	for (IrqFunc& f : uartIrq)
	{
		f = DummyHandler;
	}
	for (IrqFunc& f : usartIrq)
	{
		f = DummyHandler;
	}
}

// End
