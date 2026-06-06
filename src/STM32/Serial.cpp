/*
 * Serial.cpp - simple serial driver for sending messages to an attached PanelDue
 *
 *  Created on: 06 Jun 2026
 *      Author: David
 */

#include "Serial.h"

#include <RTOSIface/RTOSIface.h>

constexpr uint32_t DiagBaudRate = 57600;		// the baud rate we default to

// Enable the clocks for the SERCOM.
void Serial::EnableUsartClock(uint8_t usartNumber) noexcept
{
	qq;	//TODO
}

// Initialise the serial port. This does not set up the I/O pins. It assumes that we always transmit on pad 0.
void Serial::InitUart(uint8_t usartNumber, uint32_t baudRate) noexcept
{
	EnableUsartClock(usartNumber);
	USART_TypeDef * const usart = GetUsart(usartNumber);
	qq;	//TODO
}

// Undo the initialisation, so that when we jump into the main firmware the USART can be initialised again
void Serial::Disable(uint8_t usartNumber) noexcept
{
	USART_TypeDef * const usart = GetUsart(usartNumber);
	qq;	//TODO
}

static void DummyHandler(void*) noexcept
{
	// Maybe we should record an exception instead of just looping?
	while (1) { }
}

constexpr unsigned int NumUsarts =
#if STM32H5
				5;
#elif STM32H7
				7;
#endif
static Serial::IrqFunc usartIrq[NumUsarts];
static void *usartParam[NumUsarts];

void Serial::SetUsartVector(uint8_t usartNumber, Serial::IrqFunc f, void *param) noexcept
{
	usartParam[usartNumber] = param;
	usartIrq[usartNumber] = f;
}

void Serial::ReleaseUsartVector(uint8_t usartNumber) noexcept
{
	usartIrq[usartNumber] = DummyHandler;
}

#define DEFINE_USART_IRQ(_usart) \
	void USART ## _usart ## _Handler() noexcept { usartIrq[_usart - 1](usartParam[_usart - 1]); }

DEFINE_USART_IRQ(1)
DEFINE_USART_IRQ(2)
DEFINE_USART_IRQ(3)
DEFINE_USART_IRQ(4)
DEFINE_USART_IRQ(5)

#if STM32H7

DEFINE_USART_IRQ(6)
DEFINE_USART_IRQ(7)

#endif

void Serial::Init() noexcept
{
	for (IrqFunc& f : usartIrq)
	{
		f = DummyHandler;
	}
}

// End
