/*
 * Serial.cpp - simple serial driver for sending messages to an attached PanelDue
 *
 *  Created on: 06 Jun 2026
 *      Author: David
 */

#include "Serial.h"

#include <RTOSIface/RTOSIface.h>

#if STM32
# include <stm32h5xx_hal_rcc.h>
#endif

constexpr uint32_t DiagBaudRate = 57600;		// the baud rate we default to

// Enable the clocks for the SERCOM.
void Serial::EnableUsartClock(uint8_t usartNumber) noexcept
{
	switch (usartNumber)
	{
	case 1:		__HAL_RCC_USART1_CLK_ENABLE(); break;
	case 2:		__HAL_RCC_USART2_CLK_ENABLE(); break;
	case 3:		__HAL_RCC_USART3_CLK_ENABLE(); break;
#if defined(UART4)
	case 4:		__HAL_RCC_UART4_CLK_ENABLE(); break;
#endif /* UART4 */
#if defined(UART5)
	case 5:		__HAL_RCC_UART5_CLK_ENABLE(); break;
#endif /* UART5 */
#if defined(USART6)
	case 6:		__HAL_RCC_USART6_CLK_ENABLE(); break;
#endif /* USART6 */
#if defined(UART7)
	case 7:		__HAL_RCC_UART7_CLK_ENABLE(; break;
#endif /* UART7 */
#if defined(UART8)
	case 8:		__HAL_RCC_UART8_CLK_ENABLE(); break;
#endif /* UART8 */
#if defined(UART9)
	case 9:		__HAL_RCC_UART9_CLK_ENABLE(); break;
#endif /* UART9 */
#if defined(USART10)
	case 10:	__HAL_RCC_USART10_CLK_ENABLE(); break;
#endif /* USART10 */
#if defined(USART11)
	case 11:	__HAL_RCC_USART11_CLK_ENABLE(); break;
#endif /* USART11 */
#if defined(UART12)
	case 12:	__HAL_RCC_UART12_CLK_ENABLE(); break;
#endif /* UART12 */
	}
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
