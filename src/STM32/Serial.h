/*
 * Serial.h
 *
 *  Created on: 8 June 2026
 *      Author: David
 */

#ifndef SRC_STM32_SERIAL_H_
#define SRC_STM32_SERIAL_H_

#include <CoreIO.h>
#include <General/RingBuffer.h>
#include <RTOSIface/RTOSIface.h>

namespace Serial
{
	static USART_TypeDef * const Usarts[] =
	{
		USART1, USART2, USART3, UART4, UART5,
#if STM32H7
		USART6, UART7, UART8
#endif
	};

	constexpr IRQn UsartIRQns[] =
	{
		USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn, UART5_IRQn,
#if STM32H7
		USART6_IRQn, UART7_IRQn, UART8_IRQn
#endif
	};

	void Init() noexcept;

	inline USART_TypeDef *GetUsart(uint8_t usartNumber) noexcept { return Usarts[usartNumber]; }
	inline constexpr IRQn GetUsartIRQn(uint8_t usartNumber) noexcept { return UsartIRQns[usartNumber]; }

	void EnableUsartClock(uint8_t usartNumber) noexcept;
	void InitUart(uint8_t usartNumber, uint32_t baudRated) noexcept;
	void Disable(uint8_t usartNumber) noexcept;

	// Support for serial interrupt vector reassignment

	// Define indirect interrupt handlers so that we can change the interrupt vectors dynamically
	typedef void (*IrqFunc)(void*) noexcept;

	void SetUsartVector(uint8_t usartNumber, IrqFunc f, void *param) noexcept;
	void ReleaseUsartVector(uint8_t usartNumber) noexcept;
}

#endif /* SRC_STM32_SERIAL_H_ */
