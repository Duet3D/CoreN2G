/*
 * Serial.h
 *
 *  Created on: 9 Aug 2019
 *      Author: David
 */

#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_

#include <CoreIO.h>
#include <usart/usart.h>

namespace Serial
{
	// Define indirect interrupt handlers so that we can change the interrupt vectors dynamically
	typedef void (*IrqFunc)(void*) noexcept;

#if SAM4E || SAM4S
	constexpr size_t NumUarts = 2;
	constexpr size_t NumUsarts = 2;
#elif SAME70
	constexpr size_t NumUarts = 5;
	constexpr size_t NumUsarts = 3;
#endif

	void Init() noexcept;

	Uart * const uarts[] =
	{
		UART0, UART1,
#if SAME70
		UART2, UART3, UART4,
#endif
	};

	constexpr uint8_t uartIds[] =
	{
		ID_UART0, ID_UART1,
#if SAME70
		ID_UART2, ID_UART3, ID_UART4,
#endif
	};

	static_assert(ARRAY_SIZE(uarts) == NumUarts);
	static_assert(ARRAY_SIZE(uartIds) == NumUarts);

	inline Uart *GetUart(uint8_t uartNumber) noexcept { return uarts[uartNumber]; }
	inline uint8_t GetUartId(uint8_t uartNumber) noexcept { return uartIds[uartNumber]; }

	void SetUartVector(uint8_t uartNumber, IrqFunc f, void *param) noexcept;
	void ReleaseUartVector(uint8_t uartNumber) noexcept;

	Usart * const usarts[] =
	{
		USART0, USART1,
#if SAME70
		USART2,
#endif
	};

	constexpr uint8_t usartIds[] =
	{
		ID_USART0, ID_USART1,
#if SAME70
		ID_USART2,
#endif
	};

	static_assert(ARRAY_SIZE(usarts) == NumUsarts);
	static_assert(ARRAY_SIZE(usartIds) == NumUsarts);

	inline Usart *GetUsart(uint8_t usartNumber) noexcept { return usarts[usartNumber]; }
	inline uint8_t GetUsartId(uint8_t usartNumber) noexcept { return usartIds[usartNumber]; }

	void SetUsartVector(uint8_t usartNumber, IrqFunc f, void *param) noexcept;
	void ReleaseUsartVector(uint8_t usartNumber) noexcept;

	// Functions that act on a UART or USART depending on whether or not the top bit of the instance number is set
	inline Uart *GetUartOrUsart(uint8_t uartOrUsartNumber) noexcept
	{
		return (uartOrUsartNumber & 0x80) ? reinterpret_cast<Uart *>(GetUsart(uartOrUsartNumber & 0x7F)) : GetUart(uartOrUsartNumber);
	}

	inline uint8_t GetUartOrUsartId(uint8_t uartOrUsartNumber) noexcept
	{
		return (uartOrUsartNumber & 0x80) ? GetUsartId(uartOrUsartNumber & 0x7F) : GetUartId(uartOrUsartNumber);
	}

	inline void SetUartOrUsartVector(uint8_t uartOrUsartNumber, IrqFunc f, void *param) noexcept
	{
		if (uartOrUsartNumber & 0x80) { SetUsartVector(uartOrUsartNumber & 0x7F, f, param); }
		else { SetUartVector(uartOrUsartNumber, f, param); }
	}

	inline void ReleaseUartOrUsartVector(uint8_t uartOrUsartNumber) noexcept
	{
		if (uartOrUsartNumber & 0x80) { ReleaseUsartVector(uartOrUsartNumber & 0x7F); }
		else { ReleaseUartVector(uartOrUsartNumber); }
	}
}

#endif

// End
