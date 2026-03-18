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
	Usart * const usarts[] =
	{
		USART0, USART1,
#if SAME70
		USART2,
#endif
	};

	uint8_t usartIds[] =
	{
		ID_USART0, ID_USART1,
#if SAME70
		ID_USART2,
#endif
	};

	inline Usart *GetUsart(uint8_t usartNumber) noexcept { return usarts[usartNumber]; }
	inline uint8_t GetUsartId(uint8_t usartNumber) noexcept { return usartIds[usartNumber]; }
}

#endif

// End
