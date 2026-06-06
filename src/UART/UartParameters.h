/*
 * UartParameters.h
 *
 *  Created on: 21 Mar 2026
 *      Author: David
 */

#ifndef SRC_UART_UARTPARAMETERS_H_
#define SRC_UART_UARTPARAMETERS_H_

#include <CoreIO.h>

// Structure to pass async serial device parameters. The details depend on the MCU.
struct UartParameters
{
#if SAME5x || SAMC21
	uint8_t sercomNumber;
	Pin rxPin;
	Pin txPin;
	GpioPinFunction pinFunction;
	uint8_t dataInPad;
	uint8_t dataOutPad;
	size_t numRxSlots;
	size_t numTxSlots;
#elif SAM4S || SAM4E || SAME70
	uint8_t uartOrUsartInstance;				// the uart number, or the usart number or'ed with 0x80
	Pin rxPin;
	Pin txPin;
	GpioPinFunction pinFunction;
	size_t numRxSlots;
	size_t numTxSlots;
#elif STM32
	uint8_t instanceNumber;						// the STM32 UART/USART number starting at 1
	Pin txPin;
	Pin txPin;
	GpioPinFunction pinFunction;
	size_t numRxSlots;
	size_t numTxSlots;
#elif RP2040
	uint8_t instanceNumber;
	Pin rxPin;
	Pin txPin;
	size_t numRxSlots;
	size_t numTxSlots;
#endif
};

#endif /* SRC_UART_UARTPARAMETERS_H_ */
