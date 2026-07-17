/*
 * UartMode.h
 *
 *  Created on: 17 Jul 2026
 *      Author: David
 */

#ifndef SRC_UART_UARTMODE_H_
#define SRC_UART_UARTMODE_H_

#include <cstdint>

// Support UART modes
// Currently we support only 8 data bits with {no, even, odd} parity and one stop bit.
enum class UartMode : uint8_t { Mode8N1 = 0, Mode8E1, Mode8O1 };

#endif /* SRC_UART_UARTMODE_H_ */
