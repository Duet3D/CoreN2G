/*
 * PIOassignments.h
 *
 *  Created on: 9 Oct 2022
 *      Author: David
 */

#ifndef SRC_RP2040_PIOASSIGNMENTS_H_
#define SRC_RP2040_PIOASSIGNMENTS_H_

// RP2040 PUI state machine assignments used by CoreN2G

// The CAN-FD interface needs all four state machines of one PIO
constexpr unsigned int CanPioNumber = 0;

// The TMC single-wire UART interface needs two state machines of a PIO
constexpr unsigned int TmcUartPioNumber = 1;
constexpr unsigned int TmcUartTxStateMachineNumber = 0;
constexpr unsigned int TmcUartRxStateMachineNumber = 1;

// The WS2812 LED driver needs one state machine
constexpr unsigned int WS2812PioNumber = 1;
constexpr unsigned int WS2812StateMachineNunmber = 2;

#endif /* SRC_RP2040_PIOASSIGNMENTS_H_ */
