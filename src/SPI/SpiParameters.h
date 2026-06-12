/*
 * SpiParameters.h
 *
 *  Created on: 17 Mar 2026
 *      Author: David
 */

#ifndef SRC_HARDWARE_SPI_SPIPARAMETERS_H_
#define SRC_HARDWARE_SPI_SPIPARAMETERS_H_

#include <CoreIO.h>

// Structure to pass SPI device parameters. The details depend on the MCU.
struct SpiParameters
{
#if SAME5x || SAMC21
	uint8_t sercomNumber;
	Pin mosiPin;
	Pin misoPin;
	Pin sclkPin;
	GpioPinFunction pinFunction;
	uint8_t dataInPad;
	uint8_t dataOutPad;
	DmaChannel dmaChanTx;
	DmaChannel dmaChanRx;
	DmaPriority dmaPrioTx;
	DmaPriority dmaPrioRx;
#elif SAME70 || SAM4E || SAM4S
	uint8_t usartNumber;
	Pin mosiPin;
	Pin misoPin;
	Pin sclkPin;
	GpioPinFunction pinFunction;
#elif STM32
	uint8_t instanceNumber;
	Pin mosiPin;
	Pin misoPin;
	Pin sclkPin;
	GpioPinFunction pinFunction;
	uint8_t dataInPad;
	uint8_t dataOutPad;
	DmaChannel dmaChanTx;
	DmaChannel dmaChanRx;
	DmaPriority dmaPrioTx;
	DmaPriority dmaPrioRx;
#elif RPXXXX
	uint8_t instanceNumber;
	Pin mosiPin;
	Pin misoPin;
	Pin sclkPin;
#endif
};

#endif /* SRC_HARDWARE_SPI_SPIPARAMETERS_H_ */
