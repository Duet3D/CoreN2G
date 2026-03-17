/*
 * SpiParameters.h
 *
 *  Created on: 17 Mar 2026
 *      Author: David
 */

#ifndef SRC_HARDWARE_SPI_SPIPARAMETERS_H_
#define SRC_HARDWARE_SPI_SPIPARAMETERS_H_

#include <CoreIO.h>
#include <CoreTypes.h>

// Structure to pass SPI device parameters. The details depend on the MCU.

#if SAME5x

struct SpiParameters
{
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
};

#elif SAME70 || SAM4E || SAM4S

struct SpiParameters
{
	uint8_t usartNumber;
	Pin mosiPin;
	Pin misoPin;
	Pin sclkPin;
	GpioPinFunction pinFunction;
};

#endif

#endif /* SRC_HARDWARE_SPI_SPIPARAMETERS_H_ */
