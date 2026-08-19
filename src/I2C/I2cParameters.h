/*
 * I2cParameters.h
 *
 *  Created on: 18 Mar 2026
 *      Author: David
 */

#ifndef SRC_I2C_I2CPARAMETERS_H_
#define SRC_I2C_I2CPARAMETERS_H_

#include <CoreIO.h>

struct I2cParameters
{
#if SAME5x || SAMC21
	uint8_t sercomNumber;
	Pin sclPin;
	Pin sdaPin;
	GpioPinFunction pinFunction;
	NvicPriority irqPriority;
	DmaChannel rxDmaChannel;			// channel used to receive long reads, NoDmaChannel to receive everything under interrupt
	DmaPriority rxDmaPriority;
#elif RP2040
	uint8_t instanceNumber;
	Pin sclPin;
	Pin sdaPin;
	NvicPriority irqPriority;
#endif
};

#endif /* SRC_I2C_I2CPARAMETERS_H_ */
