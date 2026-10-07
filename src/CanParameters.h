/*
 * CanParameters.h
 *
 *  Created on: 23 Jun 2026
 *      Author: David
 */

#ifndef SRC_CANPARAMETERS_H_
#define SRC_CANPARAMETERS_H_

#include <CoreIO.h>

// Struct used to define the CAN peripheral and the pins it uses, passed to CanInterface::Init()
struct CanParameters
{
	unsigned int instanceNumber;
	Pin txPin;
	Pin rxPin;
#if SAME70									// on the SAME70 we sometimes use different functions on the Tx and Rx pins
	GpioPinFunction txPinFunction;
	GpioPinFunction rxPinFunction;
#else
	GpioPinFunction pinsFunction;
#endif
};

#endif /* SRC_CANPARAMETERS_H_ */
