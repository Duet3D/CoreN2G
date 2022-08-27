/*
 * VirtualCanRegisters.h
 *
 *  Created on: 26 Aug 2022
 *      Author: David
 */

#ifndef SRC_RP2040_VIRTUALCANREGISTERS_H_
#define SRC_RP2040_VIRTUALCANREGISTERS_H_

#include <CoreIO.h>

struct VirtualCanRegisters
{
	uint32_t errors;
};

#endif /* SRC_RP2040_VIRTUALCANREGISTERS_H_ */
