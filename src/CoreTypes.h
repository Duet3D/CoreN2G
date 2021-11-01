/*
 * CoreTypes.h
 *
 *  Created on: 29 Oct 2021
 *      Author: David
 */

#ifndef SRC_CORETYPES_H_
#define SRC_CORETYPES_H_

#include <stdint.h>

// Core types used in interfaces and associated constants

typedef uint8_t DmaChannel;			///< A type that represents a DMA channel number
typedef uint8_t DmaPriority;		///< A type that represents a DMA priority
typedef uint8_t Pin;				///< A type that represents an I/O pin on the microcontroller
typedef uint16_t PwmFrequency;		///< A type that represents a PWM frequency. 0 sometimes means "default".
typedef uint8_t CanAddress;			///< A type that represents the 7-bit CAN address of a board
typedef uint32_t NvicPriority;		///< A type that represents an interrupt priority
typedef uint8_t ExintNumber;		///< A type that represents an EXINT number (used on SAME5x/SAMC21 only)
typedef uint8_t EventNumber;		///< A type that represents an event number (used on SAME5x/SAMC21 only)
typedef __fp16 float16_t;			///< A 16-bit floating point type
static const Pin NoPin = 0xFF;		///< A number that represents no I/O pin
static const Pin Nx = 0xFF;			///< A number that represents no I/O EXINT number (used on SAME5x/SAMC21 only)

#endif /* SRC_CORETYPES_H_ */
