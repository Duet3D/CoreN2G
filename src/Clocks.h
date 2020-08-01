/*
 * Clocks.h
 *
 *  Created on: 1 Aug 2020
 *      Author: David
 */

#ifndef SRC_CLOCKS_H_
#define SRC_CLOCKS_H_

#include <CoreIO.h>

#if SAME5x

/**
 * @brief Initialise crystal oscillator and standard clocks
 * GCLK0 = 120MHz
 * GCLK1 = 32258Hz (1MHz divided by 31)
 * GCLK2 not initialised, reserved for Ethernet PHY on Duet 3 Mini 5+
 * GCLK3 = 60MHz, for peripherals that can't handle 120MHz
 * GCLK4 = 48MHz, for CAN and/or USB and/or step clock
 * MCLK = GCLK0 undivided
 *
 * @param xoscFrequency Crystal frequency in MHz
 * @param xoscNumber Which XOSC we are using, 0 or 1
 */
void InitStandardClocks(unsigned int xoscFrequency, unsigned int xoscNumber) noexcept;

#endif

#if SAMC21

/**
 * @brief Initialise crystal oscillator and standard clocks
 * GCLK0 = 48MHz
 * GCLK1 = 32258Hz (1MHz divided by 31)
 * MCLK = GCLK0 undivided
 *
 * @param xoscFrequency Crystal frequency in MHz
 */
void InitStandardClocks(unsigned int xoscFrequency) noexcept;

#endif

#endif /* SRC_CLOCKS_H_ */
