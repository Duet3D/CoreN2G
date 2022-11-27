/*
 * WS2812.h
 *
 *  Created on: 10 Oct 2022
 *      Author: David
 */

#ifndef SRC_RP2040_WS2812_H_
#define SRC_RP2040_WS2812_H_

#include <CoreIO.h>
#include <hardware/structs/pio.h>

// Class to drive WS2812 LEDs on RP2040. Multiple instances can be used, subject to available PIO state machines.
class WS2812
{
public:
	WS2812(Pin p_pin, bool p_isRgbw, unsigned int p_dmaChan) noexcept;
	~WS2812();

	// Send data, one 32-bit word per WS2812 LED. Either 24 or 32 bits are sent from each word depending on the isRgbw flag.
	// The data format of each 32-bit word is:
	//  Bits 24-31 Green intensity
	//  Bits 16-23 Red intensity
	//  Bits 8-15  Blue intensity
	//  Bits 0-7   White intensity (ignored for RGB LEDs)
	void SendData(const uint32_t *data, unsigned int numLeds) noexcept;

	// Set all LEDs to the same colour (0 = off). Colour encoding is as above.
	void SetColour(uint32_t colour, unsigned int numLeds) noexcept;

private:
	static bool pioProgramLoaded;
	static unsigned int pioProgramOffset;

	pio_hw_t *hw;
	int stateMachineNumber;
	uint32_t frequency = 800000;		// default timing = 1.25us per bit
	unsigned int dmaChan;
	Pin pin;
	bool isRgbw;
};

#endif /* SRC_RP2040_WS2812_H_ */
