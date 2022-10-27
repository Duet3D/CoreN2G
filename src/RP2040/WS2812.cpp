/*
 * WS2812.cpp
 *
 *  Created on: 10 Oct 2022
 *      Author: David
 */

#include "WS2812.h"
#include "PIOassignments.h"

#include <hardware/pio.h>
#include <hardware/dma.h>

bool WS2812::pioProgramLoaded = false;
unsigned int WS2812::pioProgramOffset;

// ------ //
// ws2812 //
// ------ //

#define ws2812_wrap_target 0
#define ws2812_wrap 3

#define ws2812_T1 2
#define ws2812_T2 5
#define ws2812_T3 3

static const uint16_t ws2812_program_instructions[] =
{
            //     .wrap_target
    0x6221, //  0: out    x, 1            side 0 [2]
    0x1123, //  1: jmp    !x, 3           side 1 [1]
    0x1400, //  2: jmp    0               side 1 [4]
    0xa442, //  3: nop                    side 0 [4]
            //     .wrap
};

static const struct pio_program ws2812_program =
{
    .instructions = ws2812_program_instructions,
    .length = 4,
    .origin = -1,
};

static inline pio_sm_config ws2812_program_get_default_config(uint offset)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + ws2812_wrap_target, offset + ws2812_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

// Constructor. Ideally we would allocate the DMA channel dynamically, but RRF currently uses static channel allocations.
WS2812::WS2812(Pin p_pin, bool p_isRgbw, unsigned int p_dmaChan) noexcept
	: hw((WS2812PioNumber) ? pio1_hw : pio0_hw), stateMachineNumber(-1), dmaChan(p_dmaChan), pin(p_pin), isRgbw(p_isRgbw)
{
	if (!pioProgramLoaded)
	{
		if (!pio_can_add_program(hw, &ws2812_program))
		{
			return;
		}
		pioProgramOffset = pio_add_program(hw, &ws2812_program);
		pioProgramLoaded = true;
	}

	stateMachineNumber = pio_claim_unused_sm(hw, false);
	if (stateMachineNumber < 0)
	{
		return;
	}

	dma_channel_claim(dmaChan);

	// Set up the state machine
	pio_gpio_init(hw, pin);
	pio_sm_set_consecutive_pindirs(hw, stateMachineNumber, pin, 1, true);

	pio_sm_config c = ws2812_program_get_default_config(pioProgramOffset);
	sm_config_set_sideset_pins(&c, pin);
	sm_config_set_out_shift(&c, false, true, isRgbw ? 32 : 24);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

	const unsigned int cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3;
	const float div = SystemCoreClock / (frequency * cycles_per_bit);
	sm_config_set_clkdiv(&c, div);

	pio_sm_init(hw, stateMachineNumber, pioProgramOffset, &c);
	pio_sm_set_enabled(hw, stateMachineNumber, true);
}

WS2812::~WS2812()
{
	if (stateMachineNumber >= 0)
	{
		dma_channel_unclaim(dmaChan);
		pio_sm_unclaim(hw, stateMachineNumber);
	}
}

void WS2812::SendData(const uint32_t *data, unsigned int numLeds) noexcept
{
	dma_channel_config config = dma_channel_get_default_config(dmaChan);
	channel_config_set_read_increment(&config, true);
	channel_config_set_write_increment(&config, false);
	channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
	dma_channel_set_read_addr(dmaChan, data, false);
	dma_channel_set_write_addr(dmaChan, &hw->txf[stateMachineNumber], false);
	dma_channel_set_trans_count(dmaChan, numLeds, true);
}

void WS2812::SetColour(uint32_t colour, unsigned int numLeds) noexcept
{
	dma_channel_config config = dma_channel_get_default_config(dmaChan);
	channel_config_set_read_increment(&config, false);
	channel_config_set_write_increment(&config, false);
	channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
	dma_channel_set_read_addr(dmaChan, &colour, false);
	dma_channel_set_write_addr(dmaChan, &hw->txf[stateMachineNumber], false);
	dma_channel_set_trans_count(dmaChan, numLeds, true);
}

// End
