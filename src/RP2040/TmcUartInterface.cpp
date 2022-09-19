/*
 * TMC22xxUartInterface.cpp
 *
 *  Created on: 18 Sept 2022
 *      Author: David
 */

#include "TmcUartInterface.h"

#if RP2040

#include <hardware/pio.h>

// PIO program to wait for IRQ 4, then start receiving bytes, packing them into 32-bit words, and pushing them through the FIFO

#define uart_rx_wrap_target 5
#define uart_rx_wrap 14

static const uint16_t uart_rx_program_instructions[] =
{
    0x27c4, //  0: wait   1 irq, 4               [7]
    0xe020, //  1: set    x, 0
    0xa0c1, //  2: mov    isr, x
    0xe044, //  3: set    y, 4
    0xc044, //  4: irq    clear 4
            //     .wrap_target
    0x2020, //  5: wait   0 pin, 0
    0xea27, //  6: set    x, 7                   [10]
    0x4001, //  7: in     pins, 1
    0x0647, //  8: jmp    x--, 7                 [6]
    0x00cd, //  9: jmp    pin, 13
    0xc000, // 10: irq    nowait 0
    0x20a0, // 11: wait   1 pin, 0
    0x0005, // 12: jmp    5
    0x0085, // 13: jmp    y--, 5
    0x8020, // 14: push   block
            //     .wrap
};

static const struct pio_program uart_rx_program =
{
    .instructions = uart_rx_program_instructions,
    .length = 15,
    .origin = -1,
};

static inline pio_sm_config uart_rx_program_get_default_config(uint offset) noexcept
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + uart_rx_wrap_target, offset + uart_rx_wrap);
    return c;
}

// PIO program to read the number of 32-bit words to be sent from the FIFO, then send that number of words to the UART
// split into 8-bit characters. When the send is complete, IRQ4 is set and then the pin is switched to being an input.

#define uart_tx_wrap_target 0
#define uart_tx_wrap 14

static const uint16_t uart_tx_program_instructions[] =
{
            //     .wrap_target
    0xe080, //  0: set    pindirs, 0
    0x80a0, //  1: pull   block
    0xa041, //  2: mov    y, x
    0x0085, //  3: jmp    y--, 5
    0x0000, //  4: jmp    0
    0xe001, //  5: set    pins, 1
    0xe781, //  6: set    pindirs, 1             [7]
    0x80a0, //  7: pull   block
    0xf727, //  8: set    x, 7            side 0 [7]
    0x6001, //  9: out    pins, 1
    0x0649, // 10: jmp    x--, 9                 [6]
    0xe401, // 11: set    pins, 1                [4]
    0x00ef, // 12: jmp    !osre, 15
    0x0087, // 13: jmp    y--, 7
    0xc004, // 14: irq    nowait 4
    0x0000, // 15: jmp    0
    0x0008, // 16: jmp    8
             //     .wrap
};

static const struct pio_program uart_tx_program =
{
    .instructions = uart_tx_program_instructions,
    .length = 16,
    .origin = -1,
};

static inline pio_sm_config uart_tx_program_get_default_config(uint offset) noexcept
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + uart_tx_wrap_target, offset + uart_tx_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}

static inline void uart_tx_program_init(PIO pio, uint sm, uint offset, uint pin_tx, uint baud) noexcept
{
	// Tell PIO to initially drive output-high on the selected pin, then map PIO onto that pin with the IO muxes.
	pio_sm_set_pins_with_mask(pio, sm, 1u << pin_tx, 1u << pin_tx);
	pio_sm_set_pindirs_with_mask(pio, sm, 1u << pin_tx, 1u << pin_tx);
	pio_gpio_init(pio, pin_tx);
	pio_sm_config c = uart_tx_program_get_default_config(offset);
	// OUT shifts to right, no autopull
	sm_config_set_out_shift(&c, true, false, 32);
	// We are mapping both OUT and side-set to the same pin, because sometimes
	// we need to assert user data onto the pin (with OUT) and sometimes
	// assert constant values (start/stop bit)
	sm_config_set_out_pins(&c, pin_tx, 1);
	sm_config_set_sideset_pins(&c, pin_tx);
	// We only need TX, so get an 8-deep FIFO!
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
	// SM transmits 1 bit per 8 execution cycles.
	float div = (float)SystemCoreClockFreq / (8 * baud);
	sm_config_set_clkdiv(&c, div);
	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);
}

// Public functions
void TmcUartInterface::Init(Pin uartPin, uint32_t baudRate) noexcept
{
	//TODO
}

void TmcUartInterface::ResetUart() noexcept
{
	//TODO
}

void TmcUartInterface::ResetDMA() noexcept
{
	//TODO
}

void TmcUartInterface::SetTxData(const volatile uint32_t* data, unsigned int numWords) noexcept
{
	//TODO
}

void TmcUartInterface::SetRxData(volatile uint32_t* data, unsigned int numWords) noexcept
{
	//TODO
}

void TmcUartInterface::StartTransfer(TmcUartCallbackFn callbackFn) noexcept
{
	//TODO
}

void TmcUartInterface::DisableCompletedCallback() noexcept
{
	//TODO
}

void TmcUartInterface::AbortTransfer() noexcept
{
	//TODO
}

#endif

// End
