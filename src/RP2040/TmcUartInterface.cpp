/*
 * TMC22xxUartInterface.cpp
 *
 *  Created on: 18 Sept 2022
 *      Author: David
 */

#include "TmcUartInterface.h"

#if RP2040

#include <hardware/pio.h>
#include <hardware/structs/resets.h>			// for RESETS_RESET_PIO0_BITS
#include <hardware/structs/pio.h>				// for pio_hw_t
#include "PIOassignments.h"

constexpr unsigned int TmcClocksPerBit = 8;

static PIO pio_hw;

// rp2040 helper function to clear a hardware reset bit
static void rp2040_clear_reset(uint32_t reset_bit)
{
    if (resets_hw->reset & reset_bit)
    {
        hw_clear_bits(&resets_hw->reset, reset_bit);
        while (!(resets_hw->reset_done & reset_bit)) { }
    }
}

// PIO programs. See file TmcUartInterface.pio for the PIOASM source code.

// ------ //
// TMC_Tx //
// ------ //

#define TMC_Tx_wrap_target 0
#define TMC_Tx_wrap 6

static const uint16_t TMC_Tx_program_instructions[] = {
            //     .wrap_target
    0xe601, //  0: set    pins, 1                [6]
    0xe080, //  1: set    pindirs, 0
    0x80a0, //  2: pull   block
    0xe781, //  3: set    pindirs, 1             [7]
    0xf727, //  4: set    x, 7            side 0 [7]
    0x6001, //  5: out    pins, 1
    0x0645, //  6: jmp    x--, 5                 [6]
            //     .wrap
};

static const struct pio_program TMC_Tx_program = {
    .instructions = TMC_Tx_program_instructions,
    .length = 7,
    .origin = -1,
};

static inline pio_sm_config TMC_Tx_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + TMC_Tx_wrap_target, offset + TMC_Tx_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}

// ------ //
// TMC_Rx //
// ------ //

#define TMC_Rx_wrap_target 0
#define TMC_Rx_wrap 8

static const uint16_t TMC_Rx_program_instructions[] = {
            //     .wrap_target
    0x2020, //  0: wait   0 pin, 0
    0xea27, //  1: set    x, 7                   [10]
    0x4001, //  2: in     pins, 1
    0x0642, //  3: jmp    x--, 2                 [6]
    0x00c8, //  4: jmp    pin, 8
    0xc014, //  5: irq    nowait 4 rel
    0x20a0, //  6: wait   1 pin, 0
    0x0000, //  7: jmp    0
    0x8000, //  8: push   noblock
            //     .wrap
};

static const struct pio_program TMC_Rx_program = {
    .instructions = TMC_Rx_program_instructions,
    .length = 9,
    .origin = -1,
};

static inline pio_sm_config TMC_Rx_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + TMC_Rx_wrap_target, offset + TMC_Rx_wrap);
    return c;
}

static inline void uart_tx_program_init(PIO pio, uint sm, uint offset, uint pin_tx, uint baud) noexcept
{
	// Tell PIO to initially drive output-high on the selected pin, then map PIO onto that pin with the IO muxes.
	pio_sm_set_pins_with_mask(pio, sm, 1u << pin_tx, 1u << pin_tx);
	pio_sm_set_pindirs_with_mask(pio, sm, 1u << pin_tx, 1u << pin_tx);
	pio_gpio_init(pio, pin_tx);
	pio_sm_config c = TMC_Tx_program_get_default_config(offset);
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
	// Configure PIO clock
	pio_hw = (TmcUartPioNumber) ? pio1_hw : pio0_hw;
	const uint32_t rb = (TmcUartPioNumber) ? RESETS_RESET_PIO1_BITS : RESETS_RESET_PIO0_BITS;
	rp2040_clear_reset(rb);

	// Setup and sync pio state machine clocks
	const uint32_t div = (256 / TmcClocksPerBit) * SystemCoreClockFreq / baudRate;
	pio_hw->sm[TmcUartTxStateMachineNumber].clkdiv = div << PIO_SM0_CLKDIV_FRAC_LSB;
	pio_hw->sm[TmcUartRxStateMachineNumber].clkdiv = div << PIO_SM0_CLKDIV_FRAC_LSB;

    // Reset state machines
    pio_hw->ctrl = PIO_CTRL_SM_RESTART_BITS | PIO_CTRL_CLKDIV_RESTART_BITS;
    pio_hw->fdebug = 0xffffffff;

#if 0	// the following code is incomplete
   // Load pio program
    for (unsigned int i = 0; i < ARRAY_SIZE(UartProgramInstructions); i++)
    {
    	pio_hw->instr_mem[i] = UartProgramInstructions[i];
    }

    // Set initial state machine state
    pio_sync_setup();
    pio_rx_setup();
    pio_match_setup();
    pio_tx_setup();

    // Start state machines
    pio_hw->ctrl = 0x07 << PIO_CTRL_SM_ENABLE_LSB;

	// Map Rx/Tx gpios
	const GpioPinFunction pioFunc = (TmcUartPioNumber) ? GpioPinFunction::Pio1 : GpioPinFunction::Pio0;
	SetPinFunction(regs->rxPin, pioFunc);
	SetPinFunction(regs->txPin, pioFunc);

	const IRQn_Type irqn = (TmcUartPioNumber) ? PIO1_IRQ_0_IRQn : PIO0_IRQ_0_IRQn;
	NVIC_DisableIRQ(irqn);
	NVIC_ClearPendingIRQ(irqn);
	NVIC_SetPriority(irqn, 1);
	irq_set_exclusive_handler ((unsigned int)irqn, PIO_isr);
	NVIC_EnableIRQ(irqn);

	//TODO
#endif
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
