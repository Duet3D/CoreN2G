/*
 * TMC22xxUartInterface.cpp
 *
 *  Created on: 18 Sept 2022
 *      Author: David
 */

#include "TmcUartInterface.h"

#if RP2040

#include <hardware/pio.h>
#include <hardware/dma.h>
#include <hardware/structs/resets.h>			// for RESETS_RESET_PIO0_BITS
#include <hardware/structs/pio.h>				// for pio_hw_t
#include "PIOassignments.h"

constexpr unsigned int TmcClocksPerBit = 8;

static PIO pio_hw;
static uint8_t firstDmaChan;					// the first of two DMA channels
static unsigned int txProgramOffset, rxProgramOffset;
static uint8_t tmcTxStateMachineNumber, tmcRxStateMachineNumber;

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

static const struct pio_program TMC_Tx_program =
{
    .instructions = TMC_Tx_program_instructions,
    .length = 7,
    .origin = -1,
};

static inline pio_sm_config TMC_Tx_program_get_default_config(uint offset)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + TMC_Tx_wrap_target, offset + TMC_Tx_wrap);
    sm_config_set_sideset(&c, 2, true, false);
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
	// We are mapping both OUT and side-set to the same pin, because sometimes we need to assert user data onto the pin (with OUT) and sometimes assert constant values (start/stop bit)
	sm_config_set_out_pins(&c, pin_tx, 1);
	sm_config_set_sideset_pins(&c, pin_tx);
	// We only need TX, so get an 8-deep FIFO!
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
	// SM transmits 1 bit per 8 execution cycles.
	const float div = (float)SystemCoreClockFreq / (8 * baud);
	sm_config_set_clkdiv(&c, div);
	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);
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

static inline void uart_rx_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud)
{
	pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
	pio_gpio_init(pio, pin);
	gpio_pull_up(pin);

	pio_sm_config c = TMC_Rx_program_get_default_config(offset);
	sm_config_set_in_pins(&c, pin); // for WAIT, IN
	sm_config_set_jmp_pin(&c, pin); // for JMP
	// Shift to right, autopush disabled
	sm_config_set_in_shift(&c, true, false, 32);
	// Deeper FIFO as we're not doing any TX
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
	// SM transmits 1 bit per 8 execution cycles.
	float div = (float)SystemCoreClockFreq / (8 * baud);
	sm_config_set_clkdiv(&c, div);

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);
}

// Public functions
// Initialise this interface. This must be called exactly once.
void TmcUartInterface::Init(Pin uartPin, uint32_t baudRate, uint8_t p_firstDmaChan) noexcept
{
	firstDmaChan = p_firstDmaChan;

	// Configure PIO clock
	pio_hw = (TmcUartPioNumber) ? pio1_hw : pio0_hw;
	const uint32_t rb = (TmcUartPioNumber) ? RESETS_RESET_PIO1_BITS : RESETS_RESET_PIO0_BITS;
	rp2040_clear_reset(rb);

    txProgramOffset = pio_add_program(pio_hw, &TMC_Tx_program);
    rxProgramOffset = pio_add_program(pio_hw, &TMC_Rx_program);
    tmcTxStateMachineNumber = pio_claim_unused_sm(pio_hw, true);
    tmcRxStateMachineNumber = pio_claim_unused_sm(pio_hw, true);

    uart_tx_program_init(pio_hw, tmcTxStateMachineNumber, txProgramOffset, uartPin, baudRate);
    uart_rx_program_init(pio_hw, tmcRxStateMachineNumber, rxProgramOffset, uartPin, baudRate);

#if 0	// we only need to monitor the interrupt if we want to detect receive framing errors
	const IRQn_Type irqn = (TmcUartPioNumber) ? PIO1_IRQ_0_IRQn : PIO0_IRQ_0_IRQn;
	NVIC_DisableIRQ(irqn);
	NVIC_ClearPendingIRQ(irqn);
	NVIC_SetPriority(irqn, 1);
	irq_set_exclusive_handler ((unsigned int)irqn, PIO_isr);
	NVIC_EnableIRQ(irqn);
#endif
}

// This is called before every transaction, so don't completely initialise everything
void TmcUartInterface::ResetUart() noexcept
{
	pio_set_sm_mask_enabled(pio_hw, (1ul << tmcTxStateMachineNumber) | (1ul << tmcRxStateMachineNumber), false);	// disable Tx and Rx state machines
	pio_sm_clear_fifos(pio_hw, tmcTxStateMachineNumber);															// clear Tx fifo
	pio_sm_clear_fifos(pio_hw, tmcRxStateMachineNumber);															// clear Rx fifo
	pio_restart_sm_mask(pio_hw, (1ul << tmcTxStateMachineNumber) | (1ul << tmcRxStateMachineNumber));				// reset their states
	pio_sm_exec(pio_hw, tmcTxStateMachineNumber, txProgramOffset);													// force Tx state machine to start
	pio_sm_set_enabled(pio_hw, tmcTxStateMachineNumber, true);														// enable Tx state machine
	pio_sm_exec(pio_hw, tmcRxStateMachineNumber, rxProgramOffset);													// force Rx state machine to start
	pio_sm_set_enabled(pio_hw, tmcRxStateMachineNumber, true);														// enable Rx state machine
}

// This is called to stop any pending DMA ready for reprogramming the DMAC
void TmcUartInterface::ResetDMA() noexcept
{
	dma_channel_abort(firstDmaChan);
	dma_channel_abort(firstDmaChan + 1);
}

// Set up the data to send
void TmcUartInterface::SetTxData(const volatile uint8_t* data, unsigned int numBytes) noexcept
{
	dma_channel_config config = dma_channel_get_default_config(firstDmaChan);
	channel_config_set_read_increment(&config, true);
	channel_config_set_write_increment(&config, false);
	channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
	dma_channel_set_read_addr(firstDmaChan, data, false);
	dma_channel_set_write_addr(firstDmaChan, &pio_hw->txf[tmcTxStateMachineNumber], false);
	dma_channel_set_trans_count(firstDmaChan, numBytes, false);
}

// Set up the data to receive
void TmcUartInterface::SetRxData(volatile uint8_t* data, unsigned int numBytes) noexcept
{
	dma_channel_config config = dma_channel_get_default_config(firstDmaChan + 1);
	channel_config_set_read_increment(&config, false);
	channel_config_set_write_increment(&config, true);
	channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
	dma_channel_set_read_addr(firstDmaChan + 1, &pio_hw->rxf[tmcRxStateMachineNumber], false);
	dma_channel_set_write_addr(firstDmaChan + 1, data, false);
	dma_channel_set_trans_count(firstDmaChan + 1, numBytes, false);
}

// Start the send and receive and enable the DMA receive complete interrupt
void TmcUartInterface::StartTransfer(TmcUartCallbackFn callbackFn) noexcept
{
	DmacManager::SetInterruptCallback(firstDmaChan + 1, callbackFn, CallbackParameter(0));
	DmacManager::EnableCompletedInterrupt(firstDmaChan + 1);
	dma_channel_start(firstDmaChan + 1);
	dma_channel_start(firstDmaChan);
}

// Disable the DMA complete interrupt
void TmcUartInterface::DisableCompletedCallback() noexcept
{
	DmacManager::DisableCompletedInterrupt(firstDmaChan + 1);
}

void TmcUartInterface::AbortTransfer() noexcept
{
	dma_channel_abort(firstDmaChan);
	dma_channel_abort(firstDmaChan + 1);
	pio_set_sm_mask_enabled(pio_hw, (1ul << tmcTxStateMachineNumber) | (1ul << tmcRxStateMachineNumber), false);	// disable Tx and Rx state machines
}

#endif

// End
