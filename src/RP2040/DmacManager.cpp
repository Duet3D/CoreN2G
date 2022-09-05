/*
 * Dmac.cpp
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#include <CoreIO.h>

#if !RP2040
# error Wrong processor
#endif

#include <DmacManager.h>
#include <RTOSIface/RTOSIface.h>
#include <hardware/regs/dma.h>
#include <hardware/structs/dma.h>
#include <hardware/regs/intctrl.h>
#include <RP2040.h>

constexpr NvicPriority TempNvicPriorityDMA = 2;			// temporary DMA interrupt priority, low enough to allow FreeRTOS system calls

// Array containing callbacks for DMAC channels
static DmaCallbackFunction dmaChannelCallbackFunctions[NumDmaChannelsSupported] = { 0 };
static CallbackParameter callbackParams[NumDmaChannelsSupported];

extern "C" void DMAC_0_Handler() noexcept;

// Initialize the DMA controller.
// We route all DMA complete interrupts through the first of the two available DMA controller interrupts. This is reasonable if only one core uses the DMAC.
void DmacManager::Init() noexcept
{
	NVIC_DisableIRQ(DMA_IRQ_0_IRQn);
	NVIC_ClearPendingIRQ(DMA_IRQ_0_IRQn);
	NVIC_SetPriority(DMA_IRQ_0_IRQn, TempNvicPriorityDMA);
	irq_set_exclusive_handler(DMA_IRQ_0, DMAC_0_Handler);
	NVIC_EnableIRQ(DMA_IRQ_0_IRQn);
}

void DmacManager::SetBtctrl(const uint8_t channel, const uint32_t val) noexcept
{
	dma_hw->ch[channel].al1_ctrl = val;
}

void DmacManager::SetDestinationAddress(const uint8_t channel, volatile void *const dst) noexcept
{
	dma_hw->ch[channel].write_addr = reinterpret_cast<uint32_t>(dst);
}

void DmacManager::SetSourceAddress(const uint8_t channel, const volatile void *const src) noexcept
{
	dma_hw->ch[channel].read_addr = reinterpret_cast<uint32_t>(src);
}

void DmacManager::SetDataLength(const uint8_t channel, const uint32_t amount) noexcept
{
	dma_hw->ch[channel].transfer_count = amount;
}

void DmacManager::SetTriggerSource(uint8_t channel, DmaTrigSource source) noexcept
{
	hw_write_masked(&(dma_hw->ch[channel].al1_ctrl), (uint32_t)source << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB, DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS);
}

void DmacManager::EnableChannel(const uint8_t channel, DmaPriority priority) noexcept
{
	dma_channel_hw_t *const hw = &dma_hw->ch[channel];
	if (priority)
	{
		hw_set_bits(&(hw->ctrl_trig), DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS | DMA_CH0_CTRL_TRIG_EN_BITS);
	}
	else
	{
		hw_clear_bits(&(hw->al1_ctrl), DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS);
		hw_set_bits(&(hw->ctrl_trig), DMA_CH0_CTRL_TRIG_EN_BITS);
	}
}

// Disable a channel. Also clears its status and disables its interrupts.
// On the SAME5x it is sometimes impossible to disable a channel. So we now return true if disabling it succeeded, false it it is still enabled.
bool DmacManager::DisableChannel(const uint8_t channel) noexcept
{
	hw_clear_bits(&(dma_hw->ch[channel].al1_ctrl), DMA_CH0_CTRL_TRIG_EN_BITS);
	return true;
}

void DmacManager::SetInterruptCallback(uint8_t channel, DmaCallbackFunction fn, CallbackParameter param) noexcept
{
	AtomicCriticalSectionLocker lock;
	dmaChannelCallbackFunctions[channel] = fn;
	callbackParams[channel] = param;
}

void DmacManager::EnableCompletedInterrupt(const uint8_t channel) noexcept
{
	hw_set_bits(&(dma_hw->inte0), 1u << channel);
}

void DmacManager::DisableCompletedInterrupt(const uint8_t channel) noexcept
{
	hw_clear_bits(&(dma_hw->inte0), 1u << channel);
}

uint32_t DmacManager::GetAndClearChannelStatus(uint8_t channel) noexcept
{
	dma_channel_hw_t& hw = dma_hw->ch[channel];
	const uint32_t ret = hw.al1_ctrl & (DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS | DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS | DMA_CH0_CTRL_TRIG_BUSY_BITS);
	hw_set_bits(&(hw.al1_ctrl), DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS | DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS);
	return ret;
}

extern "C" void DMAC_0_Handler() noexcept
{
	uint32_t status = dma_hw->ints0;
	dma_hw->ints0 = status;
	status &= (1ul << NumDmaChannelsSupported) - 1;
	unsigned int chan = 0;
	while (status != 0)
	{
		if ((status & 1ul) && dmaChannelCallbackFunctions[chan] != nullptr)
		{
			const uint32_t stat = DmacManager::GetAndClearChannelStatus(chan);
			DmaCallbackReason reason = (stat == 0) ? DmaCallbackReason::complete
										: (stat == DMA_CH0_CTRL_TRIG_BUSY_BITS) ? DmaCallbackReason::none
											: (stat & DMA_CH0_CTRL_TRIG_BUSY_BITS) ? DmaCallbackReason::error
												: DmaCallbackReason::completeAndError;
			dmaChannelCallbackFunctions[chan](callbackParams[chan], reason);		//TODO read error status
		}
		status >>= 1;
		++chan;
	}
}

// End
