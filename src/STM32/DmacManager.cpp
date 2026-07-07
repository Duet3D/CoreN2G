/*
 * Dmac.cpp
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#include <CoreIO.h>

#if STM32H5
# include <stm32h5xx_hal_dma.h>
#elif STM32H7
# include <stm32h7xx_hal_dma.h>
#else
# error Unsupported processor
#endif

#include <DmacManager.h>
#include <RTOSIface/RTOSIface.h>
#include <Cache.h>

constexpr NvicPriority TempNvicPriorityDMA = 2;			// temporary DMA interrupt priority, low enough to allow FreeRTOS system calls

// Array containing callbacks for DMAC channels
static DmaCallbackFunction dmaChannelCallbackFunctions[NumDmaChannelsSupported];
static CallbackParameter callbackParams[NumDmaChannelsSupported];

static DMA_Channel_TypeDef *GetChannel(unsigned int channel) noexcept
{
	return  (DMA_Channel_TypeDef*)
				((channel < 8) ? (GPDMA1_Channel0_BASE + 0x80 * channel) : (GPDMA2_Channel0_BASE + 0x80 * (channel - 8)));
}

// Initialize the DMA controller
void DmacManager::Init() noexcept
{
	//TODO

	// Enable the DMA channel interrupts
#if STM32H5
	// STM32 DMACs have 8 contiguous IRQ numbers, one for each channel
	for (unsigned int i = 0; i < 8; i++)
	{
		GetChannel(i)->CCR |= DMA_CCR_RESET;
		NVIC_DisableIRQ((IRQn)(GPDMA1_Channel0_IRQn + i));
		NVIC_ClearPendingIRQ((IRQn)(GPDMA1_Channel0_IRQn + i));
		NVIC_SetPriority((IRQn)(GPDMA1_Channel0_IRQn + i), TempNvicPriorityDMA);
		NVIC_EnableIRQ((IRQn)(GPDMA1_Channel0_IRQn + i));

		GetChannel(i + 8)->CCR |= DMA_CCR_RESET;
		NVIC_DisableIRQ((IRQn)(GPDMA2_Channel0_IRQn + i));
		NVIC_ClearPendingIRQ((IRQn)(GPDMA2_Channel0_IRQn + i));
		NVIC_SetPriority((IRQn)(GPDMA2_Channel0_IRQn + i), TempNvicPriorityDMA);
		NVIC_EnableIRQ((IRQn)(GPDMA2_Channel0_IRQn + i));
	}
#elif STM32H7
	//TODO
#else
# error Unsupported processor
#endif
}

void DmacManager::SetBtctrl(const uint8_t channel, const uint32_t val) noexcept
{
	DMA_Channel_TypeDef *const chan = GetChannel(channel);
	chan->CTR1 = (chan->CTR1 & ~(  DMA_CTR1_DAP | DMA_CTR1_DHX | DMA_CTR1_DBX | DMA_CTR1_DBL_1 | DMA_CTR1_DINC | DMA_CTR1_DDW_LOG2
								 | DMA_CTR1_SAP | DMA_CTR1_SBX | DMA_CTR1_PAM | DMA_CTR1_SBL_1 | DMA_CTR1_SINC | DMA_CTR1_SDW_LOG2
								)
				 ) | val;
}

void DmacManager::SetDestinationAddress(const uint8_t channel, volatile void *const dst) noexcept
{
	GetChannel(channel)->CDAR = (uint32_t)dst;
}

void DmacManager::SetSourceAddress(const uint8_t channel, const volatile void *const src) noexcept
{
	GetChannel(channel)->CSAR = (uint32_t)src;
}

// Caution: SetDataLength must be called up AFTER setting up source sand destination addresses!
void DmacManager::SetDataLength(const uint8_t channel, const uint32_t amount) noexcept
{
	GetChannel(channel)->CBR1 = amount;
}

void DmacManager::SetTriggerSource(uint8_t channel, DmaTrigSource source) noexcept
{
	DMA_Channel_TypeDef *const chan = GetChannel(channel);
	chan->CTR2 = (chan->CTR2 & ~(  DMA_CTR2_TCEM | DMA_CTR2_TRIGPOL | DMA_CTR2_TRIGSEL | DMA_CTR2_TRIGM
								 | DMA_CTR2_PFREQ |  DMA_CTR2_BREQ |  DMA_CTR2_SWREQ | DMA_CTR2_REQSEL
								)
				 ) | (uint32_t)source;
}

void DmacManager::SetTriggerSourceSpiTx(DmaChannel channel, uint8_t spiNumber) noexcept
{
	static constexpr DmaTrigSource SpiTxTrigSources[] =
		{ 	DmaTrigSource::spi1_tx, DmaTrigSource::spi2_tx, DmaTrigSource::spi3_tx, DmaTrigSource::spi4_tx,
#if STM32H7
			DmaTrigSource::spi5_tx,
#endif
		};
	SetTriggerSource(channel, SpiTxTrigSources[spiNumber]);
}

void DmacManager::SetTriggerSourceSpiRx(DmaChannel channel, uint8_t spiNumber) noexcept
{
	static constexpr DmaTrigSource SpiRxTrigSources[] =
		{ 	DmaTrigSource::spi1_rx, DmaTrigSource::spi2_rx, DmaTrigSource::spi3_rx, DmaTrigSource::spi4_rx,
#if STM32H7
			DmaTrigSource::spi5_rx,
#endif
		};
	SetTriggerSource(channel, SpiRxTrigSources[spiNumber]);
}

void DmacManager::EnableChannel(const uint8_t channel, DmaPriority priority) noexcept
{
	DMA_Channel_TypeDef *const chan = GetChannel(channel);
	chan->CCR = (chan->CCR & ~(DMA_CCR_PRIO | DMA_CCR_RESET | DMA_CCR_SUSP)) | ((priority & 3) << DMA_CCR_PRIO_Pos) | DMA_CCR_EN;
}

// Disable a channel. Also clears its status and disables its interrupts.
bool DmacManager::DisableChannel(const uint8_t channel) noexcept
{
	GetChannel(channel)->CCR &= ~DMA_CCR_EN;
	return true;
}

bool DmacManager::SuspendChannel(DmaChannel channel) noexcept
{
	GetChannel(channel)->CCR |= DMA_CCR_SUSP;
	return true;
}

void DmacManager::ResumeChannel(DmaChannel channel) noexcept
{
	GetChannel(channel)->CCR &= ~DMA_CCR_SUSP;
}

void DmacManager::SetInterruptCallback(uint8_t channel, DmaCallbackFunction fn, CallbackParameter param) noexcept
{
	AtomicCriticalSectionLocker lock;
	dmaChannelCallbackFunctions[channel] = fn;
	callbackParams[channel] = param;
}

void DmacManager::EnableCompletedInterrupt(const uint8_t channel) noexcept
{
	GetChannel(channel)->CCR |= DMA_CCR_TCIE;
}

void DmacManager::DisableCompletedInterrupt(const uint8_t channel) noexcept
{
	GetChannel(channel)->CCR &= ~DMA_CCR_TCIE;
}

uint32_t DmacManager::GetAndClearChannelStatus(uint8_t channel) noexcept
{
	DMA_Channel_TypeDef *const chan = GetChannel(channel);
	const uint32_t status = chan->CSR;
	chan->CFCR |= (status & (DMA_CSR_TOF | DMA_CSR_SUSPF | DMA_CSR_USEF | DMA_CSR_ULEF | DMA_CSR_DTEF | DMA_CSR_HTF | DMA_CSR_TCF));
	return status;
}

// Internal DMAC interrupt handler
static inline void CommonDmacHandler(uint8_t channel) noexcept
{
	DMA_Channel_TypeDef *const chan = GetChannel(channel);
	const uint32_t intflag = chan->CSR & chan->CCR;
	if (intflag != 0)					// should always be true
	{
		chan->CFCR |= intflag;
		const DmaCallbackFunction fn = dmaChannelCallbackFunctions[channel];
		if (fn != nullptr)
		{
			fn(callbackParams[channel], (DmaCallbackReason)intflag);
		}
	}
}

extern "C" void GPDMA1_Channel0_IRQHandler() noexcept
{
	CommonDmacHandler(0);
}

extern "C" void GPDMA1_Channel1_IRQHandler() noexcept
{
	CommonDmacHandler(1);
}

extern "C" void GPDMA1_Channel2_IRQHandler() noexcept
{
	CommonDmacHandler(2);
}

extern "C" void GPDMA1_Channel3_IRQHandler() noexcept
{
	CommonDmacHandler(3);
}

extern "C" void GPDMA1_Channel4_IRQHandler() noexcept
{
	CommonDmacHandler(4);
}

extern "C" void GPDMA1_Channel5_IRQHandler() noexcept
{
	CommonDmacHandler(5);
}

extern "C" void GPDMA1_Channel6_IRQHandler() noexcept
{
	CommonDmacHandler(6);
}

extern "C" void GPDMA1_Channel7_IRQHandler() noexcept
{
	CommonDmacHandler(7);
}

extern "C" void GPDMA2_Channel0_IRQHandler() noexcept
{
	CommonDmacHandler(8);
}

extern "C" void GPDMA2_Channel1_IRQHandler() noexcept
{
	CommonDmacHandler(9);
}

extern "C" void GPDMA2_Channel2_IRQHandler() noexcept
{
	CommonDmacHandler(10);
}

extern "C" void GPDMA2_Channel3_IRQHandler() noexcept
{
	CommonDmacHandler(11);
}

extern "C" void GPDMA2_Channel4_IRQHandler() noexcept
{
	CommonDmacHandler(12);
}

extern "C" void GPDMA2_Channel5_IRQHandler() noexcept
{
	CommonDmacHandler(13);
}

extern "C" void GPDMA2_Channel6_IRQHandler() noexcept
{
	CommonDmacHandler(14);
}

extern "C" void GPDMA2_Channel7_IRQHandler() noexcept
{
	CommonDmacHandler(15);
}

// End
