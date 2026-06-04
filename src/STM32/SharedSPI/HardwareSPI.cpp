//Hardware SPI
#include "HardwareSPI.h"

#ifdef RTOS
#include <RTOSIface/RTOSIface.h>
#include <CoreNotifyIndices.h>
#endif
#include <DMA.h>
#include "spi_com.h"
#include "Cache.h"

/*
DMA Notes
The original version of this code only used DMA for all SPI devices. However during testing
I hit a problem when sharing the DMA2 unit between SPI1 and the software UART code. What appeared
to be happening was that write operations took places to the wrong GPIO pins, but this only
seemed to happen when the DMA2 unit was also in use by the SD access code (which uses SPI1).
The following Errata document describes problems which may be related to this issue (even though
we are not using the FiFO in this case):
https://www.st.com/resource/en/errata_sheet/dm00037591-stm32f405-407xx-and-stm32f415-417xx-device-limitations-stmicroelectronics.pdf

For now the solution I've choosen is to not use DMA for SPI1 (as it is not possible to use DMA1 with
SPI1 and DMA1 is not able to access GPIO memory). This seems to work fine and since the SD card is
only used in a synchronous manner did not require any code to be restructured. It also seems to
be faster (I suspect because the SD card access code uses many short SPI operations and the DMA
setup is relatively large).

Andy - 6/8/2020

Some notes on SPI I/O operation
The STM32 supports 3 forms of SPI I/O: polled, interrupt and dma. Currently polled operations are no longer used.

polled: can use any memory address (as the cpu does the memory read/write operation), but has high cpu usage especially
on large long operations. All devices support polled operations.

interrupt: Can access all memory areas, has lower cpu usage for large, slow operations, but may have higher cpu usage
for fast/large operations. The use of interrupts at the end of the operation makes it easier to ensure that there
is a minimal gap between the end of the operation and cs being released (this is important on the MCP151xFD SPI CAN device,
due to errors in the chip). This mode is now supported by all devices, we use a customized HAL to provide sending dummy
0xff values when performing read operations. We have also optimised the code on the F4 version. Note that interrupt based
I/O is not really fast enough currently for use when talking to the RRF WiFi interface as this operates at 20MHz and higher.

dma: This mode may not be able to access all memory areas, but it has the lowest cpu overhead for large operations.
It is the prefered operating mode when available. If a memory areas can not be accessed by DMA or if the operation is
very short we fall back to using an iterrupt based version.

Some notes on SPI optimisations 21/10/2025
This file now contains code that has been extracted from the HAL files for F4 and H7 systems and optimised for use with RRF.
This code is used in conjunction with the standard HAL routines to provide interrupt and DMA operations, polled mode
is no longer used by RRF.

The primary optimisation is to remove overhead checks/locks etc. that is not used by RRF. In additon the startTransfer*
functions have been modified such that passing a nullptr for the write buffer will feed 0xff to output and that passing a
nullptr for the read buffer will simply discard data from input. In addition on H7 based systems we run in continuous mode
(see notes below) that reduces the SPI latency.

On all systems we do not use DMA for short transactions as in this case using interrupt based I/O is faster and has a lower
latency. This is of particular use for the TMC SPI interface as transactions are short (5 bytes).

*/
#if USE_SSP1 || USE_SSP2 || USE_SSP3 || USE_SSP4 || USE_SSP5 || USE_SSP6
// Create SPI devices the actual configuration is set later
#if STM32H7
// On the H7 we need to make sure that and dma address is within a none cached memory area
extern uint8_t _nocache_ram_start;
extern uint8_t _nocache_ram_end;
extern uint8_t _nocache2_ram_start;
extern uint8_t _nocache2_ram_end;
#define CAN_USE_DMA(ptr, len) ((ptr) == nullptr || (((const char *)(ptr) >= (const char *)&_nocache_ram_start) && ((const char *)(ptr) + (len) < (const char *)&_nocache_ram_end)) || (((const char *)(ptr) >= (const char *)&_nocache2_ram_start) && ((const char *)(ptr) + (len) < (const char *)&_nocache2_ram_end)))

// Create SPI devices the actual configuration is set later
#if USE_SSP1
HardwareSPI HardwareSPI::SSP1(SPI1, SPI1_IRQn, DMA1_Stream6, DMA_REQUEST_SPI1_RX, DMA1_Stream6_IRQn, DMA1_Stream7, DMA_REQUEST_SPI1_TX, DMA1_Stream7_IRQn);
#endif
#if USE_SSP2
HardwareSPI HardwareSPI::SSP2(SPI2, SPI2_IRQn, DMA1_Stream3, DMA_REQUEST_SPI2_RX, DMA1_Stream3_IRQn, DMA1_Stream4, DMA_REQUEST_SPI2_TX, DMA1_Stream4_IRQn);
#endif
#if USE_SSP3
HardwareSPI HardwareSPI::SSP3(SPI3, SPI3_IRQn, DMA1_Stream0, DMA_REQUEST_SPI3_RX, DMA1_Stream0_IRQn, DMA1_Stream5, DMA_REQUEST_SPI3_TX, DMA1_Stream5_IRQn);
#endif
#if USE_SSP4
HardwareSPI HardwareSPI::SSP4(SPI4, SPI4_IRQn, DMA1_Stream1, DMA_REQUEST_SPI4_RX, DMA1_Stream1_IRQn, DMA1_Stream2, DMA_REQUEST_SPI4_TX, DMA1_Stream2_IRQn);
#endif
#if USE_SSP5
HardwareSPI HardwareSPI::SSP5(SPI5, SPI5_IRQn, DMA2_Stream1, DMA_REQUEST_SPI5_RX, DMA2_Stream1_IRQn, DMA2_Stream2, DMA_REQUEST_SPI5_TX, DMA2_Stream2_IRQn);
#endif
#if USE_SSP6
// SPI6 only has DMA via BDMA, currently we don't use that at all and have no easy way to test it...
HardwareSPI HardwareSPI::SSP6(SPI6, SPI6_IRQn);
#endif
#else
extern uint8_t _sccmram;
extern uint8_t _ccmramend;
#define CAN_USE_DMA(ptr, len) ((ptr) == nullptr || !(ptr >= &_sccmram && ptr <= &_ccmramend))

// Create SPI devices the actual configuration is set later
#if USE_SSP1
HardwareSPI HardwareSPI::SSP1(SPI1, SPI1_IRQn);
#endif
#if USE_SSP2
HardwareSPI HardwareSPI::SSP2(SPI2, SPI2_IRQn, DMA1_Stream3, DMA_CHANNEL_0, DMA1_Stream3_IRQn, DMA1_Stream4, DMA_CHANNEL_0, DMA1_Stream4_IRQn);
#endif
#if USE_SSP3
HardwareSPI HardwareSPI::SSP3(SPI3, SPI3_IRQn, DMA1_Stream0, DMA_CHANNEL_0, DMA1_Stream0_IRQn, DMA1_Stream5, DMA_CHANNEL_0, DMA1_Stream5_IRQn);
#endif
#endif

//#define SSPI_DEBUG
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

#if STM32F4
//The following functions are optimised versions of the HAL code for F4 based systems.

void HardwareSPI::SPI_IRQHandler(SPI_HandleTypeDef *hspi) noexcept
{
  uint16_t val = hspi->Instance->DR;
  if (READ_BIT(hspi->Instance->CR1, SPI_CR1_DFF))
  {
    // Data length is 16 bit multiple, use 16 bit I/O
    if (hspi->pTxBuffPtr)
    {
      *(hspi->pRxBuffPtr++) = val >> 8;
      *(hspi->pRxBuffPtr++) = val;
    }
    if (--hspi->RxXferCount == 0)
    {
      // transfer complete
      __HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);
      hspi->State = HAL_SPI_STATE_READY;
      // Get pointer to containing object and handle operation complete
      HardwareSPI *s = (HardwareSPI *)((uint8_t *)hspi - ((uint8_t *)&(HardwareSPI::SSP1.spi.handle) - (uint8_t *)&HardwareSPI::SSP1));
      s->transferActive = false;
      if (s->callback) s->callback(s);
      return;
    }
    // Data length is 16 bit multiple, use 16 bit I/O
    if (hspi->pTxBuffPtr)
    {
      val = ((uint16_t)*hspi->pTxBuffPtr++) << 8;
      val |=  ((uint16_t)(*hspi->pTxBuffPtr++)) & 0xff;
    }
    else
      val = 0xffff;
    hspi->Instance->DR = val;
  }
  else
  {
    // use 8 bit transfers
    if (hspi->pTxBuffPtr)
      *(hspi->pRxBuffPtr++) = val;

    if (--hspi->RxXferCount == 0)
    {
      // transfer complete
      __HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);
      hspi->State = HAL_SPI_STATE_READY;
      // Get pointer to containing object and handle operation complete
      HardwareSPI *s = (HardwareSPI *)((uint8_t *)hspi - ((uint8_t *)&(HardwareSPI::SSP1.spi.handle) - (uint8_t *)&HardwareSPI::SSP1));
      s->transferActive = false;
      if (s->callback) s->callback(s);
      return;
    }
    if (hspi->pTxBuffPtr)
      hspi->Instance->DR = ((uint8_t)*hspi->pTxBuffPtr++);
    else
      hspi->Instance->DR = 0xff;
  }
}

void HardwareSPI::SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma) noexcept
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent);
  // Disable Rx/Tx DMA Request
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  hspi->TxXferCount = 0U;
  hspi->RxXferCount = 0U;
  hspi->State = HAL_SPI_STATE_READY;
  // Get pointer to containing object and handle operation complete
  HardwareSPI *s = (HardwareSPI *)((uint8_t *)hspi - ((uint8_t *)&(HardwareSPI::SSP1.spi.handle) - (uint8_t *)&HardwareSPI::SSP1));
  s->transferActive = false;
  if (s->callback) s->callback(s);
}

HAL_StatusTypeDef HardwareSPI::startTransferDMA(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size) noexcept
{
  static uint32_t dummyDMATxdata = 0xffffffff;
  static uint32_t dummyDMARxdata;


  hspi->pTxBuffPtr  = (pTxData) ? (uint8_t *)pTxData : (uint8_t *)&dummyDMATxdata;
  hspi->TxXferCount = Size;
  hspi->pRxBuffPtr  = (pRxData) ? (uint8_t *)pRxData : (uint8_t *)&dummyDMARxdata;
  hspi->RxXferCount = Size;

  // Set the transaction information
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

  // Set the SPI Tx/Rx DMA Half transfer complete callback
  hspi->hdmarx->XferHalfCpltCallback = NULL;
  hspi->hdmarx->XferCpltCallback     = SPI_DMATransmitReceiveCplt;
  hspi->hdmarx->XferErrorCallback = NULL;
  hspi->hdmarx->XferAbortCallback = NULL;

  // Enable the Rx DMA Stream/Channel
  DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)hspi->pRxBuffPtr, hspi->RxXferCount, DMA_NORMAL, DMA_PERIPH_TO_MEMORY, (pRxData ? DMA_MINC_ENABLE : 0));
  // Enable Rx DMA Request
  SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

  // Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
  // is performed in DMA reception complete callback
  hspi->hdmatx->XferHalfCpltCallback = NULL;
  hspi->hdmatx->XferCpltCallback     = NULL;
  hspi->hdmatx->XferErrorCallback    = NULL;
  hspi->hdmatx->XferAbortCallback    = NULL;
  // Enable the Tx DMA Stream/Channel
  DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DR, hspi->TxXferCount, DMA_NORMAL, DMA_MEMORY_TO_PERIPH, (pTxData ? DMA_MINC_ENABLE : 0));

  // Check if the SPI is already enabled
  if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
  {
    // Enable SPI peripheral
    __HAL_SPI_ENABLE(hspi);
  }
  // Enable Tx DMA Request
  SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);
  return HAL_OK;
}

HAL_StatusTypeDef HardwareSPI::startTransferIT(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size) noexcept
{
  // Set the transaction information
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
  hspi->pRxBuffPtr  = (uint8_t *)pRxData;

  uint16_t val = 0xffff;

  // prepare inital value to transmit
  if ((Size & 1) == 0)
  {
    // Data length is 16 bit multiple, use 16 bit I/O
    if (hspi->pTxBuffPtr)
    {
      val = ((uint16_t)*hspi->pTxBuffPtr++) << 8;
      val |=  ((uint16_t)(*hspi->pTxBuffPtr++)) & 0xff;
    }
    SET_BIT(hspi->Instance->CR1, SPI_CR1_DFF);
    Size = Size/2;
  }
  else
  {
    // Otherwise use 8 bit I/O
    if (hspi->pTxBuffPtr)
    {
      val =  ((uint16_t)(*hspi->pTxBuffPtr++)) & 0xff;
    }
    CLEAR_BIT(hspi->Instance->CR1, SPI_CR1_DFF);
  }
  // Check if the SPI is already enabled
  if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
  {
    // Enable SPI peripheral
    __HAL_SPI_ENABLE(hspi);
  }
  hspi->RxXferCount = Size;
  // "Prime the pump" with the initial output value
  hspi->Instance->DR = val;
  hspi->TxXferCount = --Size;
  __HAL_SPI_ENABLE_IT(hspi, SPI_IT_RXNE);
  return HAL_OK;
}
#endif

#if STM32H7
/*
The following functions are optimised versions of the HAL code for H7 based systems. Note that this code
operates in "continuous" mode. We do not set a transaction length and use the feeding of data into the fifo
to control bus activity. We also avoid enabling/disabling the device between transactions.
The reason for this is that testing has shown that when writing a short packet (in this
case the 5 bytes used with TMC5160 drivers) then there is a delay between starting the transfer and the first
clocks appearing on the clock line. This delay seems to vary in length depending upon the SPI clock speed,
but it can be as high as 3uS per transaction. I suspect it may be caused by the device skipping over empty fifo
elements, but that is just speculation. Running in continuous mode and avoiding enable/disable seems to avoid 
this issue.
*/
void HardwareSPI::SPI_IRQHandler(SPI_HandleTypeDef *hspi) noexcept
{
  // read data from fifo
  while (hspi->RxXferCount != 0UL && HAL_IS_BIT_SET(hspi->Instance->SR, SPI_FLAG_RXP))
  {
    if (hspi->pRxBuffPtr)
      *((uint8_t *)hspi->pRxBuffPtr++) = (*(__IO uint8_t *)&hspi->Instance->RXDR);
    else
      // Just discard the data
      *(__IO uint8_t *)&hspi->Instance->RXDR;
    hspi->RxXferCount--;
  }
  // Write data to fifo if we have any left
  while (hspi->TxXferCount != 0UL && HAL_IS_BIT_SET(hspi->Instance->SR, SPI_FLAG_TXP))
  {
    if (hspi->pTxBuffPtr)
    {
      *(__IO uint8_t *)&hspi->Instance->TXDR = *((uint8_t *)hspi->pTxBuffPtr++);
    }
    else
      *(__IO uint8_t *)&hspi->Instance->TXDR = 0xff;
    hspi->TxXferCount--;
  }
  if (hspi->RxXferCount == 0)
  {
    // finished receiving data, transfer now complete
    // Disable ITs
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));
    // Get pointer to containing object and handle operation complete
    HardwareSPI *s = (HardwareSPI *)((uint8_t *)hspi - ((uint8_t *)&(HardwareSPI::SSP1.spi.handle) - (uint8_t *)&HardwareSPI::SSP1));
    s->transferActive = false;
    if (s->callback) s->callback(s);
  }
}

void HardwareSPI::SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma) noexcept
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (hspi->State != HAL_SPI_STATE_ABORT)
  {
    // Disable ITs
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));
    // Disable Tx DMA Request
    CLEAR_BIT(hspi->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);
    hspi->State = HAL_SPI_STATE_READY;
    // Get pointer to containing object and handle operation complete
    HardwareSPI *s = (HardwareSPI *)((uint8_t *)hspi - ((uint8_t *)&(HardwareSPI::SSP1.spi.handle) - (uint8_t *)&HardwareSPI::SSP1));
    s->transferActive = false;
    if (s->callback) s->callback(s);
  }
}

HAL_StatusTypeDef HardwareSPI::startTransferDMA(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size) noexcept
{
  static uint32_t dummyDMATxdata = 0xffffffff;
  static uint32_t dummyDMARxdata;

  hspi->pTxBuffPtr  = (pTxData) ? (uint8_t *)pTxData : (uint8_t *)&dummyDMATxdata;
  hspi->TxXferCount = Size;
  hspi->pRxBuffPtr  = (pRxData) ? (uint8_t *)pRxData : (uint8_t *)&dummyDMARxdata;
  hspi->RxXferCount = Size;

  // Reset the Tx/Rx DMA bits
  CLEAR_BIT(hspi->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

  // Set the SPI Tx/Rx DMA Half transfer complete callback
  hspi->hdmarx->XferHalfCpltCallback = NULL;
  hspi->hdmarx->XferCpltCallback     = SPI_DMATransmitReceiveCplt;
  hspi->hdmarx->XferErrorCallback = NULL;
  hspi->hdmarx->XferAbortCallback = NULL;
  // It seems that when using DMA in slave mode the SPI unit can sometimes have an extra byte left in the RX register.
  // If we enable DMA with this still in place it triggers a premature completion of the read.
  if ((hspi->Init.Mode & SPI_MODE_MASTER) != SPI_MODE_MASTER)
  {
    __HAL_SPI_DISABLE(hspi);
    __HAL_SPI_ENABLE(hspi);
  }

  // Enable the Rx DMA Stream/Channel
  DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->RXDR, (uint32_t)hspi->pRxBuffPtr, hspi->RxXferCount, DMA_NORMAL, DMA_PERIPH_TO_MEMORY, (pRxData ? DMA_MINC_ENABLE : 0));

  // Enable Rx DMA Request
  SET_BIT(hspi->Instance->CFG1, SPI_CFG1_RXDMAEN);

  // Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
  // is performed in DMA reception complete callback
  hspi->hdmatx->XferHalfCpltCallback = NULL;
  hspi->hdmatx->XferCpltCallback     = NULL;
  hspi->hdmatx->XferErrorCallback    = NULL;
  hspi->hdmatx->XferAbortCallback    = NULL;
  // Enable the Tx DMA Stream/Channel
  DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->TXDR, hspi->TxXferCount, DMA_NORMAL, DMA_MEMORY_TO_PERIPH, (pTxData ? DMA_MINC_ENABLE : 0));

  // Enable Tx DMA Request
  SET_BIT(hspi->Instance->CFG1, SPI_CFG1_TXDMAEN);

  if (hspi->Init.Mode == SPI_MODE_MASTER)
  {
    // Master transfer start
    SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
  }

  return HAL_OK;
}

HAL_StatusTypeDef HardwareSPI::startTransferIT(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size) noexcept
{
  // Even when only doing transmit we still read data, this allows us to use the same code
  // path for all transfers and makes detecting end of operation easy/efficient.
  // This also means we do not need to worry about flushing the RX fifo even when in
  // continuous mode.
  hspi->pRxBuffPtr  = (uint8_t *)pRxData;
  hspi->RxXferCount = Size;

  // Fill in the TxFIFO
  while ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXP)) && (Size != 0UL))
  {
    if (pTxData)
    {
      *((__IO uint8_t *)&hspi->Instance->TXDR) = *((uint8_t *)pTxData++);
    }
    else
      *((__IO uint8_t *)&hspi->Instance->TXDR) = 0xff;
    Size--;
  }
  __DSB();
  hspi->TxXferCount = Size;
  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
  // If all of the data fits in the fifo, we can just wait for the tx and bus activity
  // to complete and save on interrupts.
  if (Size == 0UL)
    __HAL_SPI_ENABLE_IT(hspi, SPI_IT_EOT);
  else
    __HAL_SPI_ENABLE_IT(hspi, SPI_IT_DXP);

  if (hspi->Init.Mode == SPI_MODE_MASTER)
  {
    // Master transfer start
    SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
    __DSB();
  }
  return HAL_OK;
}
#endif

#if USE_SSP2
extern "C" void DMA1_Stream3_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP2.dmaRx));
}

extern "C" void DMA1_Stream4_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP2.dmaTx));
}

extern "C" void SPI2_IRQHandler()
{
    HardwareSPI::SPI_IRQHandler(&(HardwareSPI::SSP2.spi.handle));
}
#endif

#if USE_SSP3
extern "C" void DMA1_Stream0_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP3.dmaRx));
}

extern "C" void DMA1_Stream5_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP3.dmaTx));
}

extern "C" void SPI3_IRQHandler()
{
    HardwareSPI::SPI_IRQHandler(&(HardwareSPI::SSP3.spi.handle));
}
#endif


#if STM32H7
#if USE_SSP1
extern "C" void DMA1_Stream6_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP1.dmaRx));
}

extern "C" void DMA1_Stream7_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP1.dmaTx));
}

extern "C" void SPI1_IRQHandler()
{
    HardwareSPI::SPI_IRQHandler(&(HardwareSPI::SSP1.spi.handle));
}
#endif

#if USE_SSP4
extern "C" void DMA1_Stream1_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP4.dmaRx));
}

extern "C" void DMA1_Stream2_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP4.dmaTx));
}

extern "C" void SPI4_IRQHandler()
{
    HardwareSPI::SPI_IRQHandler(&(HardwareSPI::SSP4.spi.handle));
}
#endif

#if USE_SSP5
extern "C" void DMA2_Stream1_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP5.dmaRx));
}

extern "C" void DMA2_Stream2_IRQHandler()
{
    HAL_DMA_IRQHandler(&(HardwareSPI::SSP5.dmaTx));
}

extern "C" void SPI5_IRQHandler()
{
    HardwareSPI::SPI_IRQHandler(&(HardwareSPI::SSP5.spi.handle));
}
#endif

#if USE_SSP6
extern "C" void SPI6_IRQHandler()
{
    HardwareSPI::SPI_IRQHandler(&(HardwareSPI::SSP6.spi.handle));
}
#endif

#else

#if USE_SSP1
extern "C" void SPI1_IRQHandler()
{
    HardwareSPI::SPI_IRQHandler(&(HardwareSPI::SSP1.spi.handle));
}
#endif
#endif

static inline void flushTxFifo(SPI_HandleTypeDef *sspDevice) noexcept
{

}

static inline void flushRxFifo(SPI_HandleTypeDef *hspi) noexcept
{
   while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
    {
        // read the received data
#if STM32H7
        (void)*(__IO uint8_t *)&hspi->Instance->RXDR;
#else
        (void)*(__IO uint8_t *)&hspi->Instance->DR;
#endif
    }
}

void HardwareSPI::flushRx() noexcept
{
   flushRxFifo(&spi.handle);
}

// Disable the device and flush any data from the fifos
void HardwareSPI::disable() noexcept
{
    if (initComplete)
    {
        if (ioType == SpiIoType::dma)
            HAL_SPI_DMAStop(&(spi.handle));
        flushRxFifo(&spi.handle);
        spi_deinit(&spi);
        initComplete = false;
        transferActive = false;
    }
}

// Wait for transmitter empty returning true if timed out
bool HardwareSPI::waitForTxEmpty() noexcept
{
    return false;
}

// Called on completion of a blocking transfer
void transferComplete(HardwareSPI *spiDevice) noexcept
{
    if (spiDevice->csPin != NoPin) fastDigitalWriteHigh(spiDevice->csPin);
#ifdef RTOS
    if (spiDevice->waitingTask != nullptr)
        spiDevice->waitingTask->GiveFromISR(NotifyIndices::Spi);
#endif
}

void HardwareSPI::initPins(Pin clk, Pin miso, Pin mosi, NvicPriority priority) noexcept
{
    spi.pin_sclk = clk;
    spi.pin_miso = miso;
    spi.pin_mosi = mosi;
    csPin = NoPin;
    if (ioType == SpiIoType::dma)
    {
        initDma(priority);   
    }
    if (ioType != SpiIoType::polled)
    {
        NVIC_SetPriority(spiIrq, priority);
        NVIC_EnableIRQ(spiIrq);
    }
    initComplete = false;
}

void HardwareSPI::configureDmaStream(DMA_HandleTypeDef& hdma, DMA_Stream_TypeDef *inst, uint32_t chan, uint32_t dir, uint32_t minc) noexcept
{
    hdma.Instance                 = inst;
#if STM32H7
    hdma.Init.Request             = chan;
#else    
    hdma.Init.Channel             = chan;
#endif
    hdma.Init.Direction           = dir;
    hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma.Init.MemInc              = minc;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma.Init.Mode                = DMA_NORMAL;
    hdma.Init.Priority            = DMA_PRIORITY_LOW;
    hdma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
    hdma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma.Init.PeriphBurst         = DMA_PBURST_SINGLE;
}

void HardwareSPI::initDma(NvicPriority priority) noexcept
{    
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_DMA_Init(&dmaRx);
    NVIC_SetPriority(rxIrq, priority);
    NVIC_EnableIRQ(rxIrq);      
    __HAL_LINKDMA(&(spi.handle), hdmarx, dmaRx);
    HAL_DMA_Init(&dmaTx); 
    NVIC_SetPriority(txIrq, priority);
    NVIC_EnableIRQ(txIrq);      
    __HAL_LINKDMA(&(spi.handle), hdmatx, dmaTx);
}

void HardwareSPI::configureDevice(uint32_t deviceMode, uint32_t bits, uint32_t clockMode, uint32_t bitRate, Pin cs) noexcept
{
    if (!initComplete || bitRate != curBitRate || bits != curBits || clockMode != curClockMode )
    {
        if (initComplete)
        {
           if (ioType == SpiIoType::dma)
                HAL_SPI_DMAStop(&(spi.handle));
            spi_deinit(&spi);
        }
        spi.pin_ssel = cs;
        spi_init(&spi, dev, deviceMode, bitRate, (spi_mode_e)clockMode, 1);
        initComplete = true;
        transferActive = false;
        curBitRate = bitRate;
        curBits = bits;
        curClockMode = clockMode;
    }
}

//setup the master device.
void HardwareSPI::configureDevice(uint32_t bits, uint32_t clockMode, uint32_t bitRate) noexcept
{
    configureDevice(SPI_MODE_MASTER, bits, clockMode, bitRate);
}

HardwareSPI::HardwareSPI(SPI_TypeDef *spi, IRQn_Type spiIrqNo, DMA_Stream_TypeDef* rxStream, uint32_t rxChan, IRQn_Type rxIrqNo,
                            DMA_Stream_TypeDef* txStream, uint32_t txChan, IRQn_Type txIrqNo) noexcept : dev(spi), spiIrq(spiIrqNo), rxIrq(rxIrqNo), txIrq(txIrqNo), initComplete(false), transferActive(false), ioType(SpiIoType::dma)
{
    configureDmaStream(dmaRx, rxStream, rxChan, DMA_PERIPH_TO_MEMORY, DMA_MINC_ENABLE);
    dmaRx.Init.Priority = DMA_PRIORITY_HIGH;
    configureDmaStream(dmaTx, txStream, txChan, DMA_MEMORY_TO_PERIPH, DMA_MINC_ENABLE);
    curBitRate = 0xffffffff;
    curClockMode = 0xffffffff;
    curBits = 0xffffffff;
}

HardwareSPI::HardwareSPI(SPI_TypeDef *spi, IRQn_Type spiIrqNo) noexcept : dev(spi), spiIrq(spiIrqNo), initComplete(false), transferActive(false), ioType(SpiIoType::interrupt)
{
    curBitRate = 0xffffffff;
    curClockMode = 0xffffffff;
    curBits = 0xffffffff;
}


void HardwareSPI::startTransfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len, SPICallbackFunction ioComplete, size_t minDMALen) noexcept
{
    callback = ioComplete;
    transferActive = true;
    HAL_StatusTypeDef status = HAL_OK;
    switch (ioType)
    {
    case SpiIoType::dma:
        if (len >= minDMALen && CAN_USE_DMA(tx_data, len) && CAN_USE_DMA(rx_data, len))
        {
            status = startTransferDMA(&(spi.handle), tx_data, rx_data, len);
        }
        else
        {
            status = startTransferIT(&(spi.handle), tx_data, rx_data, len);
        }
        break;
    case SpiIoType::polled:
        debugPrintf("Error: Poled SPI is not suported\n");
        break;
    case SpiIoType::interrupt:
        status = startTransferIT(&(spi.handle), tx_data, rx_data, len);
        break;
    default:
        debugPrintf("Warning invalid SPI I/O type %d used\n", (int)ioType);
    }
    if (status != HAL_OK)
        debugPrintf("SPI Error %d\n", (int)status);
}

void HardwareSPI::stopTransfer() noexcept
{
    // Stop a DMA transfer.
    // Note on the STM32F4 HAL_SPI_Abort does not
    // work because it leaves data in the TX fifo (which will not be clocked out 
    // because cs is not set). It seems that the only way to flush this fifo is
    // re-init the device, so we just do that.
    if (initComplete)
    {
        if (transferActive)
        {
#if STM32H7
            HAL_SPI_Abort(&(spi.handle));
            transferActive = false;
#else
            disable();
            configureDevice(spi.handle.Init.Mode, curBits, curClockMode, curBitRate, spi.pin_ssel);
#endif
        }
    }
}

spi_status_t HardwareSPI::transceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len, Pin cs) noexcept
{
    spi_status_t ret = SPI_OK;
    if (cs != NoPin) fastDigitalWriteLow(cs);
    csPin = cs;
#ifdef RTOS
    waitingTask = TaskBase::GetCallerTaskHandle();
    startTransfer(tx_data, rx_data, len, transferComplete, minDMAThreshold);
    while (transferActive)
    {
        if (!TaskBase::TakeIndexed(NotifyIndices::Spi, SPITimeoutMillis))
        {
            break;
        }
    }
    if(transferActive) // timed out
    {
        ret = SPI_TIMEOUT;
        debugPrintf("SPI timeout\n");
        stopTransfer();
    }
    waitingTask = 0;
#else
    startTransfer(tx_data, rx_data, len, transferComplete);
    uint32_t start = millis();
    while(transferActive && millis() - start < SPITimeoutMillis)
    {
    }
    if (transferActive)
    {
        ret = SPI_TIMEOUT;
        debugPrintf("SPI timeout\n");
        stopTransfer();
    }
#endif
    csPin = NoPin;
    if (cs != NoPin) fastDigitalWriteHigh(cs);
    return ret;
}
#endif
