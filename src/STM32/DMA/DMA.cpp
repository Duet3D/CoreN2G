/*
This file contains functions extracted from the HAL DMA modules and modified/optimised for use
with RRF.

Author: Andy
*/
#include "DMA.h"


#if STM32F4
typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength, uint32_t Mode, uint32_t Direction, uint32_t MemInc) noexcept
{
  /* Clear DBM bit, and set inc and direction */
  hdma->Instance->CR &= (uint32_t)(~(DMA_SxCR_DBM|DMA_SxCR_DIR|DMA_SxCR_MINC|DMA_SxCR_PINC|DMA_SxCR_CIRC));
  hdma->Instance->CR |= (Direction | hdma->Init.PeriphInc | MemInc | Mode);
  /* Configure DMA Stream data length */
  hdma->Instance->NDTR = DataLength;

  /* Memory to Peripheral */
  if(Direction == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Stream destination address */
    hdma->Instance->PAR = DstAddress;

    /* Configure DMA Stream source address */
    hdma->Instance->M0AR = SrcAddress;
  }
  /* Peripheral to Memory */
  else
  {
    /* Configure DMA Stream source address */
    hdma->Instance->PAR = SrcAddress;

    /* Configure DMA Stream destination address */
    hdma->Instance->M0AR = DstAddress;
  }
}

HAL_StatusTypeDef DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength, uint32_t Mode,  uint32_t Direction, uint32_t MemInc) noexcept
{
  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
  if(HAL_DMA_STATE_READY == hdma->State)
  {
    /* Change DMA peripheral state */
    hdma->State = HAL_DMA_STATE_BUSY;

    /* Initialize the error code */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;

    /* Configure the source, destination address and the data length */
    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength, Mode, Direction, MemInc);

    /* Clear all interrupt flags at correct offset within the register */
    regs->IFCR = 0x3FU << hdma->StreamIndex;

    /* Enable Common interrupts*/
    hdma->Instance->CR  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    hdma->Instance->FCR |= DMA_IT_FE;

    /* Enable the Peripheral */
    __HAL_DMA_ENABLE(hdma);
  }
  return HAL_OK;
}
#endif

#if STM32H7
typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength, uint32_t Mode, uint32_t Direction, uint32_t MemInc) noexcept
{
  /* calculate DMA base and stream number */
  DMA_Base_Registers  *regs_dma  = (DMA_Base_Registers *)hdma->StreamBaseAddress;

  if(IS_DMA_DMAMUX_ALL_INSTANCE(hdma->Instance) != 0U) /* No DMAMUX available for BDMA1 */
  {
    /* Clear the DMAMUX synchro overrun flag */
    hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;

    if(hdma->DMAmuxRequestGen != 0U)
    {
      /* Clear the DMAMUX request generator overrun flag */
      hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
    }
  }

  if(IS_DMA_STREAM_INSTANCE(hdma->Instance) != 0U) /* DMA1 or DMA2 instance */
  {
    /* Clear all interrupt flags at correct offset within the register */
    regs_dma->IFCR = 0x3FUL << (hdma->StreamIndex & 0x1FU);

    /* Clear DBM bit, and set inc and direction */
    ((DMA_Stream_TypeDef *)hdma->Instance)->CR &= (uint32_t)(~(DMA_SxCR_DBM|DMA_SxCR_DIR|DMA_SxCR_MINC|DMA_SxCR_PINC|DMA_SxCR_CIRC));
    ((DMA_Stream_TypeDef *)hdma->Instance)->CR |= (Direction | hdma->Init.PeriphInc | MemInc | Mode);

    /* Configure DMA Stream data length */
    ((DMA_Stream_TypeDef *)hdma->Instance)->NDTR = DataLength;

    /* Peripheral to Memory */
    if(Direction == DMA_MEMORY_TO_PERIPH)
    {
      /* Configure DMA Stream destination address */
      ((DMA_Stream_TypeDef *)hdma->Instance)->PAR = DstAddress;

      /* Configure DMA Stream source address */
      ((DMA_Stream_TypeDef *)hdma->Instance)->M0AR = SrcAddress;
    }
    /* Memory to Peripheral */
    else
    {
      /* Configure DMA Stream source address */
      ((DMA_Stream_TypeDef *)hdma->Instance)->PAR = SrcAddress;

      /* Configure DMA Stream destination address */
      ((DMA_Stream_TypeDef *)hdma->Instance)->M0AR = DstAddress;
    }
  }
}

HAL_StatusTypeDef DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength, uint32_t Mode, uint32_t Direction, uint32_t MemInc) noexcept
{
  if(HAL_DMA_STATE_READY == hdma->State)
  {
    /* Change DMA peripheral state */
    hdma->State = HAL_DMA_STATE_BUSY;

    /* Initialize the error code */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;

    /* Disable the peripheral */
    __HAL_DMA_DISABLE(hdma);

    /* Configure the source, destination address and the data length */
    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength, Mode, Direction, MemInc);

    MODIFY_REG(((DMA_Stream_TypeDef   *)hdma->Instance)->CR, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT), (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME));
    if(IS_DMA_DMAMUX_ALL_INSTANCE(hdma->Instance) != 0U) /* No DMAMUX available for BDMA1 */
    {
      /* Check if DMAMUX Synchronization is enabled */
      if((hdma->DMAmuxChannel->CCR & DMAMUX_CxCR_SE) != 0U)
      {
        /* Enable DMAMUX sync overrun IT*/
        hdma->DMAmuxChannel->CCR |= DMAMUX_CxCR_SOIE;
      }

      if(hdma->DMAmuxRequestGen != 0U)
      {
        /* if using DMAMUX request generator, enable the DMAMUX request generator overrun IT*/
        /* enable the request gen overrun IT */
        hdma->DMAmuxRequestGen->RGCR |= DMAMUX_RGxCR_OIE;
      }
    }

    /* Enable the Peripheral */
    __HAL_DMA_ENABLE(hdma);
  }
  return HAL_OK;
}
#endif