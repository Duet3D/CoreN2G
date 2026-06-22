/**
  ******************************************************************************
  * @file    spi_com.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    01-August-2016
  * @brief   provide the SPI interface
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#if 0	// DC temporarily disable incomplete SPI code

 // NOTE: This file has been extensively modified for use in the STM32 port of RRF
 // by Andy.
//#include "stm32_def.h"
#include "spi_com.h"
#include "pinconfig.h"

#ifdef __cplusplus
extern "C" {
#endif
#if defined(HAL_SPI_MODULE_ENABLED)
extern void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

/* Private Functions */
/**
  * @brief  return clock freq of an SPI instance
  * @param  spi_inst : SPI instance
  * @retval clock freq of the instance else SystemCoreClock
  */
uint32_t spi_getClkFreqInst(SPI_TypeDef *spi_inst)
{
  uint32_t spi_freq = SystemCoreClock;
  if (spi_inst != NP) {
#if defined(STM32F0xx) || defined(STM32G0xx)
    /* SPIx source CLK is PCKL1 */
    spi_freq = HAL_RCC_GetPCLK1Freq();
#else
#if defined(SPI1_BASE)
    if (spi_inst == SPI1) {
#if defined(RCC_PERIPHCLK_SPI1) || defined(RCC_PERIPHCLK_SPI123)
#ifdef RCC_PERIPHCLK_SPI1
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI1);
#else
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
#endif
      if (spi_freq == 0)
#endif
      {
        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
        spi_freq = HAL_RCC_GetPCLK2Freq();
      }
    }
#endif // SPI1_BASE
#if defined(SPI2_BASE)
    if (spi_inst == SPI2) {
#if defined(RCC_PERIPHCLK_SPI2) || defined(RCC_PERIPHCLK_SPI123) ||\
    defined(RCC_PERIPHCLK_SPI23)
#ifdef RCC_PERIPHCLK_SPI2
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2);
#elif defined(RCC_PERIPHCLK_SPI123)
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
#else
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI23);
#endif
      if (spi_freq == 0)
#endif
      {
        /* SPI_2 and SPI_3. Source CLK is PCKL1 */
        spi_freq = HAL_RCC_GetPCLK1Freq();
      }
    }
#endif // SPI2_BASE
#if defined(SPI3_BASE)
    if (spi_inst == SPI3) {
#if defined(RCC_PERIPHCLK_SPI3) || defined(RCC_PERIPHCLK_SPI123) ||\
    defined(RCC_PERIPHCLK_SPI23)
#ifdef RCC_PERIPHCLK_SPI3
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI3);
#elif defined(RCC_PERIPHCLK_SPI123)
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
#else
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI23);
#endif
      if (spi_freq == 0)
#endif
      {
        /* SPI_2 and SPI_3. Source CLK is PCKL1 */
        spi_freq = HAL_RCC_GetPCLK1Freq();
      }
    }
#endif // SPI3_BASE
#if defined(SPI4_BASE)
    if (spi_inst == SPI4) {
#if defined(RCC_PERIPHCLK_SPI4) || defined(RCC_PERIPHCLK_SPI45)
#ifdef RCC_PERIPHCLK_SPI4
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI4);
#else
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45);
#endif
      if (spi_freq == 0)
#endif
      {
        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
        spi_freq = HAL_RCC_GetPCLK2Freq();
      }
    }
#endif // SPI4_BASE
#if defined(SPI5_BASE)
    if (spi_inst == SPI5) {
#if defined(RCC_PERIPHCLK_SPI5) || defined(RCC_PERIPHCLK_SPI45)
#ifdef RCC_PERIPHCLK_SPI5
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI5);
#else
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45);
#endif
      if (spi_freq == 0)
#endif
      {
        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
        spi_freq = HAL_RCC_GetPCLK2Freq();
      }
    }
#endif // SPI5_BASE
#if defined(SPI6_BASE)
    if (spi_inst == SPI6) {
#if defined(RCC_PERIPHCLK_SPI6)
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6);
      if (spi_freq == 0)
#endif
      {
        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
        spi_freq = HAL_RCC_GetPCLK2Freq();
      }
    }
#endif // SPI6_BASE
#endif
  }
  return spi_freq;
}


/**
  * @brief  SPI initialization function
  * @param  obj : pointer to spi_t structure
  * @param spimode: master/slave mode
  * @param  speed : spi output speed
  * @param  mode : one of the spi modes
  * @param  msb : set to 1 in msb first
  * @retval None
  */
void spi_init(spi_t *obj, SPI_TypeDef *dev, uint32_t spimode, uint32_t speed, spi_mode_e mode, uint8_t msb)
{
  if (obj == NULL) {
    return;
  }
  SPI_HandleTypeDef *handle = &(obj->handle);
  uint32_t spi_freq = 0;
  obj->spi = dev;
  // Configure the SPI pins
  if (obj->pin_ssel != NC) {
    handle->Init.NSS = (spimode == SPI_MODE_SLAVE ? SPI_NSS_HARD_INPUT : SPI_NSS_HARD_OUTPUT);
  } else {
    handle->Init.NSS = SPI_NSS_SOFT;
  }

  /* Fill default value */
  handle->Instance               = obj->spi;
  handle->Init.Mode              = spimode;

  spi_freq = spi_getClkFreqInst(obj->spi);
  //debugPrintf("spi freq %d lowest %d requested %d\n", spi_freq, spi_freq / SPI_SPEED_CLOCK_DIV256_MHZ, speed);
  if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV2_MHZ)) {
    handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV4_MHZ)) {
    handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV8_MHZ)) {
    handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV16_MHZ)) {
    handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV32_MHZ)) {
    handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV64_MHZ)) {
    handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV128_MHZ)) {
    handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV256_MHZ)) {
    handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  } else {
    handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  }
  //debugPrintf("selected prescaler %x div256 %x\n", handle->Init.BaudRatePrescaler, SPI_BAUDRATEPRESCALER_256);
  handle->Init.Direction         = SPI_DIRECTION_2LINES;

  if ((mode == CORE_SPI_MODE_0) || (mode == CORE_SPI_MODE_2)) {
    handle->Init.CLKPhase          = SPI_PHASE_1EDGE;
  } else {
    handle->Init.CLKPhase          = SPI_PHASE_2EDGE;
  }

  if ((mode == CORE_SPI_MODE_0) || (mode == CORE_SPI_MODE_1)) {
    handle->Init.CLKPolarity       = SPI_POLARITY_LOW;
  } else {
    handle->Init.CLKPolarity       = SPI_POLARITY_HIGH;
  }

  handle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  handle->Init.CRCPolynomial     = 7;
  handle->Init.DataSize          = SPI_DATASIZE_8BIT;

  if (msb == 0) {
    handle->Init.FirstBit          = SPI_FIRSTBIT_LSB;
  } else {
    handle->Init.FirstBit          = SPI_FIRSTBIT_MSB;
  }

  handle->Init.TIMode            = SPI_TIMODE_DISABLE;
#if defined(SPI_NSS_PULSE_DISABLE)
  handle->Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
#endif
#ifdef SPI_MASTER_KEEP_IO_STATE_ENABLE
  handle->Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  /* Recommended setting to avoid glitches */
#endif

  /* Configure SPI GPIO pins */
  if (!pinmap_pinout2(dev, obj->pin_mosi, PinMap_SPI_MOSI) ||
      !pinmap_pinout2(dev, obj->pin_miso, PinMap_SPI_MISO) ||
      !pinmap_pinout2(dev, obj->pin_sclk, PinMap_SPI_SCLK) ||
      !pinmap_pinout2(dev, obj->pin_ssel, PinMap_SPI_SSEL))
  {
    debugPrintf("ERROR: Not all pins are available for SPI device\n");
    return;
  }
  if (spimode == SPI_MODE_SLAVE)
  {
    uint32_t pull = (handle->Init.CLKPolarity == SPI_POLARITY_LOW) ? GPIO_PULLDOWN : GPIO_PULLUP;
    pin_PullConfig(get_GPIO_Port(STM_PORT(obj->pin_sclk)), STM_LL_GPIO_PIN(obj->pin_sclk), pull);
  }
  else
  {
    pin_speed(obj->pin_sclk, GPIO_SPEED_FREQ_LOW);
    pin_speed(obj->pin_mosi, GPIO_SPEED_FREQ_LOW);
  }

#if defined SPI1_BASE
  // Enable SPI clock
  if (handle->Instance == SPI1) {
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
  }
#endif

#if defined SPI2_BASE
  if (handle->Instance == SPI2) {
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
  }
#endif

#if defined SPI3_BASE
  if (handle->Instance == SPI3) {
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_SPI3_FORCE_RESET();
    __HAL_RCC_SPI3_RELEASE_RESET();
  }
#endif

#if defined SPI4_BASE
  if (handle->Instance == SPI4) {
    __HAL_RCC_SPI4_CLK_ENABLE();
    __HAL_RCC_SPI4_FORCE_RESET();
    __HAL_RCC_SPI4_RELEASE_RESET();
  }
#endif

#if defined SPI5_BASE
  if (handle->Instance == SPI5) {
    __HAL_RCC_SPI5_CLK_ENABLE();
    __HAL_RCC_SPI5_FORCE_RESET();
    __HAL_RCC_SPI5_RELEASE_RESET();
  }
#endif

#if defined SPI6_BASE
  if (handle->Instance == SPI6) {
    __HAL_RCC_SPI6_CLK_ENABLE();
    __HAL_RCC_SPI6_FORCE_RESET();
    __HAL_RCC_SPI6_RELEASE_RESET();
  }
#endif

  HAL_SPI_Init(handle);
#if STM32H7
  // Set transfer size to zero to enable continuous mode
  MODIFY_REG(handle->Instance->CR2, SPI_CR2_TSIZE, 0UL);
#endif
  /* In order to set correctly the SPI polarity we need to enable the peripheral */
  __HAL_SPI_ENABLE(handle);
}

/**
  * @brief This function is implemented to deinitialize the SPI interface
  *        (IOs + SPI block)
  * @param  obj : pointer to spi_t structure
  * @retval None
  */
void spi_deinit(spi_t *obj)
{
  if (obj == NULL) {
    return;
  }

  SPI_HandleTypeDef *handle = &(obj->handle);

  HAL_SPI_DeInit(handle);

#if defined SPI1_BASE
  // Reset SPI and disable clock
  if (handle->Instance == SPI1) {
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    __HAL_RCC_SPI1_CLK_DISABLE();
  }
#endif
#if defined SPI2_BASE
  if (handle->Instance == SPI2) {
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
    __HAL_RCC_SPI2_CLK_DISABLE();
  }
#endif

#if defined SPI3_BASE
  if (handle->Instance == SPI3) {
    __HAL_RCC_SPI3_FORCE_RESET();
    __HAL_RCC_SPI3_RELEASE_RESET();
    __HAL_RCC_SPI3_CLK_DISABLE();
  }
#endif

#if defined SPI4_BASE
  if (handle->Instance == SPI4) {
    __HAL_RCC_SPI4_FORCE_RESET();
    __HAL_RCC_SPI4_RELEASE_RESET();
    __HAL_RCC_SPI4_CLK_DISABLE();
  }
#endif

#if defined SPI5_BASE
  if (handle->Instance == SPI5) {
    __HAL_RCC_SPI5_FORCE_RESET();
    __HAL_RCC_SPI5_RELEASE_RESET();
    __HAL_RCC_SPI5_CLK_DISABLE();
  }
#endif

#if defined SPI6_BASE
  if (handle->Instance == SPI6) {
    __HAL_RCC_SPI6_FORCE_RESET();
    __HAL_RCC_SPI6_RELEASE_RESET();
    __HAL_RCC_SPI6_CLK_DISABLE();
  }
#endif
}

#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
