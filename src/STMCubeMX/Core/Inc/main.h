/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_LPTIM_MspPostInit(LPTIM_HandleTypeDef *hlptim);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRIVER_DIAG_Pin GPIO_PIN_13
#define DRIVER_DIAG_GPIO_Port GPIOC
#define DRIVER_ENN_Pin GPIO_PIN_14
#define DRIVER_ENN_GPIO_Port GPIOC
#define IO1_Pin GPIO_PIN_15
#define IO1_GPIO_Port GPIOC
#define TEMP0_Pin GPIO_PIN_0
#define TEMP0_GPIO_Port GPIOC
#define TEMP1_Pin GPIO_PIN_1
#define TEMP1_GPIO_Port GPIOC
#define TEMP2_Pin GPIO_PIN_2
#define TEMP2_GPIO_Port GPIOC
#define VIN_MON_Pin GPIO_PIN_3
#define VIN_MON_GPIO_Port GPIOC
#define TEMP3_Pin GPIO_PIN_0
#define TEMP3_GPIO_Port GPIOA
#define ADC_DRDY_Pin GPIO_PIN_1
#define ADC_DRDY_GPIO_Port GPIOA
#define NP_OUT_Pin GPIO_PIN_2
#define NP_OUT_GPIO_Port GPIOA
#define DRIVER_DIR_Pin GPIO_PIN_3
#define DRIVER_DIR_GPIO_Port GPIOA
#define BOARD_TYPE_Pin GPIO_PIN_4
#define BOARD_TYPE_GPIO_Port GPIOC
#define HEATER_CURRENT_Pin GPIO_PIN_5
#define HEATER_CURRENT_GPIO_Port GPIOC
#define LDC_INT_Pin GPIO_PIN_0
#define LDC_INT_GPIO_Port GPIOB
#define IO0_OUT_Pin GPIO_PIN_2
#define IO0_OUT_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_6
#define OUT1_GPIO_Port GPIOC
#define OUT2_Pin GPIO_PIN_7
#define OUT2_GPIO_Port GPIOC
#define IO0_Pin GPIO_PIN_8
#define IO0_GPIO_Port GPIOC
#define LDC_CLK_Pin GPIO_PIN_9
#define LDC_CLK_GPIO_Port GPIOC
#define DRIVER_CLK_Pin GPIO_PIN_8
#define DRIVER_CLK_GPIO_Port GPIOA
#define OUT0_Pin GPIO_PIN_9
#define OUT0_GPIO_Port GPIOA
#define OUT_2_TACHO_Pin GPIO_PIN_10
#define OUT_2_TACHO_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define DRIVER_STEP_Pin GPIO_PIN_2
#define DRIVER_STEP_GPIO_Port GPIOD
#define IO2_Pin GPIO_PIN_3
#define IO2_GPIO_Port GPIOB
#define OUT_1_TACHO_Pin GPIO_PIN_4
#define OUT_1_TACHO_GPIO_Port GPIOB
#define ACC_INT_Pin GPIO_PIN_5
#define ACC_INT_GPIO_Port GPIOB
#define ADC_CLK_Pin GPIO_PIN_6
#define ADC_CLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
