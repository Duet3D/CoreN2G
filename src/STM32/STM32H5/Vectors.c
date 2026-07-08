/**
 * \file
 *
 * \brief gcc starttup file for SAME54
 *
 * Copyright (c) 2017 Microchip Technology Inc.
 *
 * \asf_license_start
 *
 * \page License
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the Licence at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * \asf_license_stop
 *
 */

#include <ecv_duet3d.h>
#include <Core.h>
#include "Vectors.h"

// Symbols defined by the linker script
extern uint32_t _estack;
extern uint32_t _firmware_crc;
extern const char VersionText[];

// SystemCoreClock is needed by FreeRTOS. Declaring this here also ensures that the linker includes this object file.
uint32_t SystemCoreClock = 240000000;

// Forward declaration
void Reset_Handler(void);

/* Default empty handler */
void Dummy_Handler(void);

/* Cortex-M4 core handlers */
void NMI_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void HardFault_Handler       ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void MemManage_Handler       ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void BusFault_Handler        ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UsageFault_Handler      ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SecureFault_Handler     ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SVC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DebugMon_Handler        ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PendSV_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SysTick_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

/* Peripherals handlers */
void WWDG_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PVD_AVD_IRQHandler 			( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_IRQHandler					( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_S_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TAMP_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RAMCFG_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void FLASH_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void FLASH_S_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GTZC_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RCC_IRQHandler					( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RCC_S_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI0_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI1_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI2_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI3_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI4_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI5_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI6_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI7_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI8_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI9_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI10_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI11_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI12_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI13_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI14_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EXTI15_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA1_Channel0_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA1_Channel1_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA1_Channel2_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA1_Channel3_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA1_Channel4_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA1_Channel5_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA1_Channel6_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA1_Channel7_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void IWDG_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC1_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DAC1_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void FDCAN1_IT0_IRQHandler			( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void FDCAN1_IT1_IRQHandler			( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM1_BRK_IRQHandler			( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM1_UP_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM1_TRG_COM_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM1_CC_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM2_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM3_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM4_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM5_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM6_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM7_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2C1_EV_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2C1_ER_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2C2_EV_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2C2_ER_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SPI1_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SPI2_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SPI3_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USART1_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USART2_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USART3_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UART4_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UART5_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void LPUART1_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void LPTIM1_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM8_BRK_IRQHandler			( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM8_UP_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM8_TRG_COM_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM8_CC_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC2_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void LPTIM2_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM15_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USB_DRD_FS_IRQHandler			( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void CRS_IRQHandler					( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UCPD1_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void FMC_IRQHandler					( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void OCTOSPI1_IRQHandler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SDMMC1_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2C3_EV_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2C3_ER_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SPI4_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USART6_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA2_Channel0_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA2_Channel1_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA2_Channel2_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA2_Channel3_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA2_Channel4_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA2_Channel5_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA2_Channel6_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void GPDMA2_Channel7_IRQHandler		( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void FPU_IRQHandler					( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ICACHE_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DCACHE1_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DCMI_PSSI_IRQHandler			( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void FDCAN2_IT0_IRQHandler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void FDCAN2_IT1_IRQHandler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DTS_IRQHandler					( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RNG_IRQHandler					( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void OFTDEC1_Handler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void HASH_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void CEC_IRQHandler					( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TIM12_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I3C1_EV_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I3C1_ER_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I3C2_EV_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I3C2_ER_IRQHandler				( void ) __attribute__ ((weak, alias("Dummy_Handler")));

// Exception Table
__attribute__ ((section(".vectors")))
const struct DeviceVectors exception_table =
{
        // Configure Initial Stack Pointer using linker-generated symbols
        .pvStack					= (void*) (&_estack),

        .pfnReset_Handler			= (void*) Reset_Handler,
        .pfnNonMaskableInt_Handler	= (void*) NMI_Handler,
        .pfnHardFault_Handler		= (void*) HardFault_Handler,
        .pfnMemManagement_Handler	= (void*) MemManage_Handler,
        .pfnBusFault_Handler		= (void*) BusFault_Handler,
        .pfnUsageFault_Handler		= (void*) UsageFault_Handler,
#if 1
		.pvReservedM9				= (void*) &_firmware_crc,				// we store a pointer to the firmware CRC here */
        .pvReservedM8				= (void*) VersionText,					// we store a pointer to the firmware or bootloader version text here */
        .pvReservedM7				= (void*) &exception_table,				// load offset of the start of the file so that programs can subtract this from the offsets of firmware_crc and VersionText */
#else
        .pvReservedM9				= (void*) SecureFault_Handler,
        .pvReservedM8				= (void*) (0UL), // Reserved
        .pvReservedM7				= (void*) (0UL), // Reserved
#endif
        .pvReservedM6				= (void*) (0UL), // Reserved
        .pfnSVCall_Handler			= (void*) SVC_Handler,
        .pfnDebugMonitor_Handler	= (void*) DebugMon_Handler,
        .pvReservedM3				= (void*) (0UL), // Reserved
        .pfnPendSV_Handler			= (void*) PendSV_Handler,
        .pfnSysTick_Handler			= (void*) SysTick_Handler,

        // Configurable interrupts
        .pfnWWDG_Handler			= (void*) WWDG_IRQHandler,				//  0
        .pfnPFD_AVD_Handler			= (void*) PVD_AVD_IRQHandler,			//  1
        .pfnRTC_Handler				= (void*) RTC_IRQHandler,				//  2
        .pfnRTC_S_Handler			= (void*) RTC_S_IRQHandler,				//  3
        .pfnTAMP_Handler			= (void*) TAMP_IRQHandler,				//  4
        .pfnRAMCFG_Handler			= (void*) RAMCFG_IRQHandler,			//  5
        .pfnFLASH_Handler			= (void*) FLASH_IRQHandler,				//  6
        .pfnFLASH_S_Handler			= (void*) FLASH_S_IRQHandler,			//  7
        .pfnGTZC_Handler			= (void*) GTZC_IRQHandler,				//  8
        .pfnRCC_Handler				= (void*) RCC_IRQHandler,				//  9
        .pfnRCC_S_Handler			= (void*) RCC_S_IRQHandler,				// 10
        .pfnEXTI0_Handler			= (void*) EXTI0_IRQHandler,				// 11
        .pfnEXTI1_Handler			= (void*) EXTI1_IRQHandler,				// 12
        .pfnEXTI2_Handler			= (void*) EXTI2_IRQHandler,				// 13
        .pfnEXTI3_Handler			= (void*) EXTI3_IRQHandler,				// 14
        .pfnEXTI4_Handler			= (void*) EXTI4_IRQHandler,				// 15
        .pfnEXTI5_Handler			= (void*) EXTI5_IRQHandler,				// 16
        .pfnEXTI6_Handler			= (void*) EXTI6_IRQHandler,				// 17
        .pfnEXTI7_Handler			= (void*) EXTI7_IRQHandler,				// 18
        .pfnEXTI8_Handler			= (void*) EXTI8_IRQHandler,				// 19
        .pfnEXTI9_Handler			= (void*) EXTI9_IRQHandler,				// 20
        .pfnEXTI10_Handler			= (void*) EXTI10_IRQHandler,			// 21
        .pfnEXTI11_Handler			= (void*) EXTI11_IRQHandler,			// 22
        .pfnEXTI12_Handler			= (void*) EXTI12_IRQHandler,			// 23
        .pfnEXTI13_Handler			= (void*) EXTI13_IRQHandler,			// 24
        .pfnEXTI14_Handler			= (void*) EXTI14_IRQHandler,			// 25
        .pfnEXTI15_Handler			= (void*) EXTI15_IRQHandler,			// 26
        .pfnGPDMA1_Channel0_Handler	= (void*) GPDMA1_Channel0_IRQHandler,	// 27
        .pfnGPDMA1_Channel1_Handler	= (void*) GPDMA1_Channel1_IRQHandler,	// 28
        .pfnGPDMA1_Channel2_Handler	= (void*) GPDMA1_Channel2_IRQHandler,	// 29
        .pfnGPDMA1_Channel3_Handler	= (void*) GPDMA1_Channel3_IRQHandler,	// 30
        .pfnGPDMA1_Channel4_Handler	= (void*) GPDMA1_Channel4_IRQHandler,	// 31
        .pfnGPDMA1_Channel5_Handler	= (void*) GPDMA1_Channel5_IRQHandler,	// 32
        .pfnGPDMA1_Channel6_Handler	= (void*) GPDMA1_Channel6_IRQHandler,	// 33
        .pfnGPDMA1_Channel7_Handler	= (void*) GPDMA1_Channel7_IRQHandler,	// 34
        .pfnIWDG_Handler			= (void*) IWDG_IRQHandler,				// 35
        .pvReserved36				= (void*) (0UL),						// 36
        .pfnADC1_Handler			= (void*) ADC1_IRQHandler,				// 37
        .pfnDAC1_Handler			= (void*) DAC1_IRQHandler,				// 38
        .pfnFDCAN1_IT0_Handler		= (void*) FDCAN1_IT0_IRQHandler,        // 39
        .pfnFDCAN1_IT1_Handler		= (void*) FDCAN1_IT1_IRQHandler,        // 40
        .pfnTIM1_BRK_Handler		= (void*) TIM1_BRK_IRQHandler,			// 41
        .pfnTIM1_UP_Handler			= (void*) TIM1_UP_IRQHandler,			// 42
        .pfnTIM1_TRG_COM_IHandler	= (void*) TIM1_TRG_COM_IRQHandler,		// 43
        .pfnTIM1_CC_Handler			= (void*) TIM1_CC_IRQHandler,			// 44
        .pfnTIM2_Handler			= (void*) TIM2_IRQHandler,				// 45
        .pfnTIM3_Handler			= (void*) TIM3_IRQHandler,				// 46
        .pfnTIM4_Handler			= (void*) TIM4_IRQHandler,				// 47
        .pfnTIM5_Handler			= (void*) TIM5_IRQHandler,				// 48
        .pfnTIM6_Handler			= (void*) TIM6_IRQHandler,				// 49
        .pfnTIM7_Handler			= (void*) TIM7_IRQHandler,				// 50
        .pfnI2C1_EV_Handler			= (void*) I2C1_EV_IRQHandler,			// 51
        .pfnI2C1_ER_Handler			= (void*) I2C1_ER_IRQHandler,			// 52
        .pfnI2C2_EV_3_Handler		= (void*) I2C2_EV_IRQHandler,			// 53
        .pfnI2C2_ER_Handler			= (void*) I2C2_ER_IRQHandler,			// 54
        .pfnSPI1_Handler			= (void*) SPI1_IRQHandler,				// 55
        .pfnSPI2_Handler			= (void*) SPI2_IRQHandler,				// 56
        .pfnSPI3_Handler			= (void*) SPI3_IRQHandler,				// 57
        .pfnUSART1_Handler			= (void*) USART1_IRQHandler,			// 58
        .pfnUSART2_Handler			= (void*) USART2_IRQHandler,			// 59
        .pfnUSART3_Handler			= (void*) USART3_IRQHandler,			// 60
        .pfnUART4_Handler			= (void*) UART4_IRQHandler,				// 61
        .pfnUART5_Handler			= (void*) UART5_IRQHandler,				// 62
        .pfnLPUART1_Handler			= (void*) LPUART1_IRQHandler,			// 63
        .pfnLPTIM1_Handler			= (void*) LPTIM1_IRQHandler,			// 64
        .pfnTIM8_BRK_Handler		= (void*) TIM8_BRK_IRQHandler,			// 65
        .pfnTIM8_UP_Handler			= (void*) TIM8_UP_IRQHandler,			// 66
        .pfnTIM8_TRG_COM_Handler	= (void*) TIM8_TRG_COM_IRQHandler,      // 67
        .pfnTIM8_CC_Handler			= (void*) TIM8_CC_IRQHandler,			// 68
        .pfnADC2_Handler			= (void*) ADC2_IRQHandler,				// 69
        .pfnLPTIM2_Handler			= (void*) LPTIM2_IRQHandler,			// 70
        .pfnTIM15_Handler			= (void*) TIM15_IRQHandler,				// 71
        .pvReserved72				= (void*) (0UL),						// 72
        .pvReserved73				= (void*) (0UL),						// 73
        .pfnUSB_DRD_FS_Handler		= (void*) USB_DRD_FS_IRQHandler,		// 74
        .pfnCRS_Handler				= (void*) CRS_IRQHandler,				// 75
        .pfnUCPD1_Handler			= (void*) UCPD1_IRQHandler,				// 76
        .pfnFMC_Handler				= (void*) FMC_IRQHandler,				// 77
        .pfnOCTOSPI1_Handler		= (void*) OCTOSPI1_IRQHandler,			// 78
        .pfnSDMMC1_Handler			= (void*) SDMMC1_IRQHandler,			// 79
        .pfnI2C3_EV_Handler			= (void*) I2C3_EV_IRQHandler,			// 80
        .pfnI2C3_ER_Handler			= (void*) I2C3_ER_IRQHandler,			// 81
        .pfnSPI4_Handler			= (void*) SPI4_IRQHandler,				// 82
        .pvReserved83				= (void*) (0UL),						// 83
        .pvReserved84				= (void*) (0UL),						// 84
        .pfnUSART6_Handler			= (void*) USART6_IRQHandler,			// 85
        .pvReserved86				= (void*) (0UL),						// 86
        .pvReserved87				= (void*) (0UL),						// 87
        .pvReserved88				= (void*) (0UL),						// 88
        .pvReserved89				= (void*) (0UL),						// 89
        .pfnGPDMA2_Channel0_Handler	= (void*) GPDMA2_Channel0_IRQHandler,	// 90
        .pfnGPDMA2_Channel1_Handler	= (void*) GPDMA2_Channel1_IRQHandler,	// 91
        .pfnGPDMA2_Channel2_Handler	= (void*) GPDMA2_Channel2_IRQHandler,	// 92
        .pfnGPDMA2_Channel3_Handler	= (void*) GPDMA2_Channel3_IRQHandler,	// 93
        .pfnGPDMA2_Channel4_Handler	= (void*) GPDMA2_Channel4_IRQHandler,	// 94
        .pfnGPDMA2_Channel5_Handler	= (void*) GPDMA2_Channel5_IRQHandler,	// 95
        .pfnGPDMA2_Channel6_Handler	= (void*) GPDMA2_Channel6_IRQHandler,	// 96
        .pfnGPDMA2_Channel7_Handler	= (void*) GPDMA2_Channel7_IRQHandler,	// 97
        .pvReserved98				= (void*) (0UL),						// 98
        .pvReserved99				= (void*) (0UL),						// 99
        .pvReserved100				= (void*) (0UL),						// 100
        .pvReserved101				= (void*) (0UL),						// 101
        .pvReserved102				= (void*) (0UL),						// 102
        .pfnFPU_Handler				= (void*) FPU_IRQHandler,				// 103
        .pfnICACHE_Handler			= (void*) ICACHE_IRQHandler,			// 104
        .pfnDCACHE1_Handler			= (void*) DCACHE1_IRQHandler,			// 105
        .pvReserved106				= (void*) (0UL),						// 106
        .pvReserved107				= (void*) (0UL),						// 107
        .pfnDCMI_PSSI_Handler		= (void*) DCMI_PSSI_IRQHandler,			// 108
        .pfnFDCAN2_IT0_Handler		= (void*) FDCAN2_IT0_IRQHandler,		// 109
        .pfnFDCAN2_IT1_Handler		= (void*) FDCAN2_IT1_IRQHandler,		// 110
        .pvReserved111				= (void*) (0UL),						// 111
        .pvReserved112				= (void*) (0UL),						// 112
        .pfnDTS_Handler				= (void*) DTS_IRQHandler,				// 113
        .pfnRNG_Handler				= (void*) RNG_IRQHandler,				// 114
        .pfnOFTDEC1_Handler			= (void*) OFTDEC1_Handler,				// 115
        .pvReserved116				= (void*) (0UL),						// 116
        .pfnHASH_Handler			= (void*) HASH_IRQHandler,				// 117
        .pvReserved118				= (void*) (0UL),						// 118
        .pfnCEC_Handler				= (void*) CEC_IRQHandler,				// 119
        .pfnTIM12_Handler			= (void*) TIM12_IRQHandler,				// 120
        .pvReserved121				= (void*) (0UL),						// 121
        .pvReserved122				= (void*) (0UL),						// 122
        .pfnI3C1_EV_Handler			= (void*) I3C1_EV_IRQHandler,			// 123
        .pfnI3C1_ER_Handler			= (void*) I3C1_ER_IRQHandler,			// 124
        .pvReserved125				= (void*) (0UL),						// 125
        .pvReserved126				= (void*) (0UL),						// 126
        .pvReserved127				= (void*) (0UL),						// 127
        .pvReserved128				= (void*) (0UL),						// 128
        .pvReserved129				= (void*) (0UL),						// 129
        .pvReserved130				= (void*) (0UL),						// 130
        .pfnI3C2_EV_Handler			= (void*) I3C2_EV_IRQHandler,			// 131
        .pfnI3C2_ER_Handler         = (void*) I3C2_ER_IRQHandler,			// 132
};

/**
 * \brief Default interrupt handler for unused IRQs.
 */
void Dummy_Handler(void) noexcept
{
	while (1) { }
}

// End
