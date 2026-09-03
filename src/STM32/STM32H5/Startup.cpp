/*
 * Startup.cpp
 *
 *  Created on: 2 Aug 2020
 *      Author: David
 */

#undef __SOFTFP__			// should not be defined, but Eclipse thinks it is

#include <CoreIO.h>

// Symbols defined by the linker
extern uint32_t _sfixed;
//extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;
extern uint32_t _sstack;

extern uint32_t _sDmaBuffers;				// defined in linker script
extern uint32_t _eDmaBuffers;				// defined in linker script

extern "C" void __libc_init_array() noexcept;

// Forward declaration
void SystemInit() noexcept;

/**
 * \brief This is the code that gets called on processor reset.
 * To initialize the device, and call the main() routine.
 */
extern "C" [[noreturn]] void Reset_Handler() noexcept
{
	asm("ldr   r0, =_sstack");
	asm("msr   MSPLIM, r0");      // set stack pointer limit

	SystemInit();

	uint32_t *pSrc = &_etext;
	uint32_t *pDest = &_srelocate;

	if (pSrc != pDest)
	{
		for (; pDest < &_erelocate; )
		{
			*pDest++ = *pSrc++;
		}
	}

	// Clear the zero segment
	// It doesn't matter if we overshoot the end of the zero segment, so now we do 4 dwords at a time to reduce the time taken to 14ms
	for (pDest = &_szero; pDest < &_ezero; )
	{
		*pDest++ = 0;
		*pDest++ = 0;
		*pDest++ = 0;
		*pDest++ = 0;
	}

	// Clear the DMA state segment
	for (pDest = &_sDmaBuffers; pDest < &_eDmaBuffers; )
	{
		*pDest++ = 0;
	}

	// Set the vector table base address
	pSrc = (uint32_t *) & _sfixed;
	SCB->VTOR = ((uint32_t) pSrc & SCB_VTOR_TBLOFF_Msk);

	static_assert(__FPU_USED);
	// Enable FPU
	SCB->CPACR |= ((3UL << 20U)|(3UL << 22U));		// set CP10 and CP11 Full Access
	__DSB();
	__ISB();

	// Initialize the C library and C++ constructors for static objects
	__libc_init_array();

	// Set up the standard clocks
	SystemCoreClockUpdate();

	// Temporarily set up systick so that delayMicroseconds works
	SysTick->LOAD = ((SystemCoreClockFreq/1000) - 1) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1 << SysTick_CTRL_ENABLE_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);

	// Initialise application, which includes setting up any additional clocks
	AppInit();

	// Run the application
	AppMain();

	while (1) { }
}

/**
  * @brief  Setup the microcontroller system.
  * @param  None
  * @retval None
  */

void SystemInit() noexcept
{
  uint32_t reg_opsr;

  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
   SCB->CPACR |= ((3UL << 20U)|(3UL << 22U));  /* set CP10 and CP11 Full Access */
  #endif

  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit while preserving the current HSIDIV value*/
  RCC->CR = (RCC->CR & RCC_CR_HSIDIV_Msk) | RCC_CR_HSION;

  /* Reset CFGR register */
  RCC->CFGR1 = 0U;
  RCC->CFGR2 = 0U;

  /* Reset HSEON, HSECSSON, HSEBYP, HSEEXT, HSIDIV, HSIKERON, CSION, CSIKERON, HSI48 and PLLxON bits */
#if defined(RCC_CR_PLL3ON)
  RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSECSSON | RCC_CR_HSEBYP | RCC_CR_HSEEXT | RCC_CR_HSIKERON | \
               RCC_CR_CSION | RCC_CR_CSIKERON |RCC_CR_HSI48ON | RCC_CR_PLL1ON | RCC_CR_PLL2ON | RCC_CR_PLL3ON);
#else
  RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSECSSON | RCC_CR_HSEBYP | RCC_CR_HSEEXT | RCC_CR_HSIKERON | \
               RCC_CR_CSION | RCC_CR_CSIKERON |RCC_CR_HSI48ON | RCC_CR_PLL1ON | RCC_CR_PLL2ON);
#endif

  /* Reset PLLxCFGR register */
  RCC->PLL1CFGR = 0U;
  RCC->PLL2CFGR = 0U;
#if defined(RCC_CR_PLL3ON)
  RCC->PLL3CFGR = 0U;
#endif /* RCC_CR_PLL3ON */

  /* Reset PLL1DIVR register */
  RCC->PLL1DIVR = 0x01010280U;
  /* Reset PLL1FRACR register */
  RCC->PLL1FRACR = 0x00000000U;
  /* Reset PLL2DIVR register */
  RCC->PLL2DIVR = 0x01010280U;
  /* Reset PLL2FRACR register */
  RCC->PLL2FRACR = 0x00000000U;
#if defined(RCC_CR_PLL3ON)
  /* Reset PLL3DIVR register */
  RCC->PLL3DIVR = 0x01010280U;
  /* Reset PLL3FRACR register */
  RCC->PLL3FRACR = 0x00000000U;
#endif /* RCC_CR_PLL3ON */

  /* Reset HSEBYP bit */
  RCC->CR &= ~(RCC_CR_HSEBYP);

  /* Disable all interrupts */
  RCC->CIER = 0U;

  /* Configure the Vector Table location add offset address ------------------*/
  uint32_t *pSrc = (uint32_t *) & _sfixed;
  SCB->VTOR = ((uint32_t) pSrc & SCB_VTOR_TBLOFF_Msk);

  /* Check OPSR register to verify if there is an ongoing swap or option bytes update interrupted by a reset */
  reg_opsr = FLASH->OPSR & FLASH_OPSR_CODE_OP;
  if ((reg_opsr == FLASH_OPSR_CODE_OP) || (reg_opsr == (FLASH_OPSR_CODE_OP_2 | FLASH_OPSR_CODE_OP_1)))
  {
    /* Check FLASH Option Control Register access */
    if ((FLASH->OPTCR & FLASH_OPTCR_OPTLOCK) != 0U)
    {
      /* Authorizes the Option Byte registers programming */
      FLASH->OPTKEYR = 0x08192A3BU;
      FLASH->OPTKEYR = 0x4C5D6E7FU;
    }
    /* Launch the option bytes change operation */
    FLASH->OPTCR |= FLASH_OPTCR_OPTSTART;

    /* Lock the FLASH Option Control Register access */
    FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;
  }
}

const uint8_t  AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
const uint8_t  APBPrescTable[8] =  {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is CSI, SystemCoreClock will contain the CSI_VALUE(*)
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(**)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(***)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(***)
  *             or HSI_VALUE(**) or CSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (*) CSI_VALUE is a constant defined in stm32h5xx_hal.h file (default value
  *             4 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSI_VALUE is a constant defined in stm32h5xx_hal.h file (default value
  *              64 MHz) but the real value may vary depending on the variations
  *              in voltage and temperature.
  *
  *         (***) HSE_VALUE is a constant defined in stm32h5xx_hal.h file (default value
  *              25 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate() noexcept
{
  uint32_t pllp, pllsource, pllm, pllfracen, hsivalue, tmp;
  float_t fracn1, pllvco;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (RCC->CFGR1 & RCC_CFGR1_SWS)
  {
  case 0x00UL:  /* HSI used as system clock source */
    SystemCoreClock = (uint32_t) (HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV)>> 3));
    break;

  case 0x08UL:  /* CSI used as system clock  source */
    SystemCoreClock = CSI_VALUE;
    break;

  case 0x10UL:  /* HSE used as system clock  source */
    SystemCoreClock = HSE_VALUE;
    break;

  case 0x18UL:  /* PLL1 used as system clock source */
    /* PLL_VCO = (HSE_VALUE or HSI_VALUE or CSI_VALUE/ PLLM) * PLLN
    SYSCLK = PLL_VCO / PLLR
    */
    pllsource = (RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1SRC);
    pllm = ((RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1M)>> RCC_PLL1CFGR_PLL1M_Pos);
    pllfracen = ((RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1FRACEN)>>RCC_PLL1CFGR_PLL1FRACEN_Pos);
    fracn1 = (float_t)(uint32_t)(pllfracen* ((RCC->PLL1FRACR & RCC_PLL1FRACR_PLL1FRACN)>> RCC_PLL1FRACR_PLL1FRACN_Pos));

    switch (pllsource)
    {
    case 0x01UL:  /* HSI used as PLL clock source */
      hsivalue = (HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV)>> 3)) ;
      pllvco = ((float_t)hsivalue / (float_t)pllm) * ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_PLL1N) + \
                (fracn1/(float_t)0x2000) +(float_t)1 );
      break;

    case 0x02UL:  /* CSI used as PLL clock source */
      pllvco = ((float_t)CSI_VALUE / (float_t)pllm) * ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_PLL1N) + \
                (fracn1/(float_t)0x2000) +(float_t)1 );
      break;

    case 0x03UL:  /* HSE used as PLL clock source */
      pllvco = ((float_t)HSE_VALUE / (float_t)pllm) * ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_PLL1N) + \
                (fracn1/(float_t)0x2000) +(float_t)1 );
      break;

    default:  /* No clock sent to PLL*/
      pllvco = (float_t) 0U;
      break;
    }

    pllp = (((RCC->PLL1DIVR & RCC_PLL1DIVR_PLL1P) >>RCC_PLL1DIVR_PLL1P_Pos) + 1U ) ;
    SystemCoreClock =  (uint32_t)(float_t)(pllvco/(float_t)pllp);

    break;

  default:
    SystemCoreClock = HSI_VALUE;
    break;
  }
  /* Compute HCLK clock frequency --------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR2 & RCC_CFGR2_HPRE) >> RCC_CFGR2_HPRE_Pos)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
}

// End
