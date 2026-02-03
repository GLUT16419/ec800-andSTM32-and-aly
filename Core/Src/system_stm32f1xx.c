/**
  ******************************************************************************
  * @file    system_stm32f1xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f103xb.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *                                      
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *   After each device reset the MSI (4 MHz) is used as system clock source.
  *   Then SystemInit() function is called, in "startup_stm32f103xb.s" file,
  *   to configure the system clock before to branch to main program.
  *
  *   This file configures the system clock as follows:
  *=============================================================================*
  *=============================================================================*
  *                    Supported STM32F103xx device revision    | Rev A
  *-----------------------------------------------------------------------------*
  * System Clock source          | HSI
  *-----------------------------------------------------------------------------*
  * SYSCLK(Hz)                   | 8000000
  *-----------------------------------------------------------------------------*
  * HCLK(Hz)                     | 8000000
  *-----------------------------------------------------------------------------*
  * AHB Prescaler                | 1
  *-----------------------------------------------------------------------------*
  * APB1 Prescaler               | 1
  *-----------------------------------------------------------------------------*
  * APB2 Prescaler               | 1
  *-----------------------------------------------------------------------------*
  * USB Clock source             | Not applicable
  *-----------------------------------------------------------------------------*
  * USB Clock(Hz)                | Not applicable
  *-----------------------------------------------------------------------------*
  *-----------------------------------------------------------------------------*
  *=============================================================================*
  *=============================================================================*
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f1xx_system
  * @{
  */

/** @addtogroup STM32F1xx_System_Private_Includes
  * @{
  */

#include "stm32f1xx.h"

#if !defined  (HSE_VALUE)
  #define HSE_VALUE               ((uint32_t)8000000U) /*!< Default value of the External oscillator in Hz. 
                                                         This value can be provided and adapted by the user application. */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE               ((uint32_t)8000000U) /*!< Value of the Internal oscillator in Hz. 
                                                         This value can be provided and adapted by the user application. */
#endif /* HSI_VALUE */

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/

/* Note: Following vector table addresses must be defined in line with linker
         configuration. */
/*!< Uncomment the following line if you need to relocate the vector table
     anywhere in Flash or Sram, else the vector table is kept at the default
     position (0x08000000). */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00000000U /*!< Vector Table base offset field. 
                                   This value must be a multiple of 0x200. */

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Variables
  * @{
  */
  
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock;
               then there is no need to call the 2 first functions listed above, 
               since SystemCoreClock variable is updated automatically.
  */
  uint32_t SystemCoreClock = HSI_VALUE;

  const uint8_t AHBPrescTable[16U] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
  const uint8_t APBPrescTable[8U]  = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_FunctionPrototypes
  * @{
  */

#ifdef  USE_FULL_ASSERT
  /**
    * @brief  Reports the name of the source file and the source line number
    *         where the assert_param error has occurred.
    * @param  file: pointer to the source file name
    * @param  line: assert_param error line source number
    * @retval None
    */
  void assert_failed(uint8_t *file, uint32_t line);
#endif

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the 
  *         SystemCoreClock variable.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
  /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001U;

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
  #ifndef STM32F105xC
    #ifndef STM32F107xC
      RCC->CFGR &= (uint32_t)0xF8FF0000U;
    #else
      RCC->CFGR &= (uint32_t)0xF0FF0000U;
    #endif /* STM32F107xC */
  #else
    RCC->CFGR &= (uint32_t)0xF0FF0000U;
  #endif /* STM32F105xC */
  
  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFFU;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFFU;

  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CFGR &= (uint32_t)0xFF80FFFFU;

#ifdef STM32F105xC
  /* Reset PLL2ON and PLL3ON bits */
  RCC->CR &= (uint32_t)0xEBFFFFFFU;

  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x00FF0000U;

  /* Reset CFGR2 register */
  RCC->CFGR2 = 0x00000000U;
#elif defined (STM32F107xC)
  /* Reset PLL2ON and PLL3ON bits */
  RCC->CR &= (uint32_t)0xEBFFFFFFU;

  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x00FF0000U;

  /* Reset CFGR2 register */
  RCC->CFGR2 = 0x00000000U;
#else
  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000U;
#endif /* STM32F105xC */

#if defined (DATA_IN_ExtSRAM)
  SystemInit_ExtMemCtl(); 
#endif /* DATA_IN_ExtSRAM */

  /* Configure the Vector Table location -------------------------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif /* VECT_TAB_SRAM */
}

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
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*) 
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied by the PLL factors.
  *
  *         (*) HSI_VALUE is a constant defined in stm32f1xx.h file (default value
  *             8 MHz)
  *
  *         (**) HSE_VALUE is a constant defined in stm32f1xx.h file (default value
  *              8 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *   - The result of this function could be not correct when using fractional
  *     value for HSE crystal. 
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0U, pllmull = 0U, pllsource = 0U;

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;
  
  switch (tmp)
  {
    case 0x00U:  /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04U:  /* HSE used as system clock */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08U:  /* PLL used as system clock */

      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
      pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
      pllmull = ( pllmull >> 18U) + 2U;

      if (pllsource == 0x00U)
      {
        /* HSI oscillator clock divided by 2 selected as PLL clock entry */
        SystemCoreClock = (HSI_VALUE >> 1U) * pllmull;
      }
      else
      {
        /* HSE selected as PLL clock entry */
        #ifdef STM32F105xC
          #ifdef STM32F107xC
            tmp = RCC->CFGR2 & RCC_CFGR2_PREDIV1;
            switch (tmp)
            {
              case 0x00U:
                SystemCoreClock = HSE_VALUE * pllmull;
                break;
              default:
                SystemCoreClock = (HSE_VALUE / ((tmp >> 4U) + 1U)) * pllmull;
                break;
            }
          #else
            SystemCoreClock = HSE_VALUE * pllmull;
          #endif /* STM32F107xC */
        #else
          SystemCoreClock = HSE_VALUE * pllmull;
        #endif /* STM32F105xC */
      }
      break;
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK clock frequency --------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4U)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number, 
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1U)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
