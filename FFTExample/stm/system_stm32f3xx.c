/**
  ******************************************************************************
  * @file    system_stm32f3xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
  *
  * 1. This file provides two functions and one global variable to be called from
  *    user application:
  *      - SystemInit(): This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f3xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  * 2. After each device reset the HSI (8 MHz) is used as system clock source.
  *    Then SystemInit() function is called, in "startup_stm32f3xx.s" file, to
  *    configure the system clock before to branch to main program.
  *
  * 3. This file configures the system clock as follows:
  *=============================================================================
  *                         Supported STM32F3xx device
  *-----------------------------------------------------------------------------
  *        System Clock source                    | PLL
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 8000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 72000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 2
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 1
  *-----------------------------------------------------------------------------
  *        USB Clock                              | DISABLE
  *-----------------------------------------------------------------------------
  *=============================================================================
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f3xx_system
  * @{
  */

/** @addtogroup STM32F3xx_System_Private_Includes
  * @{
  */

#include "stm32f3xx.h"
#include "main.h"

/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_Defines
  * @{
  */
#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)8000000) /*!< Default value of the External oscillator in Hz.
                                                This value can be provided and adapted by the user application. */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)8000000) /*!< Default value of the Internal oscillator in Hz.
                                                This value can be provided and adapted by the user application. */
#endif /* HSI_VALUE */

#define PWM

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x0 /*!< Vector Table base offset field.
                                  This value must be a multiple of 0x200. */
/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_Variables
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock there is no need to
               call the 2 first functions listed above, since SystemCoreClock variable is 
               updated automatically.
  */
uint32_t SystemCoreClock = 8000000;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F3xx_System_Private_Functions
  * @{
  */

static void spinDelay(uint32_t time) {
	uint32_t then = TIM2->CNT + time;
	while (TIM2->CNT < then);
}

__STATIC_FORCEINLINE void initADC() {
	uint32_t temp;
	// Enable clock to ADC12 and reset it
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	__DSB();
	temp = RCC->AHBRSTR & ~RCC_AHBRSTR_ADC12RST;
	RCC->AHBRSTR = temp | RCC_AHBRSTR_ADC12RST;
	__DSB();
	RCC->AHBRSTR = temp;
	// Enable ADRDY interrupt
	// Unfortunately no interrupt is available for calibration complete/VREG ready
	ADC1->IER = ADC_IER_ADRDYIE;
	// Start up the voltage regulator (must transition through the 0x00 state)
	ADC1->CR = 0U;
	__DSB();
	ADC1->CR = ADC_CR_ADVREGEN_0;
	// 10 us minimum VREG startup time
	spinDelay(16U);
	// Launch calibration (single ended mode) and wait for completion
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL);
	// Configure 12 bit resolution, DMA enable in circular mode
	ADC1->CFGR = ADC_CFGR_DMAEN | ADC_CFGR_DMACFG | ADC_CFGR_CONT;
	// PA3 (ADC1 IN4 / Arduino A0 on the nucleo ZE board) is our target
	ADC1->SQR1 = (ADC_SQR1_L_Msk & 0U) | (4U << ADC_SQR1_SQ1_Pos);
	// Sampling time 2.5 cycles (total time at 12 bit = 15 cycles)
	ADC1->SMPR1 = ADC_SMPR1_SMP4_0;
	// Enable the ADC, interrupt will start conversion
	ADC1->CR |= ADC_CR_ADEN;
}

__STATIC_FORCEINLINE void initClocks() {
	uint32_t temp;
	// Clear interrupt pending bits
	RCC->CIR = RCC_CIR_CSSC | RCC_CIR_PLLRDYC | RCC_CIR_HSERDYC | RCC_CIR_HSIRDYC |
		RCC_CIR_LSERDYC | RCC_CIR_LSIRDYC;
	// Reset Sleep Control register to zero to avoid unwanted deep sleep
	SCB->SCR = 0x00000000;
	// Turn on the HSE (8 MHz)
	temp = RCC->CR;
	temp |= RCC_CR_HSEON;
	// TODO Set HSE to bypass since we are using the nucleo which has an 8MHz square clock
#ifdef HSE
	temp &= ~RCC_CR_HSEBYP;
#else
	temp |= RCC_CR_HSEBYP;
#endif
	RCC->CR = temp;
	RCC->CFGR &= ~RCC_CFGR_SW;
	// Wait for HSE to start up
	while (!(RCC->CR & RCC_CR_HSERDY));
	temp = FLASH->ACR;
	temp &= ~FLASH_ACR_LATENCY;
	// Prefetch buffer on, 2 wait states (reason for the prefetch buffer)
	temp |= FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE;
	FLASH->ACR = temp;
	// APB1 is 36 MHz and APB2 is 72 MHz
	// ADC clock is 72 MHz
	// PLL enabled from HSE = 8 MHz * 9 = 72 MHz
	RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLMUL9 | RCC_CFGR_PLLSRC_HSE_PREDIV;
	// Turn PLL on
	RCC->CR |= RCC_CR_PLLON;
	// Wait for PLL to start up
	while (!(RCC->CR & RCC_CR_PLLRDY));
	// Select PLL as system clock
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	// Wait for system clock to become the PLL
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	// ADC1/2 clock = PLL / 1
	RCC->CFGR2 = RCC_CFGR2_ADCPRE12_4;
}

__STATIC_FORCEINLINE void initDMA() {
	// Enable clock to DMA1
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	__DSB();
	// Clear DMA interrupts
	DMA1->IFCR = 0x0FFFFFFFU;
	// ADC is on channel 1 of the DMA, highest priority, 16 bit data size, circular mode
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)&adcResults;
	DMA1_Channel1->CNDTR = (uint32_t)ADC_BUFFER_SIZE;
	DMA1_Channel1->CCR = DMA_CCR_PL_0 | DMA_CCR_PL_1 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 |
		DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_EN;
}

__STATIC_FORCEINLINE void initInterrupts() {
	NVIC_SetPriorityGrouping(3U);
	// SysTick fires every 9000 clock cycles (72M / 8 / 9K = 1K/s = 1 ms)
	SysTick->LOAD = 8999;
	// Reset counter to zero
	SysTick->VAL = 0;
	// Enable DMA1 channel 1 with the highest priority
	NVIC_SetPriority(DMA1_Channel1_IRQn, 0U);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	// Enable ADC1 with high priority (should only trigger once)
	NVIC_SetPriority(ADC1_2_IRQn, 1U);
	NVIC_EnableIRQ(ADC1_2_IRQn);
	// Enable USART3 with medium priorty
	NVIC_SetPriority(USART3_IRQn, 3U);
	NVIC_EnableIRQ(USART3_IRQn);
	// Turn on, enable interrupt, select clock / 8
	SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

__STATIC_FORCEINLINE void initPorts() {
	uint32_t temp;
	// Enable clocks to all I/O ports that are used
	RCC->AHBENR |= RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN;
	__DSB();
	// Reset I/O ports
	temp = RCC->AHBRSTR & ~(RCC_AHBRSTR_GPIODRST | RCC_AHBRSTR_GPIOBRST | RCC_AHBRSTR_GPIOARST);
	RCC->AHBRSTR = temp | (RCC_AHBRSTR_GPIODRST | RCC_AHBRSTR_GPIOBRST | RCC_AHBRSTR_GPIOARST);
	__DSB();
	RCC->AHBRSTR = temp;
	// Set up GPIOD 8 and 9 as USART3 TX/RX
	ioSetDirection(GPIOD, 8, DDR_AFO_PU);
	ioSetDirection(GPIOD, 9, DDR_INPUT_PULLUP);
	GPIOD->AFR[1] = (7U << GPIO_AFRH_AFRH1_Pos) | (7U << GPIO_AFRH_AFRH0_Pos);
	// Set up LEDs
	ioSetOutput(PIN_LED_1, false);
	ioSetDirection(PIN_LED_1, DDR_OUTPUT);
	ioSetOutput(PIN_LED_2, false);
	ioSetDirection(PIN_LED_2, DDR_OUTPUT);
	ioSetOutput(PIN_LED_3, false);
	ioSetDirection(PIN_LED_3, DDR_OUTPUT);
#ifdef PWM
	ioSetDirection(GPIOB, 1, DDR_AFO);
	GPIOB->AFR[0] = (2U << GPIO_AFRL_AFRL1_Pos);
#endif
}

__STATIC_FORCEINLINE void initSerial() {
	uint32_t temp;
	// Enable clocks to USART3
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	__DSB();
	// Reset USART3
	temp = RCC->APB1RSTR & ~(RCC_APB1RSTR_USART3RST);
	RCC->APB1RSTR = temp | (RCC_APB1RSTR_USART3RST);
	__DSB();
	RCC->APB1RSTR = temp;
	USART3->CR2 = 0U;
	USART3->CR3 = 0U;
	USART3->BRR = 313U;
	// Eight-N-One, nil parity or flow control
	USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
}

__STATIC_FORCEINLINE void initTIM() {
	uint32_t temp;
	// (RM0316 figure 13) TIM2-7 by default get an x2 multiplier from APB1 speed so they
	// actually run at 72 MHz, not 36 MHz
	// Enable clocks to TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	__DSB();
	// Reset TIM2
	temp = RCC->APB1RSTR & ~(RCC_APB1RSTR_TIM2RST);
	RCC->APB1RSTR = temp | (RCC_APB1RSTR_TIM2RST);
	__DSB();
	RCC->APB1RSTR = temp;
	// TIM2 is the 1us resolution timer used for timestamping; 2 us at 32 bits will last for
	// almost 1.2 hours before overflowing
	TIM2->PSC = 71U;
	TIM2->ARR = 0xFFFFFFFFU;
	// No DMA, no interrupts
	TIM2->DIER = 0U;
	TIM2->CR2 = 0U;
	TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
#ifdef PWM
	// Enable clocks to TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	__DSB();
	// Reset TIM3
	temp = RCC->APB1RSTR & ~(RCC_APB1RSTR_TIM3RST);
	RCC->APB1RSTR = temp | (RCC_APB1RSTR_TIM3RST);
	__DSB();
	RCC->APB1RSTR = temp;
	// 562.5k
	TIM3->PSC = 31U;
	// 50% duty cycle, very high speed
	TIM3->ARR = 0x3U;
	TIM3->CCMR1 = 0U;
	// Channel 4 runs in PWM mode 1
	TIM3->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;
	TIM3->CCR4 = 0x2U;
	TIM3->CCER = TIM_CCER_CC4E;
	// No DMA, no interrupts
	TIM3->DIER = 0U;
	TIM3->CR2 = 0U;
	TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
#endif
}

/**
  * @brief  Setup the microcontroller system
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
/* FPU settings --------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
#endif

#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

	initClocks();
	initPorts();
	initSerial();
	initTIM();
	initDMA();
	initADC();
	initInterrupts();
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
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (*) HSI_VALUE is a constant defined in stm32f3xx_hal.h file (default value
  *             8 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSE_VALUE is a constant defined in stm32f3xx_hal.h file (default value
  *              8 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0, predivfactor = 0;

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case RCC_CFGR_SWS_HSI:  /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
    case RCC_CFGR_SWS_HSE:  /* HSE used as system clock */
      SystemCoreClock = HSE_VALUE;
      break;
    case RCC_CFGR_SWS_PLL:  /* PLL used as system clock */
      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmull = RCC->CFGR & RCC_CFGR_PLLMUL;
      pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
      pllmull = ( pllmull >> 18) + 2;

#if defined (STM32F302xE) || defined (STM32F303xE) || defined (STM32F398xx)
        predivfactor = (RCC->CFGR2 & RCC_CFGR2_PREDIV) + 1;
      if (pllsource == RCC_CFGR_PLLSRC_HSE_PREDIV)
      {
        /* HSE oscillator clock selected as PREDIV1 clock entry */
        SystemCoreClock = (HSE_VALUE / predivfactor) * pllmull;
      }
      else
      {
        /* HSI oscillator clock selected as PREDIV1 clock entry */
        SystemCoreClock = (HSI_VALUE / predivfactor) * pllmull;
      }
#else      
      if (pllsource == RCC_CFGR_PLLSRC_HSI_DIV2)
      {
        /* HSI oscillator clock divided by 2 selected as PLL clock entry */
        SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
      }
      else
      {
        predivfactor = (RCC->CFGR2 & RCC_CFGR2_PREDIV) + 1;
        /* HSE oscillator clock selected as PREDIV1 clock entry */
        SystemCoreClock = (HSE_VALUE / predivfactor) * pllmull;
      }
#endif /* STM32F302xE || STM32F303xE || STM32F398xx */
      break;
    default: /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK clock frequency ----------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

