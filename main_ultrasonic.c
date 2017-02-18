/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c 
  * @author  MCD Application Team
  * @version V1.1.3
  * @date    13-November-2015
  * @brief   This example describes how to configure and use GPIOs through 
  *          the STM32F4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static GPIO_InitTypeDef  GPIO_InitStruct;


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

float DISTANCE;

/*
void waitMicroS(int uSec) {
	TIM3->CNT = 0;
	while (TIM3->CNT <= uSec){};
}*/


/*
void triggerSonar(void) {
	GPIOA->ODR &= ~(1<<5);	//Pull-low for clean trigger
	waitMicroS(2);
	GPIOA->ODR |= (1<<5);	//Start trigger
	waitMicroS(10);
	GPIOA->ODR &= ~(1<<5);	//End trigger
}*/

/*
void TIM2_IRQHandler (void){

	if (TIM2->SR & TIM_SR_UIF) { //Check interrupt flag

		TIM2->SR &= ~TIM_SR_UIF; //Reset interrupt flag
		//triggerSonar();
	}
} */

void EXTI0_IRQHandler (void) {

	// TODO: here we should check the interrupt flag
	//static float tempDist = 0;
	//static int times = 0;

	//times++;
	if (GPIOB->ODR & 1) {
		TIM3->CNT = 0; // Reset the timer
	} else {
		int duration = TIM3->CNT;
		//range = high level time * velocity (340M/S) / 2;
		//tempDist += (float)duration * 340 / ( 1000000 * (float)2.0);
		DISTANCE = (float)duration * (float)340.0 / ( (float)1000000.0 * (float)2.0);
	}

	/*if (times >= 3) {

		DISTANCE = tempDist / (float)times;
		tempDist = 0;
		times = 0;
	}*/


	EXTI->PR = 1; // clear interrupt flag PR0

}

void initSetup (void) {
	__disable_irq(); //Disable global interrupts while setting TIM2 up
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //Enable clock for TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable clock for TIM3
	// RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Enable clock for TIM4
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable clock port A


	GPIOA->MODER |= (1 << 10); //GPIOA 5 to output function.
	GPIOA->MODER |= (1 << 1); // GPIOA 0 to Alternate function.
	GPIOA->AFR[0] |= GPIO_AF1_TIM2; // Select AF1 (TIM2 channel 1) for A0

	TIM2->DIER |= TIM_DIER_UIE; //Enable update interrupt event
	TIM2->PSC = 50-1; //Set prescaler to get timer at 1MHz
	TIM2->ARR = 1000000 - 1; //Set Auto-reload register 60000-1
	TIM2->CCMR1 |= TIM_CCMR1_OC1M; // Enable PWM mode 2
	TIM2->CCR1 = 999980 - 1; // Set compare value for triggering high. 59990-1
	TIM2->DIER |= TIM_DIER_CC1IE; // Enable Capture/Compare 1
	TIM2->CCER |= TIM_CCER_CC1E; // 1: On - OC1 signal is output on the corresponding output pin
	TIM2->CR1 |= TIM_CR1_CEN; //Enable TIM2



	TIM3->PSC = 50-1; // Prescale to 1Mhz

	//TODO: Change the measurement by using the built in time pulse length feature
	TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3

	// TIM4->PSC = 50-1; // Prescale to 1Mhz


	// Input mode is the reset state, so we don't care about setting it.
	//GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0; // Set pin A0? as pull-up
	//GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH; // Set GPIO A0? speed very high, change?

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock
	SYSCFG->EXTICR[0] |= 1; // Set external interrupt EXTI0 for PB0
	EXTI->RTSR |= EXTI_RTSR_TR0; // Enable interrupt on rising edge
	EXTI->FTSR |= EXTI_FTSR_TR0; // Enable interrupt on falling edge
	EXTI->IMR |= EXTI_IMR_MR0; // Mask the interrupt register




	__enable_irq(); //Enable global interrupts

	NVIC_SetPriority(EXTI0_IRQn, 15); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(EXTI0_IRQn); // Enable the interrupt

	//NVIC_EnableIRQ(TIM2_IRQn); //Enable TIM2 interrupt handler
	//NVIC_SetPriority(TIM2_IRQn, 35); //Set interrupt priority

}



int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 100 MHz */
  SystemClock_Config();
  
  /*##-1- Enable GPIOA Clock (to be able to program the configuration registers) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /*##-2- Configure PA05 IO in output push-pull mode to drive external LED ###*/  
  /*GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
  */
  /*##-3- Toggle PA05 IO in an infinite loop #################################*/  



	initSetup();

	while (1){} //Infinte loop



}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
