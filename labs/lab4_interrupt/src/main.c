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

/**
 * Setup for TIM2
 */

void initSetup (void) {
	__disable_irq(); //Disable global interrupts while setting TIM2 up
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //Enable clock for TIM2
	TIM2->DIER |= TIM_DIER_UIE; //Enable update interrupt event
	TIM2->PSC = 50-1; //Set prescaler to get timer at 1MHz
	TIM2->ARR = 1000000-1; //Set Auto-reload register

	TIM2->CR1 |= TIM_CR1_CEN; //Enable TIM2
	__enable_irq(); //Enable global interrupts

	NVIC_EnableIRQ(TIM2_IRQn); //Enable TIM2 interrupt handler
	NVIC_SetPriority(TIM2_IRQn, 35); //Set interrupt priority
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

int main(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enable GPIOA
	GPIOA->MODER |= (1<<10); //GPIOA 5 to General purpose output.
	initSetup();
	while (1){} //Infinte loop


}

void TIM2_IRQHandler (void){

	if (TIM2->SR & TIM_SR_UIF) { //Check interrupt flag

		TIM2->SR &= ~TIM_SR_UIF; //Reset interrupt flag
		GPIOA->ODR ^= (1<<5);	//Toggle LED
	}
}
