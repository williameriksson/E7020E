
#include "ultrasonic.h"

float DISTANCE;


/*
void TIM2_IRQHandler (void){

	if (TIM2->SR & TIM_SR_UIF) { //Check interrupt flag

		TIM2->SR &= ~TIM_SR_UIF; //Reset interrupt flag
		//triggerSonar();
	}
} */

void EXTI0_IRQHandler (void) {

	if (EXTI->PR & EXTI_PR_PR0) {	// Check interrupt flag for PR0
		if (GPIOB->IDR & 1) {		// Check if rising edge
			TIM3->CNT = 0;			// Reset the timer
		} else {
			int duration = TIM3->CNT;
			DISTANCE = duration * 0.0340 / 2.0;
		}
		/*if (times >= 3) {
			DISTANCE = tempDist / (float)times;
			tempDist = 0;
			times = 0;
		}*/
	}

	EXTI->PR |= EXTI_PR_PR0; 		// clear interrupt flag PR0 by writing 1

}


void initUltrasonic (void) {
	__disable_irq(); //Disable global interrupts while setting TIM2 up
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //Enable clock for TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable clock for TIM3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable clock port A
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable clock port B


	//GPIOA->MODER |= (1 << 10); //GPIOA 5 to output function.
	GPIOA->MODER |= (1 << 1); // GPIOA 0 to Alternate function.
	GPIOA->AFR[0] |= GPIO_AF1_TIM2; // Select AF1 (TIM2 channel 1) for A0

	TIM2->DIER |= TIM_DIER_UIE; //Enable update interrupt event
	TIM2->PSC = 100-1; //Set prescaler to get timer at 1MHz
	TIM2->ARR = 100000 - 1; //Set Auto-reload register 60000-1
	TIM2->CCMR1 |= TIM_CCMR1_OC1M; // Enable PWM mode 2
	TIM2->CCR1 = 99900 - 1; // Set compare value for triggering high. 59990-1
	TIM2->DIER |= TIM_DIER_CC1IE; // Enable Capture/Compare 1
	TIM2->CCER |= TIM_CCER_CC1E; // 1: On - OC1 signal is output on the corresponding output pin
	TIM2->CR1 |= TIM_CR1_CEN; //Enable TIM2


	TIM3->PSC = 100-1; // Prescale to 1Mhz
	TIM3->ARR = 0xFFFF;
	TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3


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




