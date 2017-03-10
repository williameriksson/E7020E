
#include "ultrasonic.h"
#include "motorControl.h"
#include "userButton.h"
#include "cargoLibs/filterUtils.h"
#include "cargoLibs/circularBuffer.h"


/*
void TIM4_IRQHandler (void){

	if (TIM4->SR & TIM_SR_UIF) { //Check interrupt flag

		TIM4->SR &= ~TIM_SR_UIF; //Reset interrupt flag
		//triggerSonar();
	}
} */
int BUFFDISTANCE;

void EXTI0_IRQHandler (void) {
	static int count = 0;
	count++;
	if (EXTI->PR & EXTI_PR_PR0) {	// Check interrupt flag for PR0
		if (GPIOB->IDR & 1) {		// Check if rising edge
			TIM3->CNT = 0;			// Reset the timer
		} else {
			int duration = TIM3->CNT;
			int actualSensorValue = duration * 0.0340 / 2.0;
			pushBuffer(&distanceBuffer, actualSensorValue);
			//first 2 values of getAverage may be sheit!
			if(count > 5) {
				DISTANCE = filterNoise(DISTANCE, &distanceBuffer, 500);
				BUFFDISTANCE = pullBuffer(&distanceBuffer, 0);
				//DISTANCE = actualSensorValue;
			}
			else {
				DISTANCE = actualSensorValue;
			}
			int limit = 100;
			if(1) {
				if(DISTANCE <= limit) {
					setSpeed(0);
				}
				else if(DISTANCE > limit) {
					setSpeed(0);
				}
			}
		}
	}

	EXTI->PR |= EXTI_PR_PR0; 		// clear interrupt flag PR0 by writing 1

}

void EXTI1_IRQHandler (void) {
	if (EXTI->PR & EXTI_PR_PR1) {	// Check interrupt flag for PR1

	}
	EXTI->PR |= EXTI_PR_PR1; 		// clear interrupt flag PR1 by writing 1
}

void EXTI2_IRQHandler (void) {

	if (EXTI->PR & EXTI_PR_PR2) {	// Check interrupt flag for PR2

	}

	EXTI->PR |= EXTI_PR_PR2; 		// clear interrupt flag PR2 by writing 1
}




//trigger running on PB6,7,8 (TIM4 ch 1,2,3)
//
void initUltrasonic (void) {
	__disable_irq(); //Disable global interrupts while setting TIM4 up
//	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //Enable clock for TIM4
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Enable clock for TIM4
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable clock for TIM3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable clock port A
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable clock port B


	//GPIOA->MODER |= (1 << 10); //GPIOA 5 to output function.
	GPIOB->MODER |= GPIO_MODER_MODER6_1; // GPIOB PB6 to Alternate function.
//	GPIOA->AFR[0] |= GPIO_AF1_TIM4; // Select AF1 (TIM4 channel 1) for A0
//	GPIOA->AFR[0] |= GPIO_AF1_TIM4 << 20;

	GPIOB->AFR[0] |= GPIO_AF2_TIM4 << 24; //PB 6

	TIM4->DIER |= TIM_DIER_UIE; //Enable update interrupt event
	TIM4->PSC = 100-1; //Set prescaler to get timer at 1MHz
	TIM4->ARR = 100000 - 1; //Set Auto-reload register 60000-1
	TIM4->CCMR1 |= TIM_CCMR1_OC1M; // Enable PWM mode 2

	TIM4->CCR1 = 99900 - 1; // Set compare value for triggering high. 59990-1
	TIM4->DIER |= TIM_DIER_CC1IE; // Enable Capture/Compare 1
	TIM4->CCER |= TIM_CCER_CC1E; // 1: On - OC1 signal is output on the corresponding output pin

	TIM4->CCR2 = 99900 - 1; // Set compare value for triggering high. 59990-1
	TIM4->DIER |= TIM_DIER_CC2IE; // Enable Capture/Compare 2
	TIM4->CCER |= TIM_CCER_CC2E; // OC2?

	TIM4->CCR3 = 99900 - 1; // Set compare value for triggering high. 59990-1
	TIM4->DIER |= TIM_DIER_CC3IE; // Enable Capture/Compare 3
	TIM4->CCER |= TIM_CCER_CC3E; // OC3?

	TIM4->CR1 |= TIM_CR1_CEN; //Enable TIM4


	TIM3->PSC = 100-1; // Prescale to 1Mhz
	TIM3->ARR = 0xFFFF; // Auto reload at max (should not be reached in practice)
	TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock

	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB; // Set external interrupt EXTI0 for PB0

	EXTI->RTSR |= EXTI_RTSR_TR0; // Enable interrupt on rising edge for TR0
	EXTI->FTSR |= EXTI_FTSR_TR0; // Enable interrupt on falling edge for TR0
	EXTI->IMR |= EXTI_IMR_MR0; // Unmask the interrupt register for MR0 (Active for PB0)

	EXTI->RTSR |= EXTI_RTSR_TR1; // Enable interrupt on rising edge for TR1
	EXTI->FTSR |= EXTI_FTSR_TR1; // Enable interrupt on falling edge for TR1
	EXTI->IMR |= EXTI_IMR_MR1; // Unmask the interrupt register for MR0 (Active for PB1)

	EXTI->RTSR |= EXTI_RTSR_TR2; // Enable interrupt on rising edge for TR2
	EXTI->FTSR |= EXTI_FTSR_TR2; // Enable interrupt on falling edge for TR2
	EXTI->IMR |= EXTI_IMR_MR2; // Unmask the interrupt register for MR2 (Active for PB2)



	__enable_irq(); //Enable global interrupts

	NVIC_SetPriority(EXTI0_IRQn, 15); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(EXTI0_IRQn); // Enable the interrupt

	NVIC_SetPriority(EXTI0_IRQn, 15); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(EXTI1_IRQn); // Enable the interrupt

	NVIC_SetPriority(EXTI0_IRQn, 15); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(EXTI2_IRQn); // Enable the interrupt

	circularBufferInit(&distanceBuffer, 0, 5);
	//NVIC_EnableIRQ(TIM4_IRQn); //Enable TIM4 interrupt handler
	//NVIC_SetPriority(TIM4_IRQn, 35); //Set interrupt priority

}




