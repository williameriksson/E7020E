
#include "ultrasonic.h"
#include "motorControl.h"
#include "userButton.h"
#include "cargoLibs/filterUtils.h"
#include "cargoLibs/circularBuffer.h"
#include "cargoLibs/timerUtils.h"


/*
void TIM4_IRQHandler (void){

	if (TIM4->SR & TIM_SR_UIF) { //Check interrupt flag

		TIM4->SR &= ~TIM_SR_UIF; //Reset interrupt flag
		//triggerSonar();
	}
} */

void EXTI0_IRQHandler (void) {
	static int count = 0;
	static int startTime = 0;

	if (EXTI->PR & EXTI_PR_PR0) {	// Check interrupt flag for PR0
		if (GPIOB->IDR & 1) {		// Check if rising edge
			startTime = TIM3->CNT;
		} else {
			int endTime = TIM3->CNT;
			int duration = getTimeDiff16b(startTime, endTime);
			int actualSensorValue = duration * 0.0340 / 2.0;
			//distance.frontLeft = actualSensorValue;
			pushBuffer(&distanceBufferLeft, actualSensorValue);
			// Poor man's solution to avoid bad values in the beginning
			if(count >= 5) {
				distance.frontLeft = filterNoise(distance.frontLeft, &distanceBufferLeft, 500);
				//int BUFFDISTANCE = pullBuffer(&distanceBufferLeft, 0);
			} else {
				distance.frontLeft = actualSensorValue;
				count++;
			}

		}
	}

	EXTI->PR |= EXTI_PR_PR0; 		// clear interrupt flag PR0 by writing 1

}

void EXTI1_IRQHandler (void) {
	static int count = 0;
	static int startTime = 0;

	if (EXTI->PR & EXTI_PR_PR1) {	// Check interrupt flag for PR1
		if (GPIOB->IDR & 2) {		// Check if rising edge
			startTime = TIM3->CNT;
		} else {
			int endTime = TIM3->CNT;
			int duration = getTimeDiff16b(startTime, endTime);
			int actualSensorValue = duration * 0.0340 / 2.0;
			//distance.frontMiddle = actualSensorValue;
			pushBuffer(&distanceBufferMiddle, actualSensorValue);
			// Poor man's solution to avoid bad values in the beginning
			if(count >= 5) {
				distance.frontMiddle = filterNoise(distance.frontMiddle, &distanceBufferMiddle, 500);
				//int BUFFDISTANCE = pullBuffer(&distanceBufferMiddle, 0);
			} else {
				distance.frontMiddle = actualSensorValue;
				count++;
			}
		}
	}

	EXTI->PR |= EXTI_PR_PR1; 		// clear interrupt flag PR1 by writing 1
}

void EXTI2_IRQHandler (void) {
	static int count = 0;
	static int startTime = 0;

	if (EXTI->PR & EXTI_PR_PR2) {	// Check interrupt flag for PR2
		if (GPIOB->IDR & 4) {		// Check if rising edge
			startTime = TIM3->CNT;
		} else {
			int endTime = TIM3->CNT;
			int duration = getTimeDiff16b(startTime, endTime);
			int actualSensorValue = duration * 0.0340 / 2.0;
			pushBuffer(&distanceBufferRight, actualSensorValue);
			// Poor man's solution to avoid bad values in the beginning
			if(count >= 5) {
				distance.frontRight = filterNoise(distance.frontRight, &distanceBufferRight, 500);
				//int BUFFDISTANCE = pullBuffer(&distanceBufferRight, 0);
			} else {
				distance.frontRight = actualSensorValue;
				count++;
			}
		}
	}

	EXTI->PR |= EXTI_PR_PR2; 		// clear interrupt flag PR2 by writing 1
}

void EXTI4_IRQHandler (void) {
	static int count = 0;
	static int startTime = 0;

	if (EXTI->PR & EXTI_PR_PR4) {	// Check interrupt flag for PR4
		if (GPIOC->IDR & 16) {		// Check if rising edge
			startTime = TIM3->CNT;
		} else {
			int endTime = TIM3->CNT;
			int duration = getTimeDiff16b(startTime, endTime);
			int actualSensorValue = duration * 0.0340 / 2.0;
			pushBuffer(&distanceBufferBack, actualSensorValue);
			// Poor man's solution to avoid bad values in the beginning
			if(count >= 5) {
				distance.back = filterNoise(distance.back, &distanceBufferBack, 500);
				//int BUFFDISTANCE = pullBuffer(&distanceBufferRight, 0);
			} else {
				distance.back = actualSensorValue;
				count++;
			}
		}
	}

	EXTI->PR |= EXTI_PR_PR4; // clear interrupt flag PR4 by writing 1
}




//trigger running on PB6,7,8 (TIM4 ch 1,2,3)
//
void initUltrasonic (void) {
	__disable_irq(); //Disable global interrupts while setting TIM4 up
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Enable clock for TIM4
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable clock for TIM3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable clock port A
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable clock port B
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable clock port C


	GPIOB->MODER |= GPIO_MODER_MODER6_1; // GPIOB PB6 to Alternate function.
	GPIOB->MODER |= GPIO_MODER_MODER7_1; // GPIOB PB6 to Alternate function.
	GPIOB->MODER |= GPIO_MODER_MODER8_1; // GPIOB PB6 to Alternate function.

	GPIOB->AFR[0] |= GPIO_AF2_TIM4 << 24; //PB 6
	GPIOB->AFR[0] |= GPIO_AF2_TIM4 << 28; //PB 7
	GPIOB->AFR[1] |= GPIO_AF2_TIM4; //PB 8

	TIM4->DIER |= TIM_DIER_UIE; //Enable update interrupt event
	TIM4->PSC = 10000 - 1; //Set prescaler to get timer at 10kHz
	TIM4->ARR = 1000 - 1; //Set Auto-reload register 60000-1

	TIM4->CCMR1 |= TIM_CCMR1_OC1M; // Enable PWM mode 2 for channel 1
	TIM4->CCR1 = 990 - 1; // Set compare value for triggering high. 59990-1
	TIM4->DIER |= TIM_DIER_CC1IE; // Enable Capture/Compare 1
	TIM4->CCER |= TIM_CCER_CC1E; // 1: On - OC1 signal is output on the corresponding output pin

	TIM4->CCMR1 |= TIM_CCMR1_OC2M; // Enable PWM mode 2 for channel 2
	TIM4->CCR2 = 990 - 1; // Set compare value for triggering high. 59990-1
	TIM4->DIER |= TIM_DIER_CC2IE; // Enable Capture/Compare 2
	TIM4->CCER |= TIM_CCER_CC2E; // OC2?

	TIM4->CCMR2 |= TIM_CCMR2_OC3M; // Enable PWM mode 2 for channel 3
	TIM4->CCR3 = 990 - 1; // Set compare value for triggering high. 59990-1
	TIM4->DIER |= TIM_DIER_CC3IE; // Enable Capture/Compare 3
	TIM4->CCER |= TIM_CCER_CC3E; // OC3?

	TIM4->CR1 |= TIM_CR1_CEN; //Enable TIM4


	TIM3->PSC = 100-1; // Prescale to 1Mhz
	TIM3->ARR = 0xFFFF; // Auto reload at max
	TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock

	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB; // Set external interrupt EXTI0 for PB0
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB; // Set external interrupt EXTI1 for PB1
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB; // Set external interrupt EXTI2 for PB2
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PC; // Set external interrupt EXTI4 for PC4


	EXTI->RTSR |= EXTI_RTSR_TR0; // Enable interrupt on rising edge for TR0
	EXTI->FTSR |= EXTI_FTSR_TR0; // Enable interrupt on falling edge for TR0
	EXTI->IMR |= EXTI_IMR_MR0; // Unmask the interrupt register for MR0 (Active for PB0)

	EXTI->RTSR |= EXTI_RTSR_TR1; // Enable interrupt on rising edge for TR1
	EXTI->FTSR |= EXTI_FTSR_TR1; // Enable interrupt on falling edge for TR1
	EXTI->IMR |= EXTI_IMR_MR1; // Unmask the interrupt register for MR1 (Active for PB1)

	EXTI->RTSR |= EXTI_RTSR_TR2; // Enable interrupt on rising edge for TR2
	EXTI->FTSR |= EXTI_FTSR_TR2; // Enable interrupt on falling edge for TR2
	EXTI->IMR |= EXTI_IMR_MR2; // Unmask the interrupt register for MR2 (Active for PB2)

	EXTI->RTSR |= EXTI_RTSR_TR4; // Enable interrupt on rising edge for TR4
	EXTI->FTSR |= EXTI_FTSR_TR4; // Enable interrupt on falling edge for TR4
	EXTI->IMR |= EXTI_IMR_MR4; // Unmask the interrupt register for MR4 (Active for PC4)

	__enable_irq(); //Enable global interrupts

	NVIC_SetPriority(EXTI0_IRQn, 15); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(EXTI0_IRQn); // Enable the interrupt

	NVIC_SetPriority(EXTI1_IRQn, 14); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(EXTI1_IRQn); // Enable the interrupt

	NVIC_SetPriority(EXTI2_IRQn, 13); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(EXTI2_IRQn); // Enable the interrupt

	NVIC_SetPriority(EXTI4_IRQn, 16); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(EXTI4_IRQn); // Enable the interrupt

	circularBufferInit(&distanceBufferLeft, 0, 5);
	circularBufferInit(&distanceBufferMiddle, 0, 5);
	circularBufferInit(&distanceBufferRight, 0, 5);
	circularBufferInit(&distanceBufferBack, 0, 5);
	//NVIC_EnableIRQ(TIM4_IRQn); //Enable TIM4 interrupt handler
	//NVIC_SetPriority(TIM4_IRQn, 35); //Set interrupt priority

}




