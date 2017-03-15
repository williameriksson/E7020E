#include "hallSensor.h"

// Converts time diff (uS) between 2 magnets of the 4 magnet setup to meters per second
//#define usToMpsFourM(t) ((2.0 * 3.141592 * 0.04) / 4) * 100000 / t
#define usToMpsFourM(t) 100000000.0 / ((float)t * 4.0)

CircularBUFFER hallBuffer;
float speed = 0;

void initHallSensor() {
	__disable_irq();
	circularBufferInit(&hallBuffer, 0, 4);

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable clock GPIOB, if we need to read it, but prolly not
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB; // Set external interrupt EXTI4 for PB4
	EXTI->FTSR |= EXTI_FTSR_TR4; // Enable interrupt on falling edge for TR4
	EXTI->IMR |= EXTI_IMR_MR4; // Unmask the interrupt register for MR2 (Active for PB4)

	//RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable clock for TIM5

	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
	TIM10->ARR = 20000; // Auto reload at max
	TIM10->PSC = 10000 - 1; // Prescale to 10kHz
	TIM10->DIER |= TIM_DIER_UIE;
	TIM10->CR1 |= TIM_CR1_CEN; // Enable TIM5



	__enable_irq(); //Enable global interrupts


	NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 15); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn ); // Enable the interrupt


	NVIC_SetPriority(EXTI4_IRQn, 15); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(EXTI4_IRQn); // Enable the interrupt

}

/*
// Skissfunktion
void regulator() {
	// Trigger each 60ms maybe?
	int theReg;
	float currentValue;
	float setPoint = 0.25; // m/s
	float p = 1;
	theReg = currentvalue + p * (setPoint - speed);
}*/


void TIM1_UP_TIM10_IRQHandler () {
	if (TIM10->SR & 1) {
		speed = 0;
	}
	TIM10->SR &= ~(1);

}

void EXTI4_IRQHandler () {
	if (EXTI->PR & EXTI_PR_PR4) {	// Check interrupt flag
		uint16_t diff = TIM10->CNT;
		TIM10->CNT = 0;
		pushBuffer(&hallBuffer, diff);
		diff = getBufferAverage(&hallBuffer);
		if (diff != 0) {
			speed = (60.0f * 10000.0f) / (2.0f * (float)diff);
		}


	}
	EXTI->PR |= EXTI_PR_PR4; 		// Clear interrupt flag
}

/*
void EXTI4_IRQHandler () {
	static uint32_t startTime = 0;
	if (EXTI->PR & EXTI_PR_PR4) {	// Check interrupt flag
		uint32_t endTime = TIM5->CNT;
		uint32_t diff = getTimeDiff32b(startTime, endTime);
		startTime = endTime;
		//pushBuffer(&hallBuffer, diff);
		//speed = usToMpsFourM(getBufferAverage(&hallBuffer));
		speed = usToMpsFourM(diff);

	}
	EXTI->PR |= EXTI_PR_PR4; 		// Clear interrupt flag
}*/
