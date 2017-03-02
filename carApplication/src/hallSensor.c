#include "hallSensor.h"

// Converts time diff (uS) between 2 magnets of the 4 magnet setup to meters per second
#define usToMpsFourM(t) ((2.0 * 3.141592 * 0.05) / 4) * 1000000 / t

CircularBUFFER hallBuffer;
int speed = 0;

void initHallSensor() {
	__disable_irq();
	initBuffer(&hallBuffer, 0, 4);

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable clock GPIOB, if we need to read it, but prolly not
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB; // Set external interrupt EXTI2 for PB2
	EXTI->FTSR |= EXTI_FTSR_TR2; // Enable interrupt on falling edge for TR2
	EXTI->IMR |= EXTI_IMR_MR2; // Unmask the interrupt register for MR2 (Active for PB2)

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable clock for TIM5
	TIM5->PSC = 100-1; // Prescale to 1Mhz
	TIM5->ARR = 0xFFFF; // Auto reload at max
	TIM5->CR1 |= TIM_CR1_CEN; // Enable TIM5

	__enable_irq(); //Enable global interrupts

	NVIC_SetPriority(EXTI2_IRQn, 15); // Set the priority, this should probably be changed..
	NVIC_EnableIRQ(EXTI2_IRQn); // Enable the interrupt

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


void EXTI2_IRQHandler () {
	static int startTime = 0;
	if (EXTI->PR & EXTI_PR_PR2) {	// Check interrupt flag
		int endTime = TIM5->CNT;
		int diff = getTimeDiff16b(startTime, endTime);
		pushBuffer(&hallBuffer, diff);
		speed = usToMpsFourM(getBufferAverage(&hallBuffer));
	}
	EXTI->PR |= EXTI_PR_PR2; 		// Clear interrupt flag
}
