#include "servoControl.h"


void initServoControl() {
	//Init for servo control on pin PB3 (TIM2 ch2)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //ensures clock on GPIOB is enabled
	GPIOB->MODER |= GPIO_MODER_MODER3_1; //sets GPIOB mode to alternating function
	GPIOB->AFR[0] |= (GPIO_AF1_TIM2 << 12);

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enables TIM2 timer
	TIM2->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM2->PSC = 10-1; //sets prescalar -> clock freq 1MHz
	TIM2->ARR = 200000-1;
	TIM2->CCR2 = 200000 - 15000 - 1; // CCR2 timer (for servo this determines angle (1200-1800))
	TIM2->DIER |= TIM_DIER_CC2IE; //sets the CC1IE flag for interrupt
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_0;
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_1; //sets CCRM1 to mode 2... 111
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2;
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE; //sets preload registers true, (CCR regs wont be loaded until next update event)
	TIM2->CCER |= TIM_CCER_CC2E; //capture/compare ch2 enabled

	TIM2->CR1 |= TIM_CR1_CEN;
	__enable_irq();

//	NVIC_EnableIRQ(TIM2_IRQn);
//	NVIC_SetPriority(TIM2_IRQn, 35);
}

void turnReset() {
	TIM2->CCR2 = 200000 - 15000 - 1;
}

void turnHalfLeft() {
	int maxLeft = 16000;
	TIM2->CCR2 = 200000 - maxLeft - 1;
}

void turnHalfRight() {
	int maxRight = 14000;
	TIM2->CCR2 = 200000 - maxRight - 1;
}


void turnLeft() {
	int maxLeft = 17000;
	TIM2->CCR2 = 200000 - maxLeft - 1;
}

void turnRight() {
	int maxRight = 13000;
	TIM2->CCR2 = 200000 - maxRight - 1;
}
