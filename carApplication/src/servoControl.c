#include "servoControl.h"


void initServoControl() {
	//Init for servo control on pin PB6 (tim4 ch1)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //ensures clock on GPIOB is enabled
	GPIOB->MODER |= GPIO_MODER_MODER6_1; //sets GPIOB mode to alternating function
	GPIOB->AFR[0] |= (GPIO_AF2_TIM4 << 24);

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //enables TIM2 timer
	TIM4->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM4->PSC = 100-1; //sets prescalar -> clock freq 1MHz
	TIM4->ARR = 20000-1;
	TIM4->CCR1 = 20000 - 1500 - 1; // CCR1 timer (for servo this determines angle (1200-1800))
	TIM4->DIER |= TIM_DIER_CC1IE; //sets the CC1IE flag for interrupt
	TIM4->CCMR1 |= TIM_CCMR1_OC1M_0;
	TIM4->CCMR1 |= TIM_CCMR1_OC1M_1; //sets CCRM1 to mode 2... 111
	TIM4->CCMR1 |= TIM_CCMR1_OC1M_2;
	TIM4->CCMR1 |= TIM_CCMR1_OC1PE; //sets preload registers true, (CCR regs wont be loaded until next update event)
	TIM4->CCER |= 1; //capture/compare ch1 enabled

	TIM4->CR1 |= TIM_CR1_CEN;
	__enable_irq();

//	NVIC_EnableIRQ(TIM2_IRQn);
//	NVIC_SetPriority(TIM2_IRQn, 35);
}

void turnReset() {
	TIM4->CCR1 = 20000 - 1500 - 1;
}

void turnLeft() {
	int maxLeft = 1700;
	TIM4->CCR1 = 20000 - maxLeft - 1;
}

void turnRight() {
	int maxRight = 1300;
	TIM4->CCR1 = 20000 - maxRight - 1;
}
