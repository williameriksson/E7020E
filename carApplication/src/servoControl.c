#include "servoControl.h"


void initServoControl() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //ensures clock on GPIOA is enabled
	GPIOA->MODER |= GPIO_MODER_MODER5_1; //sets GPIOA mode to alternating function
	GPIOA->AFR[0] |= (GPIO_AF1_TIM2 << 20);

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enables TIM2 timer
	TIM2->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM2->PSC = 100-1; //sets prescalar -> clock freq 1MHz
	TIM2->ARR = 20000-1;
	TIM2->CCR1 = 20000 - 1500 - 1; // CCR1 timer (for servo this determines angle (1200-1800))
	TIM2->DIER |= TIM_DIER_CC1IE; //sets the CC1IE flag for interrupt
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_0;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1; //sets CCRM1 to mode 2... 111
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_2;
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE; //sets preload registers true, (CCR regs wont be loaded until next update event)
	TIM2->CCER |= 1; //capture/compare ch1 enabled

	TIM2->CR1 |= TIM_CR1_CEN;
	__enable_irq();

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 35);
}

void turnReset() {
	TIM2->CCR1 = 20000 - 1500 - 1;
}

void turnLeft() {
	int maxLeft = 1800;
	TIM2->CCR1 = 20000 - maxLeft - 1;
}

void turnRight() {
	int maxRight = 1200;
	TIM2->CCR1 = 20000 - maxRight - 1;
}

void TIM2_IRQHandler(void) {
	TIM2->SR &= ~(3);
	//GPIOA->ODR ^= (1<<5);
		//reducePulseWidth();
}
