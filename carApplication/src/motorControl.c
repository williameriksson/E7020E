#include "motorControl.h"


void initMotorControl() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //ensures clock on GPIOA is enabled
	GPIOA->MODER |= GPIO_MODER_MODER6_1; //sets GPIOA mode to alternating function
	GPIOA->AFR[0] |= (GPIO_AF2_TIM3 << 24);

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //enables TIM3 timer
	TIM3->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM3->PSC = 100-1; //sets prescalar -> clock freq 1MHz
	TIM3->ARR = 20000-1; //50Hz freq
	TIM3->CCR1 = 20000 - 1500 - 1; // CCR1 timer (for servo this determines angle (1200-1800))
	TIM3->DIER |= TIM_DIER_CC1IE; //sets the CC1IE flag for interrupt
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_0;
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1; //sets CCRM1 to mode 2... 111
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2;
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE; //sets preload registers true, (CCR regs wont be loaded until next update event)
	TIM3->CCER |= 1; //capture/compare ch1 enabled

	TIM3->CR1 |= TIM_CR1_CEN;
	__enable_irq();

	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 35);
}

void accelerate() {
	int pw = TIM3->ARR - TIM3->CCR1;
	pw = pw + 20;
	TIM3->CCR1 = 20000 - pw - 1;
}
void decelerate() {
	int pw = TIM3->ARR - TIM3->CCR1;
	pw = pw - 20;
	TIM3->CCR1 = 20000 - pw - 1;
}

void resetSpeed() {
	TIM3->CCR1 = 20000 - 1500 - 1;
}

void TIM3_IRQHandler(void) {
	TIM3->SR &= ~(3);
	//GPIOA->ODR ^= (1<<6);
}
