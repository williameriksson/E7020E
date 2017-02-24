#include "motorControl.h"


void initMotorControl() {
	//Init for motor control on pin PB7 (tim4 ch2)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //ensures clock on GPIOA is enabled
	GPIOB->MODER |= GPIO_MODER_MODER7_1; //sets GPIOA mode to alternating function
	GPIOB->AFR[0] |= (GPIO_AF2_TIM4 << 28);

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //enables TIM4 timer
	TIM4->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM4->PSC = 100-1; //sets prescalar -> clock freq 1MHz
	TIM4->ARR = 20000-1; //50Hz freq
	TIM4->CCR2 = 20000 - 1500 - 1; // CCR1 timer (for motor this determines speed (1000-2000))
	TIM4->DIER |= TIM_DIER_CC2IE; //sets the CC1IE flag for interrupt
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_0;
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_1; //sets CCRM1 to mode 2... 111
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_2;
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE; //sets preload registers true, (CCR regs wont be loaded until next update event)
	TIM4->CCER |= 16; //capture/compare ch2 enabled

	TIM4->CR1 |= TIM_CR1_CEN;
	__enable_irq();

//	NVIC_EnableIRQ(TIM4_IRQn);
//	NVIC_SetPriority(TIM4_IRQn, 35);
}

void accelerate() {
	int pw = TIM4->ARR - TIM4->CCR2;
	pw = pw + 20;
	TIM4->CCR2 = 20000 - pw - 1;
}
void decelerate() {
	int pw = TIM4->ARR - TIM4->CCR2;
	pw = pw - 20;
	TIM4->CCR2 = 20000 - pw - 1;
}

void resetSpeed() {
	TIM4->CCR2 = 20000 - 1500 - 1;
}

//void TIM4_IRQHandler(void) {
//	TIM4->SR &= ~(3);
//	//GPIOA->ODR ^= (1<<6);
//}
