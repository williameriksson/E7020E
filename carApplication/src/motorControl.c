#include "motorControl.h"

//int baseClock = 100;
//int motorPWMClock = baseClock - 1;
//int motorARR = (baseClock * 20) - 1;
//int motorCRR = motorARR - 1500;

void initMotorControl() {
	//Init for motor control on pin PB10 (TIM2 ch3)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //ensures clock on GPIOA is enabled
	GPIOB->MODER |= GPIO_MODER_MODER10_1; //sets GPIOA mode to alternating function
	GPIOB->AFR[1] |= (GPIO_AF1_TIM2 << 8);

	__disable_irq();
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enables TIM2 timer
	TIM2->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM2->PSC = 10-1; //sets prescalar -> clock freq 1MHz
	TIM2->ARR = 200000-1; //50Hz freq
	TIM2->CCR3 = 200000 - 15000 - 1; // CCR1 timer (for motor this determines speed (1000-2000))
	TIM2->DIER |= TIM_DIER_CC2IE; //sets the CC1IE flag for interrupt
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_0;
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1; //sets CCRM1 to mode 2... 111
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_2;
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE; //sets preload registers true, (CCR regs wont be loaded until next update event)
	TIM2->CCER |= TIM_CCER_CC3E; //capture/compare ch3 enabled

	TIM2->CR1 |= TIM_CR1_CEN;
	__enable_irq();

//	NVIC_EnableIRQ(TIM2_IRQn);
//	NVIC_SetPriority(TIM2_IRQn, 35);
}

void accelerate() {
	int pw = TIM2->ARR - TIM2->CCR3;
	pw = pw + 200;
	TIM2->CCR3 = 200000 - pw - 1;
}
void decelerate() {
	int pw = TIM2->ARR - TIM2->CCR3;
	pw = pw - 200;
	TIM2->CCR3 = 200000 - pw - 1;
}

void resetSpeed() {
	TIM2->CCR3 = 200000 - 15000 - 1;
}

//speed is between -100 (full reverse) and 100 (full forward)
void setSpeed(int speed) {
	int pw = 15000 + (speed * 50);
	TIM2->CCR3 = 200000 - pw - 1;
}

//void TIM2_IRQHandler(void) {
//	TIM2->SR &= ~(3);
//	//GPIOA->ODR ^= (1<<6);
//}
