#include "userButton.h"

void initUserButton() {
	__disable_irq();
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	//GPIOC->PUPDR |= GPIO_PUPDR_PUPDR13_1;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
	EXTI->FTSR |= EXTI_FTSR_TR13; //sets interrupt on falling edge
	EXTI->IMR |= EXTI_IMR_MR13; //sets mask.

	__enable_irq();

	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn, 40);
}

void EXTI15_10_IRQHandler(void) {
	EXTI->PR |= EXTI_PR_PR13;
	accelerate();
}
