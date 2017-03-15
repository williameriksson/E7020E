#include "controller.h"

float looptime = 0.1; //looptime interval in seconds

void initController() {
	//Init for controller on TIM9
	testInput = 0;
	__disable_irq();
	//for testing
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enable GPIOA
//	GPIOA->MODER |= (1<<10); //GPIOA 5 to General purpose output.

	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; //enables TIM9 timer
	TIM9->DIER |= TIM_DIER_UIE; //enables update interrupts
	TIM9->PSC = 1000-1; //sets prescalar -> clock freq 100kHz
	TIM9->ARR = 10000-1; //100ms delay loop
	TIM9->CR1 |= TIM_CR1_CEN;
	__enable_irq();

	NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 20);
}
//PID parameters (requires tuning)
float Kp = 1;
float Kd = 1;
float Ki = 1;

float prevErr = 0;
float prevIntegral = 0;

//Feedback PID controller.
void controlLoop(float desiredSpeed, float currentSpeed) {
	float err = (desiredSpeed - currentSpeed);
	float derivative = err - prevErr;
	float integral = prevIntegral + err;
	float output = Kp * err + (Ki * integral * looptime) + (Kd * derivative /looptime);

	prevErr = err;
	prevIntegral = integral;
	//TODO: figure out a way to translate speed (from hallsensor) into the PWM signal.
}

void TIM1_BRK_TIM9_IRQHandler (void) {
	//get hallsensor value
	float velocity = 1;
	controlLoop(0, testInput);
	TIM9->SR &= ~(1); //reset the int handler
	GPIOA->ODR ^= (1<<5);
}
