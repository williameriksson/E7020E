#include "controller.h"
#include "hallSensor.h"
#include "motorControl.h"

float looptime = 0.1; //looptime interval in seconds

int count;
int runController = 0;
void initController() {
	count = 0;
	Kp = 0.6;
	Ki = 0.02;
	Kd = 1.2;
	//Init for controller on TIM9
	referenceSpeed = 10.0f;
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
	NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 10);
	startController();
}
//PID parameters (requires tuning)


//float Kp = 0.3;
//float Ki = 0.03;
//float Kd = 0.0;

float prevErr = 0;
float prevIntegral = 0;

void resetPID() {
	prevErr = 0;
	prevIntegral = 0;
}

void startController() {
	runController = 1;
}

void stopController() {
	runController = 0;
	resetPID();
}


//Feedback PID controller.
void controlLoop(float desiredSpeed, float currentSpeed) {
	static int prevDirection = 1;

	if (direction == 0) {
		currentSpeed = -currentSpeed;
	}

	float err = (desiredSpeed - currentSpeed);
	float derivative = err - prevErr;
	float integral = prevIntegral + err;
	float output = Kp * err + (Ki * integral * looptime) + (Kd * derivative /looptime);
	//float currentpw = TIM2->ARR - TIM2->CCR3;

	prevErr = err;
	prevIntegral = integral;
//	if(count > 20) { //2 sec delay on integral buildup
//		prevIntegral = integral;
//	}
//	else {
//		count++;
//	}
	if (direction != prevDirection) {
		//resetPID();
	}

//	if (desiredDirection != direction) {
//		output = -output;
//	}

//	if(direction != 1) { //if not forwards, it must be backwards
//		output = -output;
//	}
	accelerate(output);
}
// 200000 - 14360 - 1
// 200000 - 15145 - 1;
//void controlLoop(float desiredSpeed, float currentSpeed) {
//	static int toggle = 0;
//	if(1) { //if (toggle) {
//		TIM2->CCR3 = 200000 - 15000 - 1;
//	} else {
//		//resetSpeed();
//	}
//	toggle = !toggle;
//
//	if(direction != 1) { //if not forwards, it must be backwards
//		//output = -output;
//	}
//	//accelerate(output);
//}

void TIM1_BRK_TIM9_IRQHandler (void) {
	//get hallsensor value
	if(runController) {
		controlLoop(referenceSpeed, speed);
	}
	TIM9->SR &= ~(1); //reset the int handler
//	GPIOA->ODR ^= (1<<5);
}
