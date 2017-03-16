
#include "main.h"
#include "ultrasonic.h"
#include "motorControl.h"
#include "servoControl.h"
#include "userButton.h"
#include "hallSensor.h"
#include "analog.h"
#include "controller.h"

int main(void)
{
//	initServoControl();
//	initUserButton();
//	initUltrasonic();
	initHallSensor();
	initMotorControl();
// 	initADC();
// 	initUART();
	initController();

	while (1)
	{
//		decisionMaker();
	}
}

void decisionMaker() {
	if(distance.frontMiddle < 50) {
		referenceSpeed = -50.0;
	}
	else if(distance.frontMiddle >= 100) {
		referenceSpeed = 50.0;
	}
}
