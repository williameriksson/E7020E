
#include "main.h"
#include "ultrasonic.h"
#include "motorControl.h"
#include "servoControl.h"
#include "userButton.h"
#include "hallSensor.h"
#include "analog.h"
#include "controller.h"
#include "bt_uart.h"
#include <math.h>

int main(void)
{
	initServoControl();
	initUltrasonic();
	initHallSensor();
	initMotorControl();
 	initUART();
	initController();
	initUserButton();

	// 	initADC();
	while (1) {
		decisionMaker();
	}
}

void decisionMaker() {
	if(distance.frontMiddle <= 100) {

		if (distance.back <= 50) {
			referenceSpeed = 10.0;
			turnLeft();
		} else {
			referenceSpeed = -10.0;
			turnRight();
		}
		//if (distance.frontRight > distance.frontLeft) {

	//	} else {
		//	turnLeft();
		//}
	} else {
		referenceSpeed = 10.0;
		if (enableTurning) {
			if (sqrt(exp2(distance.frontLeft - distance.frontRight)) >= 70) {
				if (distance.frontRight > distance.frontLeft) {
					turnHalfRight();
				} else {
					turnHalfLeft();
				}
			} else {
				turnReset();
			}
		}
	}
}
