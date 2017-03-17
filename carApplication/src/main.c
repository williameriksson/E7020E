
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

	// 	initADC();
	initUserButton();
	while (1) {
		decisionMaker();
	}
}

void decisionMaker() {
	if(distance.frontMiddle <= 150) {
		referenceSpeed = -10.0;
		if (distance.frontRight > distance.frontLeft) {
			turnRight();
		} else {
			turnLeft();
		}
	} else {
		referenceSpeed = 10.0;
		if (enableTurning) {
			if (sqrt(exp2(distance.frontLeft - distance.frontRight)) >= 50) {
				if (distance.frontRight > distance.frontLeft) {
					turnRight();
				} else {
					turnLeft();
				}
			} else {
				turnReset();
			}
		}
	}
}
