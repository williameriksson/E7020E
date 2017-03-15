
#include "main.h"
#include "ultrasonic.h"
#include "motorControl.h"
#include "servoControl.h"
#include "userButton.h"
#include "hallSensor.h"
#include "analog.h"

int main(void)
{
//	initServoControl();
//	initUserButton();
//	initMotorControl();
//	initUltrasonic();
	initHallSensor();
	initMotorControl();
// 	initADC();
// 	initUART();
	initController();

	while (1)
	{

	}
}

