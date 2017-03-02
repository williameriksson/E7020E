
#include "main.h"
#include "ultrasonic.h"
#include "motorControl.h"
#include "servoControl.h"
#include "userButton.h"
#include "hallSensor.h"

int main(void)
{
	initServoControl();
	initUserButton();
	initMotorControl();
	initUltrasonic();
	initHallSensor();

	while (1)
	{

	}
}

