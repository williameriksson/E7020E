#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

void initUserButton(void);
void initServoControl(void);

void turnReset(void);
void turnLeft(void);
void turnRight(void);

#endif /* SERVOCONTROL_H_ */
