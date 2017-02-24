/*
 * servoControl.h
 *
 *  Created on: 24 Feb 2017
 *      Author: sebas
 */

#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

void initUserButton();
void initServoControl();

void turnReset();
void turnLeft();
void turnRight();



#endif /* SERVOCONTROL_H_ */
