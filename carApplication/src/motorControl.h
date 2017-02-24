/*
 * motorControl.h
 *
 *  Created on: 24 Feb 2017
 *      Author: sebas
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

void initMotorControl();

void resetSpeed();
void accelerate();
void decelerate();

#endif /* MOTORCONTROL_H_ */
