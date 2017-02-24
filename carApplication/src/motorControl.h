/*
 * motorControl.h
 *
 *  Created on: 24 Feb 2017
 *      Author: sebas
 */


#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

void initMotorControl(void);

void resetSpeed(void);
void accelerate(void);
void decelerate(void);
