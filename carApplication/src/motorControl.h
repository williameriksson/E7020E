#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

void initMotorControl(void);

void resetSpeed(void);
void setSpeed(int);
void accelerate(float);
void decelerate(void);


#endif /* MOTORCONTROL_H_ */
