#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

float testInput; //for internal testing of the controller.
float referenceSpeed;
float Kp;
float Ki;
float Kd;

void initController(void);
void controlLoop(float, float);
void resetPID(void);
void startController(void);
void stopController(void);

#endif /* CONTROLLER_H_ */
