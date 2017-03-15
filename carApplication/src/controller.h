#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

float testInput; //for internal testing of the controller.
float referenceSpeed;

void initController();
void controlLoop(float, float);

#endif /* CONTROLLER_H_ */
