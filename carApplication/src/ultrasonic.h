#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "cargoLibs/circularBuffer.h"

float DISTANCE;
CircularBUFFER distanceBuffer;

void initUltrasonic(void);

#endif /* ULTRASONIC_H_ */
