#ifndef HALLSENSOR_H_
#define HALLSENSOR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "cargoLibs/circularBuffer.h"
#include "cargoLibs/timerUtils.h"

void initHallSensor(void);

float speed;

#endif /* HALLSENSOR_H_ */
