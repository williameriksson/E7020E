#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "cargoLibs/circularBuffer.h"

typedef struct {
	int frontLeft;
	int frontMiddle;
	int frontRight;
} DistanceStruct;

DistanceStruct distance;
CircularBUFFER distanceBufferLeft;
CircularBUFFER distanceBufferMiddle;
CircularBUFFER distanceBufferRight;

void initUltrasonic(void);

#endif /* ULTRASONIC_H_ */
