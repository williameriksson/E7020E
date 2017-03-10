#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

#include "circularBuffer.h"

float filterNoise(float previous, CircularBUFFER *buff, int tolerance);

void varianceFilter32b(uint32_t *start, uint32_t *end, float tolerance);
