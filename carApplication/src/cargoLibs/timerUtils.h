#ifndef TIMERUTILS_H_
#define TIMERUTILS_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

int getTimeDiff16b(int, int);
uint32_t getTimeDiff32b(uint32_t, uint32_t);

#endif /* TIMERUTILS_H_ */
