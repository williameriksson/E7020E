#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

#define BUFFERSIZE 3

typedef struct{
	uint32_t current;
	uint32_t bufferList[BUFFERSIZE];
}CircularBUFFER;

void circularBufferInit(CircularBUFFER *buffer);
void bufferAdd(CircularBUFFER *buffer, int input);
int getAverage(CircularBUFFER *buffer);

