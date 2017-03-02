#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

typedef struct {
	uint8_t indexPointer;
	uint8_t size;
	int *buffer;
} CircularBUFFER;

/*
#define BUFFERSIZE 3

typedef struct{
	uint32_t current;
	uint32_t bufferList[BUFFERSIZE];
}CircularBUFFER;
*/
void circularBufferInit(CircularBUFFER* , int, int);
void pushBuffer(CircularBUFFER* , int);
int getBufferAverage(CircularBUFFER* );

