/*
 * bt_uart.h
 *
 *  Created on: 20 feb. 2017
 *      Author: Robin
 */

#ifndef BT_UART_H_
#define BT_UART_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "stdbool.h"
#define USARTBUFFSIZE 16
//bool run = true;

typedef struct{
	uint8_t in;
	uint8_t out;
	uint8_t count;
	uint8_t buff[USARTBUFFSIZE];
}FIFO_TypeDef;

//typedef struct{
//}FIFO_TypeDef;

void tunePID(uint8_t);
void controlCar(uint8_t);


void initUART();
//void takeAction();
void echoUART();
void blinkLED();
//ErrorStatus BufferGet(FIFO_TypeDef, uint8_t);
//void BufferInit(FIFO_TypeDef);
//ErrorStatus BufferIsEmpty(FIFO_TypeDef);
//ErrorStatus BufferPut(FIFO_TypeDef, uint8_t);
void initInterrupt();
//volatile FIFO_TypeDef;
//uint8_t Usart2Get();
//int pollUART();
void Usart2Put(uint8_t);

uint8_t Usart2Get(void);
void BufferInit(__IO FIFO_TypeDef *buffer);
ErrorStatus BufferPut(__IO FIFO_TypeDef *buffer, uint8_t ch);
ErrorStatus BufferGet(__IO FIFO_TypeDef *buffer, uint8_t *ch);
ErrorStatus BufferIsEmpty(__IO FIFO_TypeDef buffer);

#endif /* BT_UART_H_ */
