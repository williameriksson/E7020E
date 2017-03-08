/*
 * bt_uart.c
 *
 *  Created on: 20 feb. 2017
 *      Author: Robin
 */

#include "bt_uart.h"
#include "stdlib.h"

#define BAUDRATE 9600
#define USARTBUFFSIZE 16
volatile FIFO_TypeDef U2Rx, U2Tx;

//int test = 0;
//Buffer struct
//typedef struct{
//	uint8_t in;
//	uint8_t out;
//	uint8_t count;
//	uint8_t buff[USARTBUFFSIZE];
//}FIFO_TypeDef;




void initUART () {
	BufferInit(&U2Rx);
	BufferInit(&U2Tx);
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	GPIOC->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER6_1;
	GPIOC->AFR[0] |= (GPIO_AF8_USART6 << 24);
	GPIOC->AFR[0] |= (GPIO_AF8_USART6 <<28);
	USART6->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	USART6->CR1 |= USART_CR1_RXNEIE; //Enable UART RXNE Interrupt
	USART6->CR1 &= ~USART_CR1_TXEIE; //Disable UART TXE Interrupt
	USART6->CR1 |= USART_CR1_UE; // Enable USART, word length defaults to 8 bits and 1 stop bit
	USART6->CR1 |= USART_CR1_TE;
	USART6->CR1 |= USART_CR1_RE;
	USART6->BRR |= 50000000L/BAUDRATE;

	NVIC_EnableIRQ(USART6_IRQn);
	NVIC_SetPriority(USART6_IRQn, 71);
}

//New USART6 Interrupt handler with buffer
void USART6_IRQHandler (void) {
	uint8_t ch;
	if (USART6->SR & USART_SR_RXNE){
		ch=(uint8_t)USART6->DR;
		BufferPut(&U2Rx, ch);
	}
	if (USART6->SR & USART_SR_TXE) {
		if (BufferGet(&U2Tx , &ch) == SUCCESS) {
			USART6->DR |= ch;

		}
		else {
			USART6->CR1 &= ~USART_CR1_TXEIE;
		}
	}

}


//Init buffer
void BufferInit(__IO FIFO_TypeDef *buffer){
	buffer->count = 0;
	buffer->in = 0;
	buffer->out = 0;
}

//But byte in buffer
ErrorStatus BufferPut(__IO FIFO_TypeDef *buffer, uint8_t ch) {
	if(buffer->count==USARTBUFFSIZE)
		return ERROR;//buffer full
	buffer->buff[buffer->in++]=ch;
	buffer->count++;
	if(buffer->in==USARTBUFFSIZE)
		buffer->in=0;//start from beginning
	return SUCCESS;
}

//Get byte from buffer
ErrorStatus BufferGet(__IO FIFO_TypeDef *buffer, uint8_t *ch) {
	if(buffer->count==0)
		return ERROR;//buffer empty
	*ch=buffer->buff[buffer->out++];
	buffer->count--;
	if(buffer->out==USARTBUFFSIZE)
		buffer->out=0;//start from beginning
	return SUCCESS;
}
//Check buffer status
ErrorStatus BufferIsEmpty(__IO FIFO_TypeDef buffer) {
    if(buffer.count==0)
        return SUCCESS;//buffer full
    return ERROR;
}

void Usart2Put(uint8_t ch) {
	BufferPut(&U2Tx, ch);
	USART6->CR1 |= USART_CR1_TXEIE;
}




uint8_t Usart2Get(void) {
	uint8_t ch;
	while (BufferIsEmpty(U2Rx) == SUCCESS);
	BufferGet (&U2Rx, &ch);
	return ch;
}

//int pollUART() {
//
//	int MAXWORDLEN = 16;
//	char received [MAXWORDLEN];
//	int i = 0;
//	int isTooLong = 0;
//	int freq = 0;
//
//	while (USART2->SR & USART_SR_RXNE) {
//		char temp = USART2->DR;
//
//		if (temp == '\n') {
//			break;
//		}
//		if (temp == '\r') {
//			continue;
//		}
//		if (i == MAXWORDLEN) {
//			isTooLong = 1;
//		}
//		if (!isTooLong) {
//			received[i] = temp;
//			i++;
//		}
//	}
//	char * ptr;
//	freq = strtol(received, &ptr, 10);
//	blinkLED(freq);
//
//
//}



