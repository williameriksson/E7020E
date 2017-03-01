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
//
//	USART2->BRR |= 100000000L/BAUDRATE;
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Start the clock for USART2
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Start the clock for the GPIOA
	//GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; // Put AF?

//	GPIOA->AFR[0] |= GPIO_AF7_USART2;
//	GPIOA->AFR[0] |= (GPIO_AF7_USART2 << 4);
//	GPIOA->AFR[0] |= (GPIO_AF7_USART2 << 8);
//	GPIOA->AFR[0] |= (GPIO_AF7_USART2 << 12);
//	USART2->CR1 |= USART_CR1_RXNEIE; //Enable UART RXNE Interrupt
//	USART2->CR1 &= ~USART_CR1_TXEIE; //Disable UART TXE Interrupt
//	USART2->CR1 |= USART_CR1_UE; // Enable USART, word length defaults to 8 bits and 1 stop bit
//	USART2->CR1 |= USART_CR1_TE;
//	USART2->CR1 |= USART_CR1_RE;
//	USART2->BRR |= 100000000L/BAUDRATE;
	USART6->CR1 |= USART_CR1_RXNEIE; //Enable UART RXNE Interrupt
	USART6->CR1 &= ~USART_CR1_TXEIE; //Disable UART TXE Interrupt
	USART6->CR1 |= USART_CR1_UE; // Enable USART, word length defaults to 8 bits and 1 stop bit
	USART6->CR1 |= USART_CR1_TE;
	USART6->CR1 |= USART_CR1_RE;
	USART6->BRR |= 50000000L/BAUDRATE;
	//NVIC_EnableIRQ(USART2_IRQn); // USART2 Interrupt enable
	//NVIC_SetPriority(USART2_IRQn, 38);

	NVIC_EnableIRQ(USART6_IRQn);
	NVIC_SetPriority(USART6_IRQn, 71);

	initInterrupt();

}

void initInterrupt() {

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Start the clock for the GPIOA
	GPIOA->MODER |= (1<<10); //GPIOA 5 to General purpose output.
	__disable_irq(); //Disable global interrupts while setting TIM2 up
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //Enable clock for TIM2
	TIM2->DIER |= TIM_DIER_UIE; //Enable update interrupt event
	TIM2->PSC = 50-1; //Set prescaler to get timer at 1MHz
	TIM2->ARR = 1000000-1; //Set Auto-reload register
	TIM2->CR1 |= TIM_CR1_CEN; //Enable TIM2
	__enable_irq(); //Enable global interrupts

	NVIC_EnableIRQ(TIM2_IRQn); // TIM2 Interrupt enable
	NVIC_SetPriority(TIM2_IRQn, 40);

	//NVIC_EnableIRQ(USART2_IRQn); // USART2 Interrupt enable
	//NVIC_SetPriority(USART2_IRQn, 38);

}
//Interrupt handler for toggling LED
void TIM2_IRQHandler (void) {
	if(TIM2->SR & TIM_SR_UIF) {
		TIM2->SR &= ~TIM_SR_UIF;
		GPIOA->ODR ^= (1<<5);
	}
}

//USART2 Interrupt handler
//void USART2_IRQHandler (void) {
//	if (USART2->SR & USART_SR_RXNE) {
//		pollUART();
//	}
//
//}

//New USART2 Interrupt handler with buffer
void USART6_IRQHandler (void) {
	uint8_t ch;
	if (USART6->SR & USART_SR_RXNE){
		ch=(uint8_t)USART6->DR;
		BufferPut(&U2Rx, ch);
	}

}
void blinkLED() {
	uint8_t temp = Usart2Get();
	char freq = temp;
	int blinkFreq = freq;
	__disable_irq();
	TIM2->ARR = blinkFreq * 1000-1;
	TIM2->CNT = 0;
	__enable_irq();

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

//void Usart2Put(uint8_t ch) {
//	BufferPut(&U1Tx, ch);
//}




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



