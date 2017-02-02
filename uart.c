
#include "stm32f4xx.h"
#include "uart.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

#include <stdlib.h>
#include <string.h>

#define BAUDRATE 19200

/*
 * PA0 USART2_CTS with AF7
 * PA1 USART2_RTS with AF7
 * PA2 USART2_TX with AF7
 * PA3 USART2_RX with AF7
 * PA4 USART2_CK with AF7
 * */



void initUart() {
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Start the clock for USART2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Start the clock for the GPIOA
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; // Put AF?

	GPIOA->AFR[0] |= GPIO_AF7_USART2;
	GPIOA->AFR[0] |= (GPIO_AF7_USART2 << 4);
	GPIOA->AFR[0] |= (GPIO_AF7_USART2 << 8);
	GPIOA->AFR[0] |= (GPIO_AF7_USART2 << 12);

	USART2->CR1 |= USART_CR1_UE; // Enable USART, word length defaults to 8 bits and 1 stop bit
	USART2->CR1 |= USART_CR1_TE;
	USART2->CR1 |= USART_CR1_RE;
	USART2->BRR |= 100000000L/BAUDRATE;
}



void pollUart() {
	uint8_t on [] = "status on";
	uint8_t off [] = "status off";
	uint8_t blink [] = "status blink";
	uint8_t error [] = "ERROR, could not parse...";
	char oncmp [] = "on";


	while (1) {
		while (!(USART2->SR & USART_SR_RXNE)) {

		}

		int MAXWORDLEN = 50;
		char received [MAXWORDLEN];
		int i = 0;
		while (1) {
			received[i] = USART2->DR;
			if (received[i] == '\n') break;
			if (i == MAXWORDLEN - 1) break;
			i++;
		}


		if (received[0] == oncmp[0] && received[1] == oncmp[1] ) {
			USART_PutString(on);
		} else if (*received == *off){
			USART_PutString(off);
		} else if (*received == *blink) {
			USART_PutString(blink);
		} else {
			USART_PutString(error);
		}
	}

}



void USART_PutChar(uint8_t ch) {
	while (!(USART2->SR & USART_SR_TXE)) {
		  ;
	}
	USART2->DR = ch;
}

void USART_PutString(uint8_t * str) {
	while(*str != 0) {
		USART_PutChar(*str);
		str++;
	}
}

