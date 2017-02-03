
#include "stm32f4xx.h"
#include "uart.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

#include <stdlib.h>
#include <string.h>

#define BAUDRATE 19200
#define NUMELEMS(x)  (sizeof(x) / sizeof((x)[0]))
/*
 * PA0 USART2_CTS with AF7
 * PA1 USART2_RTS with AF7
 * PA2 USART2_TX with AF7
 * PA3 USART2_RX with AF7
 * PA4 USART2_CK with AF7
 * */

int blinkRate = 500;
int blinkEnable = 1;
int statusOn = 1;

void initUart() {
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Start the clock for USART2
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Start the clock for the GPIOA
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


void startUpSequence() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Start the clock for the GPIOA
	GPIOA->MODER |= (1<<10); //GPIOA 5 to General purpose output.
	__disable_irq(); //Disable global interrupts while setting TIM2 up
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //Enable clock for TIM2
	TIM2->DIER |= TIM_DIER_UIE; //Enable update interrupt event
	TIM2->PSC = 50-1; //Set prescaler to get timer at 1MHz
	TIM2->ARR = 1000000-1; //Set Auto-reload register

	TIM2->CR1 |= TIM_CR1_CEN; //Enable TIM2
	__enable_irq(); //Enable global interrupts

	NVIC_EnableIRQ(TIM2_IRQn); //Enable TIM2 interrupt handler
	NVIC_SetPriority(TIM2_IRQn, 35); //Set interrupt priority

	initUart();

	uint8_t initString1 [] = "System Operational";
	uint8_t initString2 [] = "status on";
	USART_PutString(initString1);
	USART_PutString(initString2);

	pollUart();


}

void TIM2_IRQHandler (void){
	static int count = 0;

	if (TIM2->SR & TIM_SR_UIF) { //Check interrupt flag

		TIM2->SR &= ~TIM_SR_UIF; //Reset interrupt flag
		if (blinkEnable)
		GPIOA->ODR ^= (1<<5);	//Toggle LED
	}
	count++;
	if (count == 6) blinkEnable = 0;
}



void pollUart() {
	uint8_t on [] = "status on";
	uint8_t off [] = "status off";
	uint8_t blink [] = "status blink ";
	uint8_t error [] = "ERROR, could not parse ";
	uint8_t tooLong [] = "ERROR, string too long";


	while (1) {
		while (!(USART2->SR & USART_SR_RXNE)) {

		}

		int MAXWORDLEN = 16;
		char received [MAXWORDLEN];

		int i = 0;
		int isTooLong = 0;
		while (1) {
			if ((USART2->SR & USART_SR_RXNE)) {
				char temp = USART2->DR;

				if (temp == '\n') {
					break;
				}
				if (temp == '\r') {
					continue;
				}
				if (i == MAXWORDLEN) {
					isTooLong = 1;
				}
				if (!isTooLong) {
					received[i] = temp;
					i++;
				}
			}
		}

		char string [i];
		memcpy(string, received, i);
		string[i] = '\0';

		char blinkValue [i - 6];
		//char newString [5];
		char newString [5] = {0, 0, 0, 0, 0};

		if (i > 6 && !isTooLong) {
			for (int j = 0; j <= i - 6; j++) {
				blinkValue[j] = string[j + 6];
			}
			//blinkValue[i-6] = '\0';

			sscanf(blinkValue, "%d", &blinkRate);

			for (int k = 0; k <= 5; k++) {
				newString[k] = string[k];
			}
			newString[5] = '\0';

			// Check so that the bytes after blink represents integers
			// else, put some garbish at the first position so it does not match it.
			for (int k = 0; k < NUMELEMS(blinkValue) - 1; k++) {
				if (!(blinkValue[k] >= 48 && blinkValue[k] <= 57)) {
					newString[0] = 0;
					break;
				}
			}



		}

		// strcmp return 0 if the char arrays are equal else 1.
		if (strcmp(string, "on") == 0) {
			statusOn = 1;
			blinkEnable = 1;
			USART_PutString(on);
		} else if (strcmp(string, "off") == 0){
			statusOn = 0;
			blinkEnable = 0;
			USART_PutString(off);
		} else if (strcmp(newString, "blink") == 0) {
			if (statusOn) {
				blinkEnable = 1;
			}

			__disable_irq();
			TIM2->ARR = blinkRate * 1000 -1; //Set Auto-reload register
			TIM2->CNT = 0x0;
			__enable_irq();
			int sizePreString = NUMELEMS(blink);
			int sizePostString = NUMELEMS(blinkValue) + 1;

			char combined [sizePreString + sizePostString];

			for (int z = 0; z < sizePreString; z++) {
				combined[z] = blink[z];
			}

			for (int y = 0; y < sizePostString; y++) {
				combined[y + sizePreString - 1] = blinkValue[y];
			}

			USART_PutString(combined);
		} else if (isTooLong) {
			isTooLong = 0;
			USART_PutString(tooLong);
		} else {

			int sizePreString = NUMELEMS(error);
			int sizePostString = NUMELEMS(string);

			char combined [sizePreString + sizePostString];

			for (int z = 0; z < sizePreString; z++) {
				combined[z] = error[z];
			}

			for (int y = 0; y < sizePostString; y++) {
				combined[y + sizePreString - 1] = string[y];
			}
			USART_PutString(combined);
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
	USART_PutChar('\r');

}

