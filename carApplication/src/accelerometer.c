#include "accelerometer.h"

void initAccelerometer() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable Clock port A
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable SPI peripheral clock

	GPIOA->MODER |= GPIO_MODER_MODER4_1; // Put PA4 to alternate function, check so value correct
	GPIOA->MODER |= GPIO_MODER_MODER5_1; // Put PA5 to alternate function, check so value correct
	GPIOA->MODER |= GPIO_MODER_MODER6_1; // Put PA6 to alternate function, check so value correct
	GPIOA->MODER |= GPIO_MODER_MODER7_1; // Put PA7 to alternate function, check so value correct

	// Put PA4-7 at medium speed, should give at least 12.5 MHz
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_0;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_0;

	GPIOA->AFR[0] |= (GPIO_AF5_SPI1 << 16); // PA4 to AF SPI1_NSS
	GPIOA->AFR[0] |= (GPIO_AF5_SPI1 << 20); // PA5 to AF SPI1_SCK
	GPIOA->AFR[0] |= (GPIO_AF5_SPI1 << 24); // PA6 to AF SPI1_MISO
	GPIOA->AFR[0] |= (GPIO_AF5_SPI1 << 28); // PA7 to AF SPI1_MOSI

	SPI1->CR1 |= SPI_CR1_MSTR; // Put the STM32 in master config
	SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0; // 0b111 to BR[2:0] (Baud rate ctrl f_PCLK/256)
	SPI1->CR2 |= SPI_CR2_TXEIE; // Tx buffer empty interrupt enable
	SPI1->CR2 |= SPI_CR2_RXNEIE; // Rx buffer NOT empty interrupt enable

	// SPI1->CR2 |= SPI_CR2_FRF; This would set TI mode instead of default motorola mode, it has to do
								// with the clock polarity, check which one it should be later...
	SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI1

	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_SetPriority(SPI1_IRQn, 15);

}

// CTRL_REG1(20h) write value: 0x57 HR / normal / Low power mode (100 Hz)


void SPI1_IRQHandler(void)
{
    unsigned char rx;
    static unsigned short int count = 0, i = 0 ;

    if (SPI1->SR & SPI_SR_RXNE) {   // Check received flag

    }

    if (SPI1->SR & SPI_SR_TXE) {    // Check transmission flag

    }



}
