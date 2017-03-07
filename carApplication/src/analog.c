#include "analog.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"



uint16_t ADCval;

void initADC(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;		//Enables peripheral clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;	//Enables clock for GPIOC
	ADC->CCR |= ADC_CCR_ADCPRE;		//0b11;  //ADC clock 12.5MHz (100MHz prescaler 8)
	GPIOC->MODER |= GPIO_MODER_MODER4;		//Sets GPIO PC4 to analog in
	ADC->CCR |= ADC_SMPR1_SMP14;			//Sets sample rate for channel 14 to (111 = 480 cycles)
	ADC->CCR |= ADC_CR1_EOCIE;			//Enables interrupts for EOC

	ADC->CCR |= 0x00000001;  		//sets ADON bit (turns on ADC)
	ADC->CCR |= ADC_CR2_SWSTART;		//Enables continuous mode
}

void ADC_IRQHandler (void) {
	if(ADC_SR_EOC) {
		ADCval = ADC_DR_DATA;		//Reads data from register to variable
				//breakpoint prints out ADCval
	}
}
