#include "analog.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"



float ADCval;


void initADC(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;		//Enables peripheral clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;	//Enables clock for GPIOC
	ADC->CCR |= ADC_CCR_ADCPRE;		//0b11;  //ADC clock 12.5MHz (100MHz prescaler 8)
	GPIOC->MODER |= GPIO_MODER_MODER4;		//Sets GPIO PC4 to analog in
	ADC1->SMPR1 |= ADC_SMPR1_SMP14;			//Sets sample rate for channel 14 to (111 = 480 cycles)
	ADC1->CR1 |= ADC_CR1_EOCIE;			//Enables interrupts for EOC
	ADC1->CR2 |= ADC_CR2_ADON;  		//sets ADON bit (turns on ADC)
	ADC1->SQR3 |= 14;

	NVIC_EnableIRQ(ADC_IRQn);			//Enables interrupt handler
	NVIC_SetPriority(ADC_IRQn, 18);
}

void singleADC(void){
	ADC1->CR2 |= ADC_CR2_SWSTART;		//Starts conversion
	//ADC1->CR2 &= !ADC_CR2_CONT;			//Ensures single conversion mode, sets CONT bit to 0

}



void ADC_IRQHandler (void) {
	if(ADC1->SR & ADC_SR_EOC) {
		ADCval = ADC1->DR;		//Reads data from register to variable
		ADCval = ADCval/1241.0;

		ADC1->SR &= ~ADC_SR_EOC;
	}

}
