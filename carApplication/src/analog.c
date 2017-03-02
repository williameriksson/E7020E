#include "analog.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx"


//int main(void){
//	analog_INIT();

//}

void adc_init(void)
{
	RCC
	ADC_CCR |= 0b11; 		//ADC clock 12.5MHz (100MHz prescaler 8)
	RCC_APB2ENR |= ADC1EN
	ADC_CR2 |= 0x00000001;  //sets ADON bit (turns on ADC)
}
