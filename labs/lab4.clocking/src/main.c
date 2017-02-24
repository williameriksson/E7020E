/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 7.0.1   2017-01-30

The MIT License (MIT)
Copyright (c) 2009-2016 Atollic AB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************
*/

/* Includes */
#include "stm32f4xx.h"

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */


/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{

	int i = 0;

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //Enable the GPIOC output
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enable the GPIOA output
	RCC->CFGR |= (0b11<<21);  //Sets MCO1 output to be PLL clock
	RCC->CFGR |= (0b111<<27); //Sets MCO2 to prescale division by 5
	RCC->CFGR |= (0b111<<24); //Sets MCO1 to prescale division by 5
	GPIOC->MODER |= (1<<19); //Changes mode to Alternative Function
	GPIOC->OSPEEDR |= (1<<19); //Changes the the output speed to high GPIOC 9
	GPIOA->MODER |= (1<<17); //Changes mode to AF GPIOA 8
	GPIOA->OSPEEDR |= (1<<17); //Changes output speed to High GPIOA 8


  /**
  *  IMPORTANT NOTE!
  *  The symbol VECT_TAB_SRAM needs to be defined when building the project
  *  if code has been located to RAM and interrupts are used. 
  *  Otherwise the interrupt table located in flash will be used.
  *  See also the <system_*.c> file and how the SystemInit() function updates 
  *  SCB->VTOR register.  
  *  E.g.  SCB->VTOR = 0x20000000;  
  */

  /* TODO - Add your application code here */

  /* Infinite loop */
  while (1)
  {
	i++;
  }
}