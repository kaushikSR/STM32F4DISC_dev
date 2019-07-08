/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdint.h>
#include "stm32f4xx.h"
			
/*Switching clock source to HSE and observing output at Microcontroller clock output 2 (MCO2)*/
int main(void)
{

	RCC_TypeDef* pRCC = RCC;


	//Turn on HSE Oscillator by setting the 16th bit
	pRCC->CR |=(1<<16);

	int n = 0;

	// Wait until HSE becomes stable by verifying 17th bit is 1
	while(!n){n = (pRCC->CR >> 17) & 1;}

	// Setting MCO2 to SYSCLK
	pRCC->CFGR &=~(1<<30);
	pRCC->CFGR &=~(1<<31); // 00 for SYSCLK

	//Setting MCO2 prescalar to no division. So we must get 8MHz as output at PC9
	pRCC->CFGR &=~(1<<27);
	pRCC->CFGR &=~(1<<28);
	pRCC->CFGR &=~(1<<29);

	// Select HSE as system clock
	pRCC->CFGR &= ~(0x3);//resetting the 0th and 1st bit
	pRCC->CFGR |=1; // setting 0th bit to change HSE as system clock.

	//Need to enable GPIOC PIN 9 which is connected to MCO2
	pRCC->AHB1ENR |=(1<<2);//enabling clock for GPIOC; setting bit 2

	// Must set pin 9 of GPIOC to alternate mode as per Reference manual. Moder9 is bits 18 and 19
	GPIO_TypeDef* pGPIO = GPIOC;

	pGPIO->MODER &=~(1<<18);
	pGPIO->MODER &=~(1<<19);
	pGPIO->MODER |= (1<<18); //18:19 must be 10 for alternate mode

	return 0;
}
