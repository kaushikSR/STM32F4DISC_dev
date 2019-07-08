/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include<stdint.h>
#include "stm32f4xx.h"
			
/*Before accessing any peripherals, we must enable the clock for it using the RCC registers.
 * By default, clock is disabled for all peripherals.
 * */
int main(void)
{	RCC_TypeDef* pRCC = RCC;
	ADC_TypeDef* pADC = ADC1;// Trying to access peripheral ADC1
	pRCC->APB2ENR |=(1<<8);//enable clock for ADC1 before changing the value.
	pADC->CR1 = 0x55;

	GPIO_TypeDef* pGPIO = GPIOA;
	pRCC->AHB1ENR |=(1<<0);
	pGPIO->OSPEEDR = 0x12;
	return 0;
}
