/*
 * stm32f407xx_rcc.c
 *
 *  Created on: Aug 20, 2019
 *      Author: kaushik
 */


#include "stm32f407xx_rcc.h"

uint16_t AHB_PreScaler[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint8_t APB1_PreScaler[4] = { 2, 4, 8, 16 };

uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}

uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	// Determining the clock source
	if (clksrc == HSI) {
		SystemClk = 16000000; //16MHz
	} else if (clksrc == HSE) {
		SystemClk = 8000000; //8MHz
	} else if (clksrc == PLL) {
		SystemClk = RCC_GetPLLOutputClock();
	}

	//AHB prescalar calculation
	temp = ((RCC->CFGR >> 4) & 0xf);
	if (temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp - 8];
	}

	//APB1 prescalar calculation
	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4) {
		apb1p = 1;
	} else {
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p; // Based on the MCU clock tree

	return pclk1;
}

