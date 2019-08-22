/*
 * stm32f407xx_rcc.h
 *
 *  Created on: Aug 20, 2019
 *      Author: kaushik
 */

#ifndef INC_STM32F407XX_RCC_H_
#define INC_STM32F407XX_RCC_H_
#include "stm32f407xx.h"



/*Register definitions*/

typedef enum {
	RCC_SW0 = BIT0,
	RCC_SW1 = BIT1,
	RCC_SWS0 = BIT2,
	RCC_SWS1 = BIT3,
	RCC_HPRE0 = BIT4,
	RCC_HPRE1 = BIT5,
	RCC_HPRE2 = BIT6,
	RCC_HPRE3 = BIT7,
	RCC_PPRE1_0 = BIT10,
	RCC_PPRE1_1 = BIT11,
	RCC_PPRE1_2 = BIT12,
	RCC_PPRE2 = BIT13,
	RCC_RTCPRE = BIT16,
	RCC_MCO1 = BIT21,
	RCC_I2SSCR = BIT23,
	RCC_MCO1PRE = BIT24,
	RCC_MCO2PRE = BIT27,
	RCC_MCO2 = BIT30
} RCC_CFGR;


uint32_t  RCC_GetPLLOutputClock();
uint32_t RCC_GetPCLK1Value(void);

#endif /* INC_STM32F407XX_RCC_H_ */


