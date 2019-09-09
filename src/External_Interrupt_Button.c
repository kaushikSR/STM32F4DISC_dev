/*
 * main.c
 *
 *  Created on: Aug 8, 2019
 *      Author: kaushik
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"

void delay(void) {
	for (int i = 0; i < 500000; i++)
		;
}

void ButtonPressLED() {
	GPIO_Handle_t pGpio_LED;
	pGpio_LED.pGPIOx = GPIOD;
	pGpio_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	pGpio_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN12;
	pGpio_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pGpio_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	pGpio_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_MEDIUM;
	GPIO_Init(&pGpio_LED);

	GPIO_Handle_t pGpio_BUTTON;
	pGpio_BUTTON.pGPIOx = GPIOA;
	pGpio_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RFT;
	pGpio_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN0;
	pGpio_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	pGpio_BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	GPIO_Init(&pGpio_BUTTON);

	GPIO_IRQITConfig(EXTI0,ENABLE);
	GPIO_IRQPriorityConfig(EXTI0,15);

	while(1);
}

void EXTI0_IRQHandler(void){
	delay(); //To avoid switch de-bouncing
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN12);
	GPIO_IRQHandle(GPIO_PIN0);

}


int main(void) {

	ButtonPressLED();
	return 0;
}

