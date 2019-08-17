/*
 * SPI_test.c
 *
 *  Created on: Aug 14, 2019
 *      Author: kaushik
 */
#include <stdio.h>
#include "stm32f407xx.h"
#include "stm32f407xx_spi.h"
#include "stm32f407xx_gpio.h"

int main() {

	GPIO_Handle_t Spi1;
	Spi1.pGPIOx = GPIOA;
	Spi1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	Spi1.GPIO_PinConfig.GPIO_PinAltFunMode = AF5;
	Spi1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Spi1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	Spi1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;

	Spi1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN4;
	GPIO_Init(&Spi1);
	Spi1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN5;
	GPIO_Init(&Spi1);
	Spi1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN6;
	GPIO_Init(&Spi1);
	Spi1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN7;
	GPIO_Init(&Spi1);

	SPI_Handle_t spi;
	spi.pSPIx = SPI1;
	spi.SPIConfig.SPI_DeviceMode = MASTER;
	spi.SPIConfig.SPI_BusConfig = BIDI_TXRX;
	spi.SPIConfig.SPI_SclkSpeed = DIV2;
	spi.SPIConfig.SPI_DFF = BYTE;
	spi.SPIConfig.SPI_CPOL = IDLE0;
	spi.SPIConfig.SPI_CPHA = TRAILING_EDGE;
	spi.SPIConfig.SPI_SSM = ENABLE;
	spi.SPIConfig.SPI_SSI = ENABLE;
	SPI_Init(&spi);



	char buf[] = "Hello World";
	SPI_SendData(SPI1, (uint8_t*)buf,sizeof(buf));

	while (1)
		;

}
