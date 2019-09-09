/*
 * SPI_ArduinoSlave.c
 *
 *  Created on: Aug 17, 2019
 *      Author: kaushik
 */
#include "string.h"
#include "stm32f407xx.h"
#include "stm32f407xx_spi.h"
#include "stm32f407xx_gpio.h"

void delay(void) {
	for (int i = 0; i < 500000; i++)
		;
}

void GPIO_Button_Init() {
	GPIO_Handle_t pGpio_BUTTON;
	pGpio_BUTTON.pGPIOx = GPIOA;
	pGpio_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	pGpio_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN0;
	pGpio_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	pGpio_BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
	GPIO_Init(&pGpio_BUTTON);
}

void SPI_GPIO_Init() {
	GPIO_Handle_t Spi2;
	Spi2.pGPIOx = GPIOB;
	Spi2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	Spi2.GPIO_PinConfig.GPIO_PinAltFunMode = AF5;
	Spi2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Spi2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	Spi2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;

	Spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN12; //NSS
	GPIO_Init(&Spi2);
	Spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN13; //SCK
	GPIO_Init(&Spi2);
	//Spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN14;//MISO
	//GPIO_Init(&Spi2);
	Spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN15;	//MOSI
	GPIO_Init(&Spi2);

}

void SPI2_Init() {
	SPI_Handle_t spi;
	spi.pSPIx = SPI2;
	spi.SPIConfig.SPI_DeviceMode = MASTER;
	spi.SPIConfig.SPI_BusConfig = BIDI_TXRX;
	spi.SPIConfig.SPI_SclkSpeed = DIV8;
	spi.SPIConfig.SPI_DFF = BYTE;
	spi.SPIConfig.SPI_CPOL = IDLE0;
	spi.SPIConfig.SPI_CPHA = LEADING_EDGE;
	spi.SPIConfig.SPI_SSM = DISABLE; // Hardware slave management
	//spi.SPIConfig.SPI_SSI = DISABLE;
	SPI_Init(&spi);

}

int main() {

	GPIO_Button_Init();
	SPI_GPIO_Init();
	SPI2_Init();

	char user_data[] =
			"An Arduino Uno board is best suited for beginners who have just started using microcontrollers, on the other hand, Arduino Mega board is for enthusiasts who require a lot of I/O pins for their projects";
	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e when SPE=1 , NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOE_Config(SPI2, ENABLE);

	while (1) {
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN0))
			; //Wait till we read something from the pin

		delay();
		SPI_Peri_Config(SPI2, ENABLE);

		//first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));

		while (SPI_GetFlagStatus(SPI2, BSY))
			; // Wait till SPI is not busy
		SPI_Peri_Config(SPI2, DISABLE);

	}

}

