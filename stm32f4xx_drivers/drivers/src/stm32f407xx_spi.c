/*
 * stm32f407xx_spi.c
 *
 *  Created on: Aug 13, 2019
 *      Author: kaushik
 */

#include "stm32f407xx_spi.h"

/*@brief: This function helps to configure the SPI peripheral Clock
 *@param[in] : A pointer to the SPIx base address
 *@param[in] : ENABLE or DISABLE
 *@return : None
 * */

void SPI_PCLK_Config(SPI_RegDef_t *pSPIx, status ENorDI) {
	if (ENorDI == ENABLE) {
		pSPIx == SPI1 ? SPI1_PCLK_EN() :\
 pSPIx == SPI2 ? SPI2_PCLK_EN() :\

		pSPIx == SPI3 ? SPI2_PCLK_EN() : -1;
	} else {
		pSPIx == SPI1 ? SPI1_PCLK_DI() :\
 pSPIx == SPI2 ? SPI2_PCLK_DI() :\

		pSPIx == SPI3 ? SPI2_PCLK_DI() : -1;
	}
}

/*@brief: This function helps to initialize the SPI peripheral
 *@param[in] : A pointer to the SPI Handle data structure
 *@return : None
 * */

void SPI_Init(SPI_Handle_t* pSPIHandle) {

	SPI_PCLK_Config(pSPIHandle->pSPIx, ENABLE);

	/*Mode*/
	if (pSPIHandle->SPIConfig.SPI_DeviceMode) {
		pSPIHandle->pSPIx->CR1 |= (1 << MSTR);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(1 << MSTR);
	}
	/*BusConfig*/
	if (pSPIHandle->SPIConfig.SPI_BusConfig == BIDI_TXRX) {
		pSPIHandle->pSPIx->CR1 &= ~(BIT15); // 2 line unidirectional
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == UNIDI_RX) {
		pSPIHandle->pSPIx->CR1 |= (BIT15); // 1 line Bidirectional
		pSPIHandle->pSPIx->CR1 &= ~(BIT14); // Receive-Only mode
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == UNIDI_TX) {
		pSPIHandle->pSPIx->CR1 |= (BIT15); // 1 line Bidirectional
		pSPIHandle->pSPIx->CR1 |= (BIT14); //Transmit-Only mode
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == BIDI_RX) {
		pSPIHandle->pSPIx->CR1 &= ~(BIT15); // 2 line unidirectional
		pSPIHandle->pSPIx->CR1 |= (BIT10); // Receive-Only
	}
	/*ClockSpeed*/
	pSPIHandle->pSPIx->CR1 &= ~(7U << BR); //clearing 3 bits [5:3]
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << BR);
	/*Data Frame*/
	if (pSPIHandle->SPIConfig.SPI_DFF) {
		pSPIHandle->pSPIx->CR1 |= (1 << DFF);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(1 << DFF);
	}
	/*Clock Polarity*/
	if (pSPIHandle->SPIConfig.SPI_CPOL) {
		pSPIHandle->pSPIx->CR1 |= (1 << CPOL);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(1 << CPOL);
	}
	/*Clock Phase*/
	if (pSPIHandle->SPIConfig.SPI_CPHA) {
		pSPIHandle->pSPIx->CR1 |= (1 << CPHA);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(1 << CPHA);
	}
	/*Software slave management*/
	if (pSPIHandle->SPIConfig.SPI_SSM) {
		pSPIHandle->pSPIx->CR1 |= (1 << SSM);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(1 << SSM);
	}
	/*slave select information*/
	if (pSPIHandle->SPIConfig.SPI_SSI) {
		pSPIHandle->pSPIx->CR1 |= (1 << SSI);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(1 << SSI);
	}
}

/*@brief: This function helps to deinitialize the SPI peripheral
 *@param[in] : A pointer to the SPI Handle data structure
 *@return : None
 * */

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_DEINIT()
		;
	} else if (pSPIx == SPI2) {
		SPI2_DEINIT()
		;
	} else if (pSPIx == SPI3) {
		SPI3_DEINIT()
		;
	}
}

void SPI_Enable(SPI_RegDef_t *pSPIx) {
	pSPIx->CR1 |= (BIT6);
}

void SPI_Disable(SPI_RegDef_t *pSPIx) {
	pSPIx->CR1 &= ~(BIT6);
}


/*@brief: These functions can be used to send/receive data (Polling/Blocking method)
 *@param[in] : A pointer to the SPI base address
 *@param[in] : A pointer to the Tx/Rx Buffer
 *@param[in] : Length of data/buffer
 *@return : None
 * */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	SPI_Enable(pSPIx);
	while (Len > 0) {
		while (!(pSPIx->SR & BIT1))
			; // wait till TXE flag is set/Tx Buffer is empty
		if (pSPIx->CR1 & BIT11) { // 16-BIT
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len -= 2;
			pTxBuffer += 2;
		} else { // 8-BIT
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	SPI_Enable(pSPIx);
	while (Len > 0) {
		while (!(pSPIx->SR & BIT0))
			; // wait till RXE flag is set/Rx Buffer is empty
		if (pSPIx->CR1 & BIT11) { // 16-BIT
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			Len -= 2;
			pRxBuffer += 2;
		} else { // 8-BIT
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/*
 * @brief : This function ENABLES/DISABLES the given IRQ on the NVIC
 *
 * @param[in]: IRQ/Position number
 * @param[in]: ENABLE/DISABLE
 * @return : None
 */
void SPI_IRQITConfig(uint8_t IRQNumber, status EnOrDi) {

}

/*
 * @brief : This function sets the priority of the given IRQ in the NVIC
 *
 * @param[in]: IRQ/Position number
 * @param[in]: Priority
 * @return : None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

}

/*
 * @brief : This function clears the pending register of the EXTI controller for a particular EXTI line
 *
 * @param[in]: EXTI line number/Pin Number
 * @return : None
 *
 * @Note : This must be called in all ISR functions for EXTI lines indicating the interrupt/event has been serviced
 */
void SPI_IRQHandle(uint8_t EXTI_line) {

}
