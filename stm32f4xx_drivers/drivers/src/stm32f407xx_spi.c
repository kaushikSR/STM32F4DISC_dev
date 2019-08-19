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
		pSPIHandle->pSPIx->CR1 |= (MSTR);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(MSTR);
	}
	/*BusConfig*/
	if (pSPIHandle->SPIConfig.SPI_BusConfig == BIDI_TXRX) {
		pSPIHandle->pSPIx->CR1 &= ~(BIDI_MODE); // 2 line unidirectional
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == UNIDI_RX) {
		pSPIHandle->pSPIx->CR1 |= (BIDI_MODE); // 1 line Bidirectional
		pSPIHandle->pSPIx->CR1 &= ~(BIDI_OE); // Receive-Only mode
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == UNIDI_TX) {
		pSPIHandle->pSPIx->CR1 |= (BIDI_MODE); // 1 line Bidirectional
		pSPIHandle->pSPIx->CR1 |= (BIDI_OE); //Transmit-Only mode
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == BIDI_RX) {
		pSPIHandle->pSPIx->CR1 &= ~(BIDI_MODE); // 2 line unidirectional
		pSPIHandle->pSPIx->CR1 |= (RX_ONLY); // Receive-Only
	}
	/*ClockSpeed*/
	pSPIHandle->pSPIx->CR1 &= ~(7U << BR); //clearing 3 bits [5:3]
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << BR);
	/*Data Frame*/
	if (pSPIHandle->SPIConfig.SPI_DFF) {
		pSPIHandle->pSPIx->CR1 |= (DFF);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(DFF);
	}
	/*Clock Polarity*/
	if (pSPIHandle->SPIConfig.SPI_CPOL) {
		pSPIHandle->pSPIx->CR1 |= (CPOL);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(CPOL);
	}
	/*Clock Phase*/
	if (pSPIHandle->SPIConfig.SPI_CPHA) {
		pSPIHandle->pSPIx->CR1 |= (CPHA);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(CPHA);
	}
	/*Software slave management*/
	if (pSPIHandle->SPIConfig.SPI_SSM) {
		pSPIHandle->pSPIx->CR1 |= (SSM);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(SSM);
	}
	/*slave select information*/
	if (pSPIHandle->SPIConfig.SPI_SSI) {
		pSPIHandle->pSPIx->CR1 |= (SSI);
	} else {
		pSPIHandle->pSPIx->CR1 &= ~(SSI);
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

/*@brief: This function helps to enable/disable the SPI peripheral
 *@param[in] : A pointer to the SPI Handle data structure
 *@param[in] : ENABLE/DISABLE
 *@return : None
 * */
void SPI_Peri_Config(SPI_RegDef_t *pSPIx, status ENorDI) {
	if (ENorDI) {
		pSPIx->CR1 |= (SPE);
	} else {
		pSPIx->CR1 &= ~(SPE);
	}
}

/*@brief: This function helps to return the status of the flag in the SPI peripheral SR register
 *@param[in] : A pointer to the SPI Handle data structure
 *@param[in] : Flag
 *@return : None
 * */

status SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, SPI_SR Flag) {
	return (pSPIx->SR & Flag);
}

/*@brief: These functions can be used to send/receive data (Polling/Blocking method)
 *@param[in] : A pointer to the SPI base address
 *@param[in] : A pointer to the Tx/Rx Buffer
 *@param[in] : Length of data/buffer
 *@return : None
 * */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while (Len > 0) {
		while (!(pSPIx->SR & TXE))
			; // wait till TXE flag is set/Tx Buffer is empty
		if (pSPIx->CR1 & DFF) { // 16-BIT
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len -= 2;
			pTxBuffer += 2;
		} else { // 8-BIT
			pSPIx->DR = (uint8_t) (*pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	while (Len > 0) {
		while (!(SPI_GetFlagStatus(pSPIx, RXNE)))
			; // wait till RXE flag is set/Rx Buffer is empty
		if (pSPIx->CR1 & DFF) { // 16-BIT
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

SPI_State SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,
		uint32_t Len) {
	SPI_State state = pSPIHandle->TxState;

	if (state != SPI_TX_BUSY) {
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_TX_BUSY;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (TXEIE);

	}
	return state;
}

SPI_State SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,
		uint32_t Len) {
	SPI_State state = pSPIHandle->RxState;

	if (state != SPI_RX_BUSY) {
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_RX_BUSY;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (RXNEIE);

	}

	return state;

}

/*@brief: This function helps to enable/disable the SSOE pin of CR2 register
 *@param[in] : A pointer to the SPI Handle data structure
 *@param[in] : ENABLE/DISABLE
 *@return : None
 *@Note : Enable it when using master mode to allow NSS to output 0
 * */

void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, status ENorDI) {
	if (ENorDI) {
		pSPIx->CR2 |= (SSOE);
	}
}

/*SPI_RegDef_t *pSPIx
 * @brief : This function ENABLES/DISABLES the given IRQ on the NVIC
 *
 * @param[in]: IRQ/Position number
 * @param[in]: ENABLE/DISABLE
 * @return : None
 */
void SPI_IRQITConfig(IRQ_Posn IRQNumber, status EnOrDi) {
	if (EnOrDi) {
		if (IRQNumber == 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 &= ~(1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 &= ~(1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 &= ~(1 << (IRQNumber % 64));
		}
	}
}

/*
 * @brief : This function sets the priority of the given IRQ in the NVIC
 *
 * @param[in]: IRQ/Position number
 * @param[in]: Priority
 * @return : None
 */
void SPI_IRQPriorityConfig(IRQ_Posn IRQNumber, uint8_t IRQPriority) {
	NVIC_IPR->IPR[IRQNumber] |= (uint8_t) (IRQPriority
			<< (8U - NVIC_PRIORITY_BITS) & (uint32_t) 0xFFUL);
}

// Helper functions

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~(TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	pSPIHandle->pSPIx->CR2 &= ~(RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,
		SPI_App_Events AppEv) {

	//This is a weak implementation . the user application may override this function.
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	// check the DFF bit in CR1
	if ((pSPIHandle->pSPIx->CR1 & (DFF))) {
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*) pSPIHandle->pTxBuffer++;
	} else {
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->TxLen) {
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_TX_COMPLETE);
	}

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	//do rxing as per the dff
	if (pSPIHandle->pSPIx->CR1 & (DFF)) {
		//16 bit
		*((uint16_t*) pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;

	} else {
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}

	if (!pSPIHandle->RxLen) {
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_RX_COMPLETE);
	}

}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	uint8_t temp;
	//1. clear the ovr flag only if SPI is not busy in transmission
	if (pSPIHandle->TxState != SPI_TX_BUSY) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void) temp;// unused warning
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_OVR_ERROR);

}


/*
 * @brief : This function can be used to clear the OVR flag of the SPI SR register.
 *
 * @param[in]: SPI register definition structure
 * @return : None
 *
 * @Note : This must be used to clear the OVR flag
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp; // unused warning

}


/*
 * @brief : This function handles the TX, RX and OVR interrupt events of the SPI peripheral
 *
 * @param[in]: SPI Handle data structure
 * @return : None
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle) {

	uint8_t temp1, temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & (TXE);
	temp2 = pHandle->pSPIx->CR2 & (TXEIE);

	if (temp1 && temp2) {
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & (RXNE);
	temp2 = pHandle->pSPIx->CR2 & (RXNEIE);

	if (temp1 && temp2) {
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & (OVR);
	temp2 = pHandle->pSPIx->CR2 & (ERRIE);

	if (temp1 && temp2) {
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}


