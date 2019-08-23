/*
 * stm32f407xx_i2c.c
 *
 *  Created on: Aug 19, 2019
 *      Author: kaushik
 */
#include "stm32f407xx.h"
#include "stm32f407xx_i2c.h"
#include "stm32f407xx_rcc.h"
/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             - Enables/Disables the I2C peripheral
 *
 * @param[in]         - A pointer to the I2C Peripheral
 * @param[in]         - ENABLE/DISABLE
 *
 * @return            - NONE
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, status EnOrDi) {
	if (EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (I2C_PE);
	} else {
		pI2Cx->CR1 &= ~(I2C_PE);
	}
}

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - Enables/Disables the clock for the I2Cx Peripheral
 *
 * @param[in]         - A pointer to the I2C peripheral
 * @param[in]         - ENABLE/DISABLE
 *
 * @return            - None
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, status EnorDi) {
	if (EnorDi == ENABLE) {
		I2C_PCLK_EN(pI2Cx);
	} else {
		I2C_PCLK_DI(pI2Cx);
	}
}

/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t tempreg = 0;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	pI2CHandle->pI2Cx->CR1 |= (I2C_ACK);

	//configure the FREQ field of CR2
	pI2CHandle->pI2Cx->CR2 |= ((RCC_GetPCLK1Value() / 1000000U) & 0x3f); // Dividing by 1MHz to get the round number and assigning to FREQ

	//program the device own address
	pI2CHandle->pI2Cx->OAR1 |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	pI2CHandle->pI2Cx->OAR1 |= (BIT14);

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		//mode is standard mode where T_high = T_low
		tempreg &= ~(I2C_FS);
		ccr_value = (RCC_GetPCLK1Value()
				/ (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	} else {
		//mode is fast mode
		tempreg |= (I2C_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = (RCC_GetPCLK1Value()
					/ (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else {
			ccr_value = (RCC_GetPCLK1Value()
					/ (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		//mode is standard mode

		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;

	} else {
		//mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}

/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	I2C_PERI_DEINIT(pI2Cx);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
	if (pI2Cx->SR1 & FlagName) {
		return SET;
	}
	return RESET;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (I2C_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1; // Only 7 bits
	SlaveAddr &= ~(BIT0); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, I2C_ACK_CONTROL EnorDi) {
	if (EnorDi == I2C_ACK_EN) {
		//enable the ack
		pI2Cx->CR1 |= (I2C_ACK);
	} else {
		//disable the ack
		pI2Cx->CR1 &= ~(I2C_ACK);
	}
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
	uint32_t dummy_read;
	//check for device mode
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_MSL)) {
		//device is in master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_RX) {
			if (pI2CHandle->RxSize == 1) {
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void) dummy_read;
			}

		} else {
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void) dummy_read;

		}

	} else {
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void) dummy_read;
	}

}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (I2C_STOP);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer,
		uint32_t Len, uint8_t SlaveAddr, status Sr) {
	//1. Generating the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirming that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB))
		;

	//3. Sending the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirming that address phase is completed by checking the ADDR flag in the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR))
		;

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. send the data until len becomes 0
	while (Len > 0) {
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE))
			; //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE))
		;
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF))
		;

	//8. Generate STOP condition and master need not wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if (!Sr)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (BIT0); //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint8_t Len, uint8_t SlaveAddr, status Sr) {

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB))
		;

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR))
		;

	//read only 1 byte from slave
	if (Len == 1) {
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DI);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE))
			;

		//generate STOP condition
		if (!Sr)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	//procedure to read data from slave when Len > 1
	if (Len > 1) {
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for (uint32_t i = Len; i > 0; i--) {
			//wait until RXNE becomes 1
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE))
				;

			/*When length is 2, disable ACKing
			 * Last data is received in RX Buffer. 2nd last data is going to be read and Len decremented to 1.
			 * When the last data byte is read, NACK will be sent as required followed by a P(stop condition).
			 * */
			if (i == 2) {
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DI);

				//generate STOP condition
				if (!Sr)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			//increment the buffer address
			pRxBuffer++;
		}

	}

	//re-enable ACKing
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_EN) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_EN);
	}

}

/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_IRQInterruptConfig(IRQ_Posn IRQNumber, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) //32 to 63
				{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	} else {
		if (IRQNumber <= 31) {
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 6 && IRQNumber < 96) {
			//program ICER2 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}

}

/*********************************************************************
 * @fn      		  - I2C_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */


void I2C_IRQPriorityConfig(IRQ_Posn IRQNumber, uint8_t IRQPriority) {
	NVIC_IPR->IPR[IRQNumber] |= (uint8_t) (IRQPriority
			<< (8U - NVIC_PRIORITY_BITS) & (uint32_t) 0xFFUL);
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_ITBUFEN);

		//enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_ITEVTEN);

		//enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_ITERREN);

	}

	return busystate;
}


/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_ITERREN);
	}

	return busystate;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == DISABLE)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( I2C_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( I2C_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_EN)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( I2C_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( I2C_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
	pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
    return (uint8_t) pI2C->DR;
}


//Common to both Master and Slave
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1   = pI2CHandle->pI2Cx->CR2 & ( I2C_ITEVTEN) ;
	temp2   = pI2CHandle->pI2Cx->CR2 & ( I2C_ITBUFEN) ;
	temp3  = pI2CHandle->pI2Cx->SR1 & ( I2C_SB);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		if(pI2CHandle->TxRxState == I2C_BUSY_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}else if (pI2CHandle->TxRxState == I2C_BUSY_RX )
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & (I2C_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & (I2C_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_TX)
		{
			//make sure that TXE is also set, to ensure the last data byte has been transferred
			if(pI2CHandle->pI2Cx->SR1 & (I2C_TxE) )
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0 ) //Close transmission only if all bytes have been sent.
				{
					//1. generate the STOP condition
					if(pI2CHandle->Sr == DISABLE)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

				}
			}

		}else if (pI2CHandle->TxRxState == I2C_BUSY_RX )
		{
			;
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & (I2C_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only to slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	if(temp1 && temp3)
	{
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}


	temp3  = pI2CHandle->pI2Cx->SR1 & ( I2C_TxE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (I2C_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->SR2 & (I2C_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		    }
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( I2C_RxNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//check device mode .
		if(pI2CHandle->pI2Cx->SR2 & (I2C_MSL))
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & (I2C_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}



/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7
 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( I2C_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( I2C_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( I2C_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( I2C_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(I2C_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( I2C_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( I2C_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( I2C_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( I2C_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}


void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	 if(EnorDi == ENABLE)
	 {
			pI2Cx->CR2 |= ( I2C_ITEVTEN);
			pI2Cx->CR2 |= ( I2C_ITBUFEN);
			pI2Cx->CR2 |= ( I2C_ITERREN);
	 }else
	 {
			pI2Cx->CR2 &= ~( I2C_ITEVTEN);
			pI2Cx->CR2 &= ~( I2C_ITBUFEN);
			pI2Cx->CR2 &= ~( I2C_ITERREN);
	 }

}
