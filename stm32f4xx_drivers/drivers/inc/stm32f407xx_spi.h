/*
 * stm32f407xx_spi.h
 *
 *  Created on: Aug 13, 2019
 *      Author: kaushik
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

#include "stm32f407xx.h"

/***Configuration Enums***/

typedef enum{
	SLAVE = 0,
	MASTER = 1
}SPI_DevMode;

typedef enum{
	BIDI_TXRX = 0, // 2-line Unidirectional - Transmit and Receive
	UNIDI_RX = 1, // 1-line Bidirectional - Receive only
	UNIDI_TX = 2, // 1-line Bidirectional - Transmit only
	BIDI_RX = 3 // 2-line Unidirectional - Receive only
}SPI_BusMode;

typedef enum{
	DIV2 = 0,
	DIV4 = 1,
	DIV8 = 2,
	DIV16 = 3,
	DIV32 = 4,
	DIV64 = 5,
	DIV128 = 6,
	DIV256 = 7
}SPI_PreScalar;

typedef enum{
	BYTE = 0,
	TWO_BYTE = 1
}SPI_DataFrame;

typedef enum{
	IDLE0 = 0,
	IDLE1 = 1
}SPI_ClkPol;

typedef enum{
	LEADING_EDGE = 0,
	TRAILING_EDGE = 1
}SPI_ClkPhase;

/***Register Enumerations ***/

/*SPI_CR1 Register*/
typedef enum{
	CPHA = 0,
	CPOL = 1,
	MSTR = 2,
	BR = 3,
	SPE = 6,
	LSBFIRST = 7,
	SSI = 8,
	SSM = 9,
	RX_ONLY = 10,
	DFF = 11,
	CRC_NEXT = 12,
	CRC_EN = 13,
	BIDI_OE = 14,
	BIDI_MODE = 15
}SPI_CR1;



/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct {
	SPI_DevMode SPI_DeviceMode;
	SPI_BusMode SPI_BusConfig;
	SPI_PreScalar SPI_SclkSpeed;
	SPI_DataFrame SPI_DFF;
	SPI_ClkPol SPI_CPOL;
	SPI_ClkPhase SPI_CPHA;
	status SPI_SSM;
	status SPI_SSI;
} SPI_Config_t;

/*
 *Handle structure for SPIx peripheral
 */
typedef struct {
	SPI_RegDef_t *pSPIx; /*!< This holds the base address of SPIx(x:1,2,3) peripheral >*/
	SPI_Config_t SPIConfig;
//	uint8_t *pTxBuffer; /* !< To store the app. Tx buffer address > */
//	uint8_t *pRxBuffer; /* !< To store the app. Rx buffer address > */
//	uint32_t TxLen; /* !< To store Tx len > */
//	uint32_t RxLen; /* !< To store Tx len > */
//	uint8_t TxState; /* !< To store Tx state > */
//	uint8_t RxState; /* !< To store Rx state > */
} SPI_Handle_t;

#endif /* INC_STM32F407XX_SPI_H_ */

/*****************************************SUPPORTED APIs*********************************************/

/*@brief: This function helps to configure the SPI peripheral Clock
 *@param[in] : A pointer to the SPIx base address
 *@param[in] : ENABLE or DISABLE
 *@return : None
 * */

void SPI_PCLK_Config(SPI_RegDef_t *pSPIx, status ENorDI);

/*@brief: This function helps to initialize the SPI peripheral
 *@param[in] : A pointer to the SPI Handle data structure
 *@return : None
 * */

void SPI_Init(SPI_Handle_t* pSPIHandle);

/*@brief: This function helps to deinitialize the SPI peripheral
 *@param[in] : A pointer to the SPI Handle data structure
 *@return : None
 * */

void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*@brief: These functions can be used to send/receive data (Polling/Blocking method)
 *@param[in] : A pointer to the SPI base address
 *@param[in] : A pointer to the Tx/Rx Buffer
 *@param[in] : Length of data/buffer
 *@return : None
 * */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * @brief : This function ENABLES/DISABLES the given IRQ on the NVIC
 *
 * @param[in]: IRQ/Position number
 * @param[in]: ENABLE/DISABLE
 * @return : None
 */
void SPI_IRQITConfig(uint8_t IRQNumber, status EnOrDi);

/*
 * @brief : This function sets the priority of the given IRQ in the NVIC
 *
 * @param[in]: IRQ/Position number
 * @param[in]: Priority
 * @return : None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * @brief : This function clears the pending register of the EXTI controller for a particular EXTI line
 *
 * @param[in]: EXTI line number/Pin Number
 * @return : None
 *
 * @Note : This must be called in all ISR functions for EXTI lines indicating the interrupt/event has been serviced
 */
void SPI_IRQHandle(uint8_t EXTI_line) ;
