/*
 * stm32f407xx_spi.h
 *
 *  Created on: Aug 13, 2019
 *      Author: kaushik
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

/*
 *  Configuration structure for SPIx peripheral
 */

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

/*
 *Handle structure for SPIx peripheral
 */
typedef struct {
	SPI_RegDef_t *pSPIx; /*!< This holds the base address of SPIx(x:1,2,3) peripheral >*/
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t *pRxBuffer; /* !< To store the app. Rx buffer address > */
	uint32_t TxLen; /* !< To store Tx len > */
	uint32_t RxLen; /* !< To store Tx len > */
	uint8_t TxState; /* !< To store Tx state > */
	uint8_t RxState; /* !< To store Rx state > */
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

void SPI_DeInit(SPI_Handle_t* pSPIHandle);

/*@brief: These functions can be used to send/receive data (Polling method)
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
