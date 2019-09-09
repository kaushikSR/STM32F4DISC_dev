/*
 * stm32f407xx_i2c.h
 *
 *  Created on: Aug 19, 2019
 *      Author: kaushik
 */

#ifndef INC_STM32F407XX_I2C_H_
#define INC_STM32F407XX_I2C_H_
#include "stm32f407xx.h"
/***Register Enumerations ***/

typedef enum{
	I2C_PE = BIT0,
	I2C_SMBUS = BIT1,
	I2C_SMBTYPE = BIT3,
	I2C_ENARP = BIT4,
	I2C_ENPEC = BIT5,
	I2C_ENGC = BIT6,
	I2C_NOSTRETCH = BIT7,
	I2C_START = BIT8,
	I2C_STOP = BIT9,
	I2C_ACK = BIT10,
	I2C_POS = BIT11,
	I2C_PEC = BIT12,
	I2C_ALERT = BIT13,
	I2C_SWRST = BIT15
}I2C_CR1;

typedef enum{
	I2C_FREQ = BIT0,
	I2C_ITERREN = BIT8,
	I2C_ITEVTEN = BIT9,
	I2C_ITBUFEN = BIT10,
	I2C_DMAEN = BIT11,
	I2C_LAST = BIT12
}I2C_CR2;

typedef enum{
	I2C_SB = BIT0,
	I2C_ADDR = BIT1,
	I2C_BTF = BIT2,
	I2C_ADD10 = BIT3,
	I2C_STOPF = BIT4,
	I2C_RxNE = BIT6,
	I2C_TxE = BIT7,
	I2C_BERR = BIT8,
	I2C_ARLO = BIT9,
	I2C_AF = BIT10,
	I2C_OVR = BIT11,
	I2C_PECERR = BIT12,
	I2C_TIMEOUT = BIT14,
	I2C_SMBALERT = BIT15
}I2C_SR1;

typedef enum{
	I2C_MSL = BIT0,
	I2C_BUSY = BIT1,
	I2C_TRA = BIT2,
	I2C_GENCALL = BIT4,
	I2C_SMBDEFAULT = BIT5,
	I2C_SMBHOST = BIT6,
	I2C_DUALF = BIT7,
	PEC = BIT8
}I2C_SR2;


typedef enum{
	I2C_CCR_ = BIT0,
	I2C_DUTY = BIT14,
	I2C_FS = BIT15
}I2C_CCR;

typedef enum{
	I2C_TRISE_ = BIT0
}I2C_TRISE;


/*I2C SCLSpeed*/

typedef enum{
	I2C_SCL_SPEED_SM = 100000,
	I2C_SCL_SPEED_FM4K = 400000,
	I2C_SCL_SPEED_FM2K = 200000
}I2C_SCL_SPEED;

/*I2C ACK control*/

typedef enum{
	I2C_ACK_DI = 0,
	I2C_ACK_EN = 1
}I2C_ACK_CONTROL;

/*I2C DUTY CYCLE*/

typedef enum{
	I2C_FM_DUTY_2 = 0,
	I2C_FM_DUTY_16_9 = 1
}I2C_DUTY_CYCLE;

/*I2C State*/

typedef enum{
	I2C_BUSY_TX = 0,
	I2C_BUSY_RX = 1,
	I2C_READY = 2
}I2C_STATE;


/*I2C Events*/

typedef enum{
	I2C_EV_RX_CMPLT = 0,
	I2C_EV_TX_CMPLT = 1,
	I2C_EV_STOP = 2,
	I2C_EV_DATA_REQ = 3,
	I2C_EV_DATA_RCV = 4
}I2C_EVENTS;

typedef enum{
	I2C_ERROR_BERR = 0,
	I2C_ERROR_ARLO = 1,
	I2C_ERROR_AF = 2,
	I2C_ERROR_OVR = 3,
	I2C_ERROR_TIMEOUT = 4
}I2C_ERROR;

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	I2C_SCL_SPEED I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	I2C_ACK_CONTROL  I2C_AckControl;
	I2C_DUTY_CYCLE  I2C_FMDutyCycle;
}I2C_Config_t;

/*
 *Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
    uint32_t        RxSize;		/* !< To store Rx size  > */
    uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, status EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);


void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(IRQ_Posn IRQNumber, status EnorDi);
void I2C_IRQPriorityConfig(IRQ_Posn IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, status EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, I2C_ACK_CONTROL EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,status EnorDi);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);


#endif /* INC_STM32F407XX_I2C_H_ */
