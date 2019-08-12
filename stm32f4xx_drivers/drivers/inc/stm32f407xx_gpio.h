/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Aug 9, 2019
 *      Author: kaushik
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"

/*
 * @GPIO_PIN_NUMBERS
 * */

typedef enum {
	GPIO_PIN0 = 0,
	GPIO_PIN1 = 1,
	GPIO_PIN2 = 2,
	GPIO_PIN3 = 3,
	GPIO_PIN4 = 4,
	GPIO_PIN5 = 5,
	GPIO_PIN6 = 6,
	GPIO_PIN7 = 7,
	GPIO_PIN8 = 8,
	GPIO_PIN9 = 9,
	GPIO_PIN10 = 10,
	GPIO_PIN11 = 11,
	GPIO_PIN12 = 12,
	GPIO_PIN13 = 13,
	GPIO_PIN14 = 14,
	GPIO_PIN15 = 15,
	GPIO_PIN16 = 16,
	GPIO_PIN17 = 17,
	GPIO_PIN18 = 18,
	GPIO_PIN19 = 19,
	GPIO_PIN20 = 20,
	GPIO_PIN21 = 21,
	GPIO_PIN22 = 22,
	GPIO_PIN23 = 23,
	GPIO_PIN24 = 24,
	GPIO_PIN25 = 25,
	GPIO_PIN26 = 26,
	GPIO_PIN27 = 27,
	GPIO_PIN28 = 28,
	GPIO_PIN29 = 29,
	GPIO_PIN30 = 30,
	GPIO_PIN31 = 31
} GPIO_Pin_No;

/*
 * @GPIO_PORT_MODES
 * */

typedef enum {
	GPIO_MODE_IN = 0,
	GPIO_MODE_OUT = 1,
	GPIO_MODE_ALTFN = 2,
	GPIO_MODE_ANALOG = 3,
	GPIO_MODE_IT_FT = 4,
	GPIO_MODE_IT_RT = 5,
	GPIO_MODE_IT_RFT = 6
} Gpio_Pin_Mode_Types;

/* @GPIO_PORT_OPTYPE
 * GPIO port output types
 * */

typedef enum{
	GPIO_OP_TYPE_PP = 0,
	GPIO_OP_TYPE_OD = 1
}Op_Type;

/*
 * @GPIO_PORT_SPEED
 * GPIO port output speed register
 * */

typedef enum {
	GPIO_OP_SPEED_LOW = 0,
	GPIO_OP_SPEED_MEDIUM = 1,
	GPIO_OP_SPEED_HIGH = 2,
	GPIO_OP_SPEED_VHIGH = 3
} Speed_type;

/*
 * @GPIO_PORT_PUPD
 * GPIO port pull-up/pull-down
 * */

typedef enum {
	GPIO_NO_PUPD = 0, GPIO_PU = 1, GPIO_PD = 2
} Pupd_Type;

/*
 * @GPIO_ALT_FUNC_MODES
 * GPIO Alternate Functionality modes
 * */

typedef enum {
	AF0 = 0,
	AF1 = 1,
	AF2 = 2,
	AF3 = 3,
	AF4 = 4,
	AF5 = 5,
	AF6 = 6,
	AF7 = 7,
	AF8 = 8,
	AF9 = 9,
	AF10 = 10,
	AF11 = 11,
	AF12 = 12,
	AF13 = 13,
	AF14 = 14,
	AF15 = 15
} Alt_Func_type;

/*Pin configuration Data Structure*/

typedef struct {
	GPIO_Pin_No GPIO_PinNumber; /*Possible values from @GPIO_PIN_NUMBERS*/
	Gpio_Pin_Mode_Types GPIO_PinMode; /*Possible values from @GPIO_PORT_MODES*/
	Speed_type GPIO_PinSpeed; /*Possible values from @GPIO_PORT_SPEED*/
	Pupd_Type GPIO_PinPuPdControl; /*Possible values from @GPIO_PORT_PUPD*/
	Op_Type GPIO_PinOPType; /*Possible values from @GPIO_PORT_OPTYPE*/
	Alt_Func_type GPIO_PinAltFunMode; /*Possible values from @GPIO_ALT_FUNC_MODES*/
} GPIO_PinConfig_t;

/*GPIO port configuration*/

typedef struct {
	GPIO_RegDef_t* pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*******************************************APIs supported*******************************/
/*
 * @brief : This function turns on the clock for a particular GPIO port
 *
 * @param[in] : GPIOx Port for which the clock must be ENABLED/DISABLED
 * @param[in] : status (ENABLE/DISABLE)
 *
 * @return : None
 * */

void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, status EnOrDi);

/*
 * @brief :This function initializes the GPIO peripheral with the values given by the user as part of the GPIO_Handle_t data structure.
 * @param[in] : A pointer to the GPIO_Handle_t data structure.
 * @return : None
 * */

void GPIO_Init(GPIO_Handle_t* pGPIOHandle);

/*
 * @brief :This function deinitializes the GPIO port specified.
 * @param[in] : A pointer to the GPIO port
 * @return : None
 * */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/*
 * @brief: This function reads the value of the pin of a particular port
 *
 * @param[in] : A pointer to the GPIO port to which the pin belongs
 * @param[in] : Pin Number to read from
 *
 * @return: value of Pin (either 0 or 1)
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, GPIO_Pin_No PinNumber);

/*
 * @brief: This function reads the value of a particular port
 *
 * @param[in] : A pointer to the GPIO port of interest
 *
 * @return: value of Port (all 16 pins)
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);

/*
 * @brief: This function writes the value to the pin of a particular port
 *
 * @param[in] : A pointer to the GPIO port to which the pin belongs
 * @param[in] : Pin Number to write to
 * @param[in] : Value (either 0 or 1)
 *
 * @return: None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, GPIO_Pin_No PinNumber,
		status value);

/*
 * @brief: This function writes the value to a particular port
 *
 * @param[in] : A pointer to the GPIO port
 * @param[in] : Value (16 bits)
 *
 * @return: None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value);

/*
 * @brief This function toggles the value of a particular pin
 *
 * @param[in] : A pointer to the GPIO port containing the pin of interest
 * @param[in] : The pin number to be toggled
 * @return : None
 * */

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, GPIO_Pin_No PinNumber);

/*
 * @brief : This function ENABLES/DISABLES the given IRQ on the NVIC
 *
 * @param[in]: IRQ/Position number
 * @param[in]: ENABLE/DISABLE
 * @return : None
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnOrDi);

/*
 * @brief : This function sets the priority of the given IRQ in the NVIC
 *
 * @param[in]: IRQ/Position number
 * @param[in]: Priority
 * @return : None
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * @brief : This function clears the pending register of the EXTI controller for a particular EXTI line
 *
 * @param[in]: EXTI line number/Pin Number
 * @return : None
 *
 * @Note : This must be called in all ISR functions for EXTI lines indicating the interrupt/event has been serviced
 */
void GPIO_IRQHandle(uint8_t EXTI_line) ;

#endif /* INC_STM32F407XX_GPIO_H_ */
