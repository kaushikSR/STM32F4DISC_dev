/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Aug 9, 2019
 *      Author: kaushik
 */

#include "stm32f407xx_gpio.h"

/*
 * @brief : This function turns on the clock for a particular GPIO port
 *
 * @param[in] : GPIOx Port for which the clock must be ENABLED/DISABLED
 * @param[in] : status (ENABLE/DISABLE)
 *
 * @return : None
 */

void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, status EnOrDi) {
	if (EnOrDi == ENABLE) {
		GPIO_PCLK_EN(pGPIOx);
	} else {
		GPIO_PCLK_DI(pGPIOx);
	}
}

/*
 * @brief :This function initializes the GPIO peripheral with the values given by the user as part of the GPIO_Handle_t data structure.
 * @param[in] : A pointer to the GPIO_Handle_t data structure.
 * @return : None
 * */

void GPIO_Init(GPIO_Handle_t* pGPIOHandle) {
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);
	//Configure the GPIO PIN Mode
	uint8_t pin = (uint8_t) pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		pGPIOHandle->pGPIOx->MODER &= ~(3U << (2U * pin));
		pGPIOHandle->pGPIOx->MODER |= pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2U * pin);
	} else {
		//Interrupt modes
		pGPIOHandle->pGPIOx->MODER &= ~(3U << (2U * pin));
		pGPIOHandle->pGPIOx->MODER |= GPIO_MODE_IN << (2U * pin);
		EXTI->IMR |= (1U << pin);
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			EXTI->FTSR |= (1U << pin);
			EXTI->RTSR &= ~(1U << pin);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				== GPIO_MODE_IT_RT) {
			EXTI->RTSR |= (1U << pin);
			EXTI->FTSR &= ~(1U << pin);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				== GPIO_MODE_IT_RFT) {
			EXTI->FTSR |= (1U << pin);
			EXTI->RTSR |= (1U << pin);
		}

		SYSCFG_PCLK_EN(); // Enable clock for SYSCFG
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[pin / 4U] |= (portcode << ((pin % 4U) * 4U));
	}
	//Configure the GPIO Pin Speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3U << (2U * pin));
	pGPIOHandle->pGPIOx->OSPEEDR |= pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2U * pin);

	//Configure the PUPD settings
	pGPIOHandle->pGPIOx->PUPDR &= ~(3U << (2U * pin));
	pGPIOHandle->pGPIOx->PUPDR |=
			pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2U * pin);

	//Configure the OP type
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT) {
		pGPIOHandle->pGPIOx->OTYPER |=
				pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pin;
	} else {
		pGPIOHandle->pGPIOx->OTYPER = 0U;
	}

	//Configure the Alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= GPIO_PIN7) {
		pGPIOHandle->pGPIOx->AFRL &= ~(15U << (4U * pin));
		pGPIOHandle->pGPIOx->AFRL |=
				pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4U * pin);
	} else {
		pGPIOHandle->pGPIOx->AFRH &= ~(15U << (4U * (pin % 8U)));
		pGPIOHandle->pGPIOx->AFRH |=
				pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode
						<< (4U * (pin % 8U));
	}
}


/*
 * @brief :This function deinitializes the GPIO port specified.
 * @param[in] : A pointer to the GPIO port
 * @return : None
 * */

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx) {
		GPIO_PERI_RESET(pGPIOx);
}

/*
 * @brief: This function reads the value of the pin of a particular port
 *
 * @param[in] : A pointer to the GPIO port to which the pin belongs
 * @param[in] : Pin Number to read from
 *
 * @return: value of Pin (either 0 or 1)
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber) {
	return (uint8_t) ((pGPIOx->IDR >> PinNumber) & BIT0);
}

/*
 * @brief: This function reads the value of a particular port
 *
 * @param[in] : A pointer to the GPIO port of interest
 *
 * @return: value of Port (all 16 pins)
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx) {
	return (uint16_t) (pGPIOx->IDR);
}

/*
 * @brief: This function writes the value to the pin of a particular port
 *
 * @param[in] : A pointer to the GPIO port to which the pin belongs
 * @param[in] : Pin Number to write to
 * @param[in] : Value (either 0 or 1)
 *
 * @return: None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber,
		uint8_t value) {

	if (value == SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*
 * @brief: This function writes the value to a particular port
 *
 * @param[in] : A pointer to the GPIO port
 * @param[in] : Value (16 bits)
 *
 * @return: None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value) {
	pGPIOx->ODR = value;
}

/*
 * @brief This function toggles the value of a particular pin
 *
 * @param[in] : A pointer to the GPIO port containing the pin of interest
 * @param[in] : The pin number to be toggled
 * @return : None
 * */

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, GPIO_Pin_No PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * @brief : This function ENABLES/DISABLES the given IRQ on the NVIC
 *
 * @param[in]: IRQ/Position number
 * @param[in]: ENABLE/DISABLE
 * @return : None
 * */

void GPIO_IRQITConfig(IRQ_Posn IRQNumber, status EnOrDi) {
	if (EnOrDi) {
		if (IRQNumber <= 31) {
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
void GPIO_IRQPriorityConfig(IRQ_Posn IRQNumber, uint8_t IRQPriority) {
	NVIC_IPR->IPR[IRQNumber] |= (uint8_t) (IRQPriority
			<< (8U - NVIC_PRIORITY_BITS) & (uint32_t) 0xFFUL);
}

/*
 * @brief : This function clears the pending register of the EXTI controller for a particular EXTI line
 *
 * @param[in]: EXTI line number/Pin Number
 * @return : None
 *
 * @Note : This must be called in all ISR functions for EXTI lines indicating the interrupt/event has been serviced
 */
void GPIO_IRQHandle(uint8_t EXTI_line) {
	if (EXTI->PR & (1 << EXTI_line)) {
		EXTI->PR |= (1 << EXTI_line);
	}
}
