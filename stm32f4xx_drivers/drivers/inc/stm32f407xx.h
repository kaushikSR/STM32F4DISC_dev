/*
 * stm32f407xx.h
 *
 *  Created on: Jul 23, 2019
 *      Author: kaushik
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include<stdint.h>

#define __IO						volatile
#define __I							volatile const

/***************************************** CPU SPECIFIC DETAILS : START *******************************************/
/*
 * Nested Vector Interrupt Controller (NVIC)
 * TODO:Make a structure overlay for NVIC with all its registers
 * */

#define NVIC_BASEADDR				(0xE000E100U)

/*ISER : Interrupt set enable registers*/
#define NVIC_ISER0					((__IO uint32_t*)0xE000E100)
#define NVIC_ISER1					((__IO uint32_t*)0xE000E104)
#define NVIC_ISER2					((__IO uint32_t*)0xE000E108)
#define NVIC_ISER3					((__IO uint32_t*)0xE000E10C)

/*ICER : Interrupt clear enable registers*/
#define NVIC_ICER0					((__IO uint32_t*)0xE000E180)
#define NVIC_ICER1					((__IO uint32_t*)0xE000E184)
#define NVIC_ICER2					((__IO uint32_t*)0xE000E188)
#define NVIC_ICER3					((__IO uint32_t*)0xE000E18C)

/*IPR : Interrupt priority register*/
typedef struct {
	__IO uint8_t IPR[240U];
} NVIC_IPR_RegDef_t;

#define NVIC_IPR					((NVIC_IPR_RegDef_t*)0xE000E400)

#define NVIC_PRIORITY_BITS 			4U

/**************************************CPU SPECIFIC DETAILS : END**************************************************/

/*
 * MEMORY RELATED BASE ADDRESSES
 * */

#define FLASH_BASEADDR				((uint32_t)0x08000000) /*Flash memory base address*/
#define ROM_BASEADDR				((uint32_t)0x1FFF0000) /*System Memory base address*/
#define SRAM1_BASEADDR				((uint32_t)0x20000000) /*SRAM1(112KB) base address*/
#define SRAM2_BASEADDR				((uint32_t)0x2001C000) /*SRAM2(16KB) base address*/
#define SRAM						SRAM1_BASEADDR

/*
 * AHBx and APBx BUS PERIPHERAL BASE ADDRESSES
 * */

#define PERIPH_BASE							((uint32_t)0x40000000) 	/*Peripheral base*/
#define APB1_BASEADDR						PERIPH_BASE 			/*APB1 base address*/
#define APB2_BASEADDR						((uint32_t)0x40010000) 	/*APB2 base address*/
#define AHB1_BASEADDR						((uint32_t)0x40020000) 	/*AHB1 base address*/
#define AHB2_BASEADDR						((uint32_t)0x50000000) 	/*AHB2 base address*/
#define AHB3_BASEADDR						((uint32_t)0xA0000000) 	/*AHB3 base address*/

/*
 * Base addresses of all peripherals hanging on AHB1 bus
 * TODO: Add missing peripherals
 */

#define GPIOA_BASEADDR						(AHB1_BASEADDR)  					/*GPIOA base address*/
#define GPIOB_BASEADDR						(AHB1_BASEADDR + 0x0400)			/*GPIOB base address*/
#define GPIOC_BASEADDR						(AHB1_BASEADDR + 0x0800)			/*GPIOC base address*/
#define GPIOD_BASEADDR						(AHB1_BASEADDR + 0x0C00)			/*GPIOD base address*/
#define GPIOE_BASEADDR						(AHB1_BASEADDR + 0x1000)			/*GPIOE base address*/
#define GPIOF_BASEADDR						(AHB1_BASEADDR + 0x1400)			/*GPIOF base address*/
#define GPIOG_BASEADDR						(AHB1_BASEADDR + 0x1800)			/*GPIOG base address*/
#define GPIOH_BASEADDR						(AHB1_BASEADDR + 0x1C00)			/*GPIOH base address*/
#define GPIOI_BASEADDR						(AHB1_BASEADDR + 0x2000)			/*GPIOI base address*/
#define GPIOJ_BASEADDR						(AHB1_BASEADDR + 0x2400)			/*GPIOJ base address*/
#define GPIOK_BASEADDR						(AHB1_BASEADDR + 0x2800)			/*GPIOK base address*/
#define RCC_BASEADDR						(AHB1_BASEADDR + 0x3800)			/*RCC base address*/

/*
 * Base addresses of all peripherals hanging on APB1 bus
 * TODO: Add missing peripherals
 */

#define I2C1_BASEADDR						(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1_BASEADDR + 0x5C00)
#define SPI2_BASEADDR						(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1_BASEADDR + 0x3C00)
#define USART2_BASEADDR						(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1_BASEADDR + 0x5000)

/*
 * Base addresses of all peripherals hanging on APB2 bus
 * TODO: Add missing peripherals
 */

#define SPI1_BASEADDR						(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR						(APB2_BASEADDR + 0x3400)
#define SPI5_BASEADDR						(APB2_BASEADDR + 0x5000)
#define SPI6_BASEADDR						(APB2_BASEADDR + 0x5400)
#define USART1_BASEADDR						(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2_BASEADDR + 0x1400)
#define EXTI_BASEADDR						(APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR						(APB2_BASEADDR + 0x3800)

/****BIT MASKS****/

#define BIT0								((uint8_t)0x01)
#define BIT1								((uint8_t)0x02)
#define BIT2								((uint8_t)0x04)
#define BIT3								((uint8_t)0x08)
#define BIT4								((uint8_t)0x10)
#define BIT5								((uint8_t)0x20)
#define BIT6								((uint8_t)0x40)
#define BIT7								((uint8_t)0x80)
#define BIT8								((uint16_t)0x0100)
#define BIT9								((uint16_t)0x0200)
#define BIT10								((uint16_t)0x0400)
#define BIT11								((uint16_t)0x0800)
#define BIT12								((uint16_t)0x1000)
#define BIT13								((uint16_t)0x2000)
#define BIT14								((uint16_t)0x4000)
#define BIT15								((uint16_t)0x8000)
#define BIT16								((uint32_t)0x00010000)
#define BIT17								((uint32_t)0x00020000)
#define BIT18								((uint32_t)0x00040000)
#define BIT19								((uint32_t)0x00080000)
#define BIT20								((uint32_t)0x00100000)
#define BIT21								((uint32_t)0x00200000)
#define BIT22								((uint32_t)0x00400000)
#define BIT23								((uint32_t)0x00800000)
#define BIT24								((uint32_t)0x01000000)
#define BIT25								((uint32_t)0x02000000)
#define BIT26								((uint32_t)0x04000000)
#define BIT27								((uint32_t)0x08000000)
#define BIT28								((uint32_t)0x10000000)
#define BIT29								((uint32_t)0x20000000)
#define BIT30								((uint32_t)0x40000000)
#define BIT31								((uint32_t)0x80000000)

/******************************** PERIPHERALS STRUCTURE OVERLAYS **********************************/

/*** RESET AND CLOCK CONTROL(RCC) ***/

typedef struct {
	__IO uint32_t CR; /*RCC clock control register (RCC_CR), Address offset: 0x00, Reset value: 0x0000 XX83 where X is undefined*/
	__IO uint32_t PLLCFGR; /*RCC PLL configuration register (RCC_PLLCFGR), Address offset: 0x04, Reset value: 0x2400 3010*/
	__IO uint32_t CFGR; /*RCC clock configuration register (RCC_CFGR), Address offset: 0x08, Reset value: 0x0000 0000*/
	__IO uint32_t CIR; /*RCC clock interrupt register (RCC_CIR), Address offset: 0x0C, Reset value: 0x0000 0000*/
	__IO uint32_t AHB1RSTR; /*RCC AHB1 peripheral reset register (RCC_AHB1RSTR), Address offset: 0x10, Reset value: 0x0000 0000*/
	__IO uint32_t AHB2RSTR; /*RCC AHB2 peripheral reset register (RCC_AHB2RSTR), Address offset: 0x14, Reset value: 0x0000 0000*/
	__IO uint32_t AHB3RSTR; /*RCC AHB3 peripheral reset register (RCC_AHB3RSTR), Address offset: 0x18, Reset value: 0x0000 0000*/
	uint32_t RESERVED1; /*Reserved bits - length = 32*/
	__IO uint32_t APB1RSTR; /*RCC APB1 peripheral reset register (RCC_APB1RSTR), Address offset: 0x20, Reset value: 0x0000 0000*/
	__IO uint32_t APB2RSTR; /*RCC APB2 peripheral reset register (RCC_APB2RSTR), Address offset: 0x24, Reset value: 0x0000 0000*/
	uint32_t RESERVED2[2]; /*Reserved bits - length = 64*/
	__IO uint32_t AHB1ENR; /*RCC AHB1 peripheral clock enable register (RCC_AHB1ENR), Address offset: 0x30, Reset value: 0x0010 0000*/
	__IO uint32_t AHB2ENR; /*RCC AHB2 peripheral clock enable register (RCC_AHB2ENR), Address offset: 0x34, Reset value: 0x0000 0000*/
	__IO uint32_t AHB3ENR; /*RCC AHB3 peripheral clock enable register (RCC_AHB3ENR), Address offset: 0x38, Reset value: 0x0000 0000*/
	uint32_t RESERVED3; /*Reserved bits - length = 32*/
	__IO uint32_t APB1ENR; /*RCC APB1 peripheral clock enable register(RCC_APB1ENR), Address offset: 0x40, Reset value: 0x0000 0000*/
	__IO uint32_t APB2ENR; /*RCC APB2 peripheral clock enable register (RCC_APB2ENR), Address offset: 0x44, Reset value: 0x0000 0000*/
	uint32_t RESERVED4[2]; /*Reserved bits - length = 64*/
	__IO uint32_t AHB1LPENR; /*RCC AHB1 peripheral clock enable in low power mode register(RCC_AHB1LPENR), Address offset: 0x50, Reset value: 0x7E67 91FF*/
	__IO uint32_t AHB2LPENR; /*RCC AHB2 peripheral clock enable in low power mode register(RCC_AHB2LPENR), Address offset: 0x54, Reset value: 0x0000 00F1*/
	__IO uint32_t AHB3LPENR; /*RCC AHB3 peripheral clock enable in low power mode register(RCC_AHB3LPENR), Address offset: 0x58, Reset value: 0x0000 0001*/
	uint32_t RESERVED5; /*Reserved bits - length = 32*/
	__IO uint32_t APB1LPENR; /*RCC APB1 peripheral clock enable in low power mode register(RCC_APB1LPENR), Address offset: 0x60, Reset value: 0x36FE C9FF*/
	__IO uint32_t APB2LPENR; /*RCC APB2 peripheral clock enable in low power mode register(RCC_APB2LPENR), Address offset: 0x64, Reset value: 0x0007 5F33*/
	uint32_t RESERVED6[2]; /*Reserved bits - length = 64*/
	__IO uint32_t BDCR; /*RCC Backup domain control register (RCC_BDCR), Address offset: 0x70, Reset value: 0x0000 0000*/
	__IO uint32_t CSR; /*RCC clock control & status register (RCC_CSR), Address offset: 0x74, Reset value: 0x0E00 0000*/
	uint32_t RESERVED7[2]; /*Reserved bits - length = 64*/
	__IO uint32_t SSCGR; /*RCC spread spectrum clock generation register (RCC_SSCGR), Address offset: 0x80, Reset value: 0x0000 0000*/
	__IO uint32_t PLLI2SCFGR; /*RCC PLLI2S configuration register (RCC_PLLI2SCFGR), Address offset: 0x84, Reset value: 0x2000 3000*/
} RCC_RegDef_t;

#define RCC 								((RCC_RegDef_t*)RCC_BASEADDR)

/************************ General-purpose I/Os (GPIO) *************************/

typedef struct {
	__IO uint32_t MODER; /*GPIO port mode register (GPIOx_MODER) (x = A..I/J/K), Address offset: 0x00*/
	__IO uint32_t OTYPER; /*GPIO port output type register (GPIOx_OTYPER)(x = A..I/J/K), Address offset: 0x04*/
	__IO uint32_t OSPEEDR; /*GPIO port output speed register (GPIOx_OSPEEDR)(x = A..I/J/K), Address offset: 0x08*/
	__IO uint32_t PUPDR; /*GPIO port pull-up/pull-down register (GPIOx_PUPDR)(x = A..I/J/K), Address offset: 0x0C*/
	__I uint32_t IDR; /*GPIO port input data register (GPIOx_IDR) (x = A..I/J/K), Address offset: 0x10*/
	__IO uint32_t ODR; /*GPIO port output data register (GPIOx_ODR) (x = A..I/J/K), Address offset: 0x14*/
	__IO uint32_t BSRR; /*GPIO port bit set/reset register (GPIOx_BSRR) (x = A..I/J/K), Address offset: 0x18*/
	__IO uint32_t LCKR; /*GPIO port configuration lock register (GPIOx_LCKR)(x = A..I/J/K), Address offset: 0x1C*/
	__IO uint32_t AFRL; /*GPIO alternate function low register (GPIOx_AFRL) (x = A..I/J/K), Address offset: 0x20*/
	__IO uint32_t AFRH; /*GPIO alternate function high register (GPIOx_AFRH)(x = A..I/J), Address offset: 0x24*/
} GPIO_RegDef_t;

#define GPIOA 								((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 								((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 								((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 								((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 								((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 								((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 								((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 								((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 								((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ 								((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK 								((GPIO_RegDef_t*)GPIOK_BASEADDR)

/*Macro to resolve bit number based on GPIO Port*/
#define RESOLVE_GPIO(x)						    ( (x) == GPIOA ? BIT0: \
											(x) == GPIOB ? BIT1: \
											(x) == GPIOC ? BIT2: \
										    (x) == GPIOD ? BIT3: \
											(x) == GPIOE ? BIT4: \
											(x) == GPIOF ? BIT5: \
											(x) == GPIOG ? BIT6: \
											(x) == GPIOH ? BIT7: \
											(x) == GPIOI ? BIT8: -1 )

/***Macro needed to get appropriate port code***/
#define GPIO_BASEADDR_TO_CODE(x)			( (x) == GPIOA ? 0:\
											(x) == GPIOB ? 1: \
											(x) == GPIOC ? 2: \
											(x) == GPIOD ? 3: \
											(x) == GPIOE ? 4: \
											(x) == GPIOF ? 5: \
											(x) == GPIOG ? 6: \
											(x) == GPIOH ? 7: \
											(x) == GPIOI ? 8: -1 )

/***Clock ENABLE/DISABLE macros for GPIO
 * @param[in] GPIOx(A..J,K)
 ***/

#define GPIO_PCLK_EN(x)						(RCC->AHB1ENR |=(RESOLVE_GPIO((x))))
#define GPIO_PCLK_DI(x)						(RCC->AHB1ENR &=~(RESOLVE_GPIO((x))))

/***GPIO Port reset Macro
 * @param[in] GPIOx(A..J,K)
 ***/
#define GPIO_PERI_RESET(x)					do{RCC->AHB1RSTR |=(RESOLVE_GPIO((x)));\
											RCC->AHB1RSTR &=~(RESOLVE_GPIO((x)));}while(0)

/********************External interrupt/event controller (EXTI)******************/

typedef struct {
	__IO uint32_t IMR; /*Interrupt mask register (EXTI_IMR), Address offset: 0x00*/
	__IO uint32_t EMR; /*Event mask register (EXTI_EMR), Address offset: 0x04*/
	__IO uint32_t RTSR; /*Rising trigger selection register (EXTI_RTSR), Address offset: 0x08*/
	__IO uint32_t FTSR; /*Falling trigger selection register (EXTI_FTSR), Address offset: 0x0C*/
	__IO uint32_t SWIER; /*Software interrupt event register (EXTI_SWIER), Address offset: 0x10*/
	__IO uint32_t PR; /*Pending register (EXTI_PR), Address offset: 0x14*/
} EXTI_RegDef_t;

#define EXTI								((EXTI_RegDef_t*)EXTI_BASEADDR)

/*******************System configuration controller (SYSCFG)***************************/

typedef struct {
	__IO uint32_t MEMRMP; /*SYSCFG memory remap register (SYSCFG_MEMRMP), Address offset: 0x00*/
	__IO uint32_t PMC; /*SYSCFG peripheral mode configuration register (SYSCFG_PMC), Address offset: 0x04*/
	__IO uint32_t EXTICR[4]; /*SYSCFG external interrupt configuration register 1-4(SYSCFG_EXTICR1-4), Address offset: 0x08-0x14*/
	uint32_t RESERVED1[2]; /*Reserved bits - length = 64*/
	__IO uint32_t CMPCR; /*Compensation cell control register (SYSCFG_CMPCR), Address offset: 0x20*/

} SYSCFG_Regdef_t;

#define SYSCFG								((SYSCFG_Regdef_t*)SYSCFG_BASEADDR)

/********************Inter-integrated circuit (I2C) interface**************************/

typedef struct {
	__IO uint32_t CR1;/*I2C Control register 1 (I2C_CR1), Address offset: 0x0*/
	__IO uint32_t CR2;/*I2C Control register 2 (I2C_CR2), Address offset: 0x04*/
	__IO uint32_t OAR1;/*I2C Own address register 1 (I2C_OAR1), Address offset: 0x08*/
	__IO uint32_t OAR2;/*I2C Own address register 2 (I2C_OAR2), Address offset: 0x0C*/
	__IO uint32_t DR;/*I2C Data register (I2C_DR), Address offset: 0x10*/
	__IO uint32_t SR1;/*I2C Status register 1 (I2C_SR1), Address offset: 0x14*/
	__IO uint32_t SR2;/*I2C Status register 2 (I2C_SR2), Address offset: 0x18*/
	__IO uint32_t CCR;/*I2C Clock control register (I2C_CCR), Address offset: 0x1C*/
	__IO uint32_t TRISE;/*I2C TRISE register (I2C_TRISE), Address offset: 0x20*/
	__IO uint32_t FLTR;/*I2C FLTR register (I2C_FLTR), Address offset: 0x24*/
} I2C_RegDef_t;

#define I2C1								((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2								((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3								((I2C_RegDef_t*)I2C3_BASEADDR)

/***Clock ENABLE/DISABLE macros for I2C***/

#define I2C1_PLCK_EN()						(RCC->APB1ENR |=(BIT21))
#define I2C2_PLCK_EN()						(RCC->APB1ENR |=(BIT22))
#define I2C3_PLCK_EN()						(RCC->APB1ENR |=(BIT23))

#define I2C1_PLCK_DI()						(RCC->APB1ENR &=~(BIT21))
#define I2C2_PLCK_DI()						(RCC->APB1ENR &=~(BIT22))
#define I2C3_PLCK_DI()						(RCC->APB1ENR &=~(BIT23))
/*********************************Serial peripheral interface (SPI)**************************************/

typedef struct {
	__IO uint32_t CR1;/*SPI control register 1 (SPI_CR1), Address offset: 0x00*/
	__IO uint32_t CR2;/*SPI control register 2 (SPI_CR2), Address offset: 0x04*/
	__IO uint32_t SR;/*SPI status register (SPI_SR), Address offset: 0x08*/
	__IO uint32_t DR;/*SPI data register (SPI_DR), Address offset: 0x0C*/
	__IO uint32_t CRCPR; /*SPI CRC polynomial register (SPI_CRCPR), Address offset: 0x10*/
	__IO uint32_t RXCRCR;/*SPI RX CRC register (SPI_RXCRCR), Address offset: 0x14*/
	__IO uint32_t TXCRCR;/*SPI TX CRC register (SPI_TXCRCR), Address offset: 0x18*/
	__IO uint32_t I2SCFGR;/*SPI_I2S configuration register (SPI_I2SCFGR), Address offset: 0x1C*/
	__IO uint32_t I2SPR; /*SPI_I 2 S prescaler register (SPI_I2SPR), Address offset: 0x20*/

} SPI_RegDef_t;

#define SPI1								((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2								((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3								((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4								((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI5								((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI6								((SPI_RegDef_t*)SPI3_BASEADDR)

/*Macro to resolve bit number based on SPI peripheral*/
#define RESOLVE_SPI_BIT(x)					( (x) == SPI1 ? BIT12: \
											(x) == SPI2 ? BIT14: \
											(x) == SPI3 ? BIT15: -1)

/***Clock ENABLE/DISABLE macros for SPI***/

#define SPI1_PCLK_EN()						(RCC->APB2ENR |=(BIT12))
#define SPI2_PCLK_EN()						(RCC->APB1ENR |=(BIT14))
#define SPI3_PCLK_EN()						(RCC->APB1ENR |=(BIT15))

#define SPI1_PCLK_DI()						(RCC->APB2ENR &=~(BIT12))
#define SPI2_PCLK_DI()						(RCC->APB1ENR &=~(BIT14))
#define SPI3_PCLK_DI()						(RCC->APB1ENR &=~(BIT15))

#define SPI1_DEINIT()						do{ RCC->APB2RSTR |= (BIT12); RCC->APB2RSTR &= ~(BIT12);}while(0)
#define SPI2_DEINIT() 						do{ RCC->APB1RSTR |= (BIT14); RCC->APB1RSTR &= ~(BIT14);}while(0)
#define SPI3_DEINIT()						do{ RCC->APB1RSTR |= (BIT15); RCC->APB1RSTR &= ~(BIT15);}while(0)

/***Clock ENABLE/DISABLE macros for UART/USART ***/

#define USART1_PCLK_EN()					(RCC->APB2ENR |=(BIT4))
#define USART2_PCLK_EN()					(RCC->APB1ENR |=(BIT17))
#define USART3_PCLK_EN()					(RCC->APB1ENR |=(BIT18))
#define UART4_PCLK_EN()						(RCC->APB1ENR |=(BIT19))
#define UART5_PCLK_EN()						(RCC->APB1ENR |=(BIT20))
#define USART6_PCLK_EN()					(RCC->APB2ENR |=(BIT5))

#define USART1_PCLK_DI()					(RCC->APB2ENR &=~(BIT4))
#define USART2_PCLK_DI()					(RCC->APB1ENR &=~(BIT17))
#define USART3_PCLK_DI()					(RCC->APB1ENR &=~(BIT18))
#define UART4_PCLK_DI()						(RCC->APB1ENR &=~(BIT19))
#define UART5_PCLK_DI()						(RCC->APB1ENR &=~(BIT20))
#define USART6_PCLK_DI()					(RCC->APB2ENR &=~(BIT5))

/*** Clock ENABLE/DISABLE macros for SYSCFG ***/
#define SYSCFG_PCLK_EN()					(RCC->APB2ENR |=(BIT14))
#define SYSCFG_PCLK_DI()					(RCC->APB2ENR &=~(BIT14))

/*
 * IRQ/Position Numbers for NVIC
 * TODO: Add the remaining IRQ numbers
 * */

typedef enum {
	EXTI0 = 6U,
	EXTI1 = 7U,
	EXTI2 = 8U,
	EXTI3 = 9U,
	EXTI4 = 10U,
	EXTI9_5 = 23,
	EXTI15_10 = 40
} IRQ_Posn;

/*Status Enumeration*/
typedef enum {
	ENABLE = 1, SET = 1, DISABLE = 0, RESET = 0
} status;

#endif /* INC_STM32F407XX_H_ */
