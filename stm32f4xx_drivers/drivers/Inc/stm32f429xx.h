/*
 * stm32f429xx.h
 *
 *  Created on: Jun 18, 2024
 *      Author: abhisar
 */

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include<stdint.h>

/**************START:Processor Specific Details******************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0 			((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1 			((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2 			((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3 			((volatile uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0 			((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1 			((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2 			((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3 			((volatile uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor Priority register address
 */
#define NVIC_PR_BASE_ADDR	((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 			4

/*
 * Base addresses of FLASH and SRAM memories
 */
#define FLASH_BASEADDR		0x08000000U	/* Base address of FLASH memory */
#define SRAM1_BASEADDR		0x20000000U /* Base address of SRAM1 memory (112 KB) */
#define SRAM2_BASEADDR		0x2001C000U /* Base address of SRAM2 memory (16 KB) */
#define ROM					0x1FFF0000U /* Base address of RAM memory */
#define SRAM				SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASE + 0x2800)

#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADD			(APB1PERIPH_BASE + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800)


/*************Peripheral register definition structures*********/
/*
 *
 */

typedef struct
{
	volatile uint32_t MODER;	/* GPIO port mode register | Address offset: 0x00 */
	volatile uint32_t OTYPER;	/* GPIO port output type register | Address offset: 0x04 */
	volatile uint32_t OSPEEDR;	/* GPIO port output speed register | Address offset: 0x08 */
	volatile uint32_t PUPDR;	/* GPIO port pull-up/pull-down register | Address offset: 0x0C */
	volatile uint32_t IDR;		/* GPIO port input data register | Address offset: 0x10 */
	volatile uint32_t ODR;		/* GPIO port output data register | Address offset: 0x14 */
	volatile uint32_t BSRR;		/* GPIO port bit set/reset register | Address offset: 0x18 */
	volatile uint32_t LCKR;		/* GPIO port configuration lock register | Address offset: 0x1C */
	volatile uint32_t AFR[2];	/* GPIO alternate function low & high register | Address offset: 0x20 */
}GPIO_Regdef_t;

typedef struct
{
	volatile uint32_t CR;			/* RCC clock control register | Address offset: 0x00 */
	volatile uint32_t PLLCFGR;		/* RCC PLL configuration register | Address offset: 0x04 */
	volatile uint32_t CFGR;			/* RCC clock configuration register | Address offset: 0x08 */
	volatile uint32_t CIR;			/* RCC clock interrupt register | Address offset: 0x0C */
	volatile uint32_t AHB1RSTR;		/* RCC AHB1 peripheral reset register | Address offset: 0x10 */
	volatile uint32_t AHB2RSTR;		/* RCC AHB2 peripheral reset register | Address offset: 0x14 */
	volatile uint32_t AHB3RSTR;		/* RCC AHB3 peripheral reset register | Address offset: 0x18 */
	uint32_t RESERVED_1;
	volatile uint32_t APB1RSTR;		/* RCC APB1 peripheral reset register | Address offset: 0x20 */
	volatile uint32_t APB2RSTR;		/* RCC APB2 peripheral reset register | Address offset: 0x24 */
	uint32_t RESERVED_2;
	uint32_t RESERVED_3;
	volatile uint32_t AHB1ENR;		/* RCC AHB1 peripheral clock enable register | Address offset: 0x30 */
	volatile uint32_t AHB2ENR;		/* RCC AHB2 peripheral clock enable register | Address offset: 0x34 */
	volatile uint32_t AHB3ENR;		/* RCC AHB3 peripheral clock enable register | Address offset: 0x38 */
	uint32_t RESERVED_4;
	volatile uint32_t APB1ENR;		/* RCC APB1 peripheral clock enable register | Address offset: 0x40 */
	volatile uint32_t APB2ENR;		/* RCC APB2 peripheral clock enable register | Address offset: 0x44 */
	uint32_t RESERVED_5;
	uint32_t RESERVED_6;
	volatile uint32_t AHB1LPENR;	/* RCC AHB1 peripheral clock enable in low power mode register | Address offset: 0x50 */
	volatile uint32_t AHB2LPENR;	/* RCC AHB2 peripheral clock enable in low power mode register | Address offset: 0x54 */
	volatile uint32_t AHB3LPENR;	/* RCC AHB3 peripheral clock enable in low power mode register | Address offset: 0x58 */
	uint32_t RESERVED_7;
	volatile uint32_t APB1LPENR;	/* RCC APB1 peripheral clock enable in low power mode register | Address offset: 0x60 */
	volatile uint32_t APB2LPENR;	/* RCC APB2 peripheral clock enabled in low power mode register | Address offset: 0x64 */
	uint32_t RESERVED_8;
	uint32_t RESERVED_9;
	volatile uint32_t BDCR;			/* RCC Backup domain control register | Address offset: 0x70 */
	volatile uint32_t CSR;			/* RCC clock control & status register | Address offset: 0x74 */
	uint32_t RESERVED_10;
	uint32_t RESERVED_11;
	volatile uint32_t SSCGR;		/* RCC spread spectrum clock generation register | Address offset: 0x80 */
	volatile uint32_t PLLI2SCFGR;	/* RCC PLLI2S configuration register | Address offset: 0x84 */
}RCC_Regdef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
}SYSCFG_RegDef_t;

/*
 *	Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA		((GPIO_Regdef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_Regdef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_Regdef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_Regdef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_Regdef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_Regdef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_Regdef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_Regdef_t*)GPIOH_BASEADDR)
#define GPIOI		((GPIO_Regdef_t*)GPIOI_BASEADDR)
#define GPIOJ		((GPIO_Regdef_t*)GPIOJ_BASEADDR)
#define GPIOK		((GPIO_Regdef_t*)GPIOK_BASEADDR)

#define RCC			((RCC_Regdef_t*)RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN()		{ RCC->AHB1ENR |= (1 << 0); }
#define GPIOB_PCLK_EN()		{ RCC->AHB1ENR |= (1 << 1); }
#define GPIOC_PCLK_EN()		{ RCC->AHB1ENR |= (1 << 2); }
#define GPIOD_PCLK_EN()		{ RCC->AHB1ENR |= (1 << 3); }
#define GPIOE_PCLK_EN()		{ RCC->AHB1ENR |= (1 << 4); }
#define GPIOF_PCLK_EN()		{ RCC->AHB1ENR |= (1 << 5); }
#define GPIOG_PCLK_EN()		{ RCC->AHB1ENR |= (1 << 6); }
#define GPIOH_PCLK_EN()		{ RCC->AHB1ENR |= (1 << 7); }
#define GPIOI_PCLK_EN()		{ RCC->AHB1ENR |= (1 << 8); }

/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN()		{ RCC->APB1ENR |= (1 << 21); }
#define I2C2_PCLK_EN()		{ RCC->APB1ENR |= (1 << 22); }
#define I2C3_PCLK_EN()		{ RCC->APB1ENR |= (1 << 23); }

/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN()		{ RCC->APB2ENR |= (1 << 12); }
#define SPI2_PCLK_EN()		{ RCC->APB1ENR |= (1 << 14); }
#define SPI3_PCLK_EN()		{ RCC->APB1ENR |= (1 << 15); }

/*
 * Clock Enable Macros for USARTx Peripherals
 */
#define USART2_PCLK_EN()	{ RCC->APB1ENR |= (1 << 17); }
#define USART3_PCLK_EN()	{ RCC->APB1ENR |= (1 << 18); }
#define UART4_PCLK_EN()		{ RCC->APB1ENR |= (1 << 19); }
#define UART5_PCLK_EN()		{ RCC->APB1ENR |= (1 << 20); }

/*
 * Clock Enable Macros for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_EN()	{ RCC->APB2ENR |= (1 << 14); }

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		{ RCC->AHB1ENR &= ~(1 << 0); }
#define GPIOB_PCLK_DI()		{ RCC->AHB1ENR &= ~(1 << 1); }
#define GPIOC_PCLK_DI()		{ RCC->AHB1ENR &= ~(1 << 2); }
#define GPIOD_PCLK_DI()		{ RCC->AHB1ENR &= ~(1 << 3); }
#define GPIOE_PCLK_DI()		{ RCC->AHB1ENR &= ~(1 << 4); }
#define GPIOF_PCLK_DI()		{ RCC->AHB1ENR &= ~(1 << 5); }
#define GPIOG_PCLK_DI()		{ RCC->AHB1ENR &= ~(1 << 6); }
#define GPIOH_PCLK_DI()		{ RCC->AHB1ENR &= ~(1 << 7); }
#define GPIOI_PCLK_DI()		{ RCC->AHB1ENR &= ~(1 << 8); }

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		{ RCC->APB1ENR &= ~(1 << 21); }
#define I2C2_PCLK_DI()		{ RCC->APB1ENR &= ~(1 << 22); }
#define I2C3_PCLK_DI()		{ RCC->APB1ENR &= ~(1 << 23); }

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		{ RCC->APB2ENR &= ~(1 << 12); }
#define SPI2_PCLK_DI()		{ RCC->APB1ENR &= ~(1 << 14); }
#define SPI3_PCLK_DI()		{ RCC->APB1ENR &= ~(1 << 15); }

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART2_PCLK_DI()	{ RCC->APB1ENR &= ~(1 << 17); }
#define USART3_PCLK_DI()	{ RCC->APB1ENR &= ~(1 << 18); }
#define UART4_PCLK_DI()		{ RCC->APB1ENR &= ~(1 << 19); }
#define UART5_PCLK_DI()		{ RCC->APB1ENR &= ~(1 << 20); }

/*
 * Clock Disable Macros for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_DI()	{ RCC->APB2ENR &= ~(1 << 14); }

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 * Return port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 : 0)

/*
 * IRQ (Interrupt Request) Number for STM32F429xx MCU
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

/*
 * Generic Macros
 */
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define	GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET


#include "stm32f429xx_gpio_driver.h"

#endif /* INC_STM32F429XX_H_ */
