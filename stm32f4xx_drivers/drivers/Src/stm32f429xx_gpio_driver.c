/*
 * stm32f429xx_gpio_driver.c
 *
 *  Created on: Jun 23, 2024
 *      Author: abhisar
 */

#include "stm32f429xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */
/***************************************************************
 * Function name		- GPIO_PeriClockControl
 *
 * Brief				- This function enables or disable peripheral clock for
 * 						the given GPIO port.
 *
 * Param				- Base address of the GPIO peripheral
 * Param				- ENABLE or DISABLE macros
 * Param				-
 *
 * Return				- None
 *
 * Note					- None
 *
 **************************************************************/
void GPIO_PeriClockControl(GPIO_Regdef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
/***************************************************************
 * Function name		- GPIO_Init
 *
 * Brief				- This function initializes a GPIO
 *
 * Param				- Handle pointer to the GPIO peripheral
 * Param				-
 * Param				-
 *
 * Return				- None
 *
 * Note					- None
 *
 **************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// 1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// Non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing
		pGPIOHandle->pGPIOx->MODER |= temp; // Setting
		temp = 0;
	} else {
		// Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// 1. Configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// 1. Configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// Clear corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// 1. Configure both FTSR & RTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);


		// 3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}

	temp = 0;
	// 2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;	// Setting

	temp = 0;

	// 3. Configure the pupd settings.
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;		// Setting

	temp = 0;

	// 4. Configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;	// Setting

	temp = 0;

	// 5. Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		// Configure the alt function registers.
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));  // Clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // Setting
	}

}

/***************************************************************
 * Function name		- GPIO_DeInit
 *
 * Brief				- This function deinitilizes the GPIO
 *
 * Param				- Base address of the GPIO peripheral
 * Param				-
 * Param				-
 *
 * Return				- None
 *
 * Note					- None
 *
 **************************************************************/
void GPIO_DeInit(GPIO_Regdef_t *pGPIOx)
{
	if(pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */
/***************************************************************
 * Function name		- GPIO_ReadFromInputPin
 *
 * Brief				- This function reads input from an input GPIO pin
 *
 * Param				- Base address of the GPIO peripheral
 * Param				- Pin number
 * Param				-
 *
 * Return				- 0 or 1
 *
 * Note					- None
 *
 **************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 );
	return value;
}

/***************************************************************
 * Function name		- GPIO_ReadFromInputPort
 *
 * Brief				- This function reads from an input GPIO port.
 *
 * Param				- Base address of the GPIO peripheral
 * Param				-
 * Param				-
 *
 * Return				- uint16_t
 *
 * Note					- None
 *
 **************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);

	return value;
}

/***************************************************************
 * Function name		- GPIO_WriteToOutputPin
 *
 * Brief				- This function writes to output pin of GPIO port.
 *
 * Param				- Base address of the GPIO peripheral
 * Param				- Pin number
 * Param				- Value to write
 *
 * Return				- None
 *
 * Note					- None
 *
 **************************************************************/
void GPIO_WriteToOutputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET) {
		// Write 1 to the output data register at bit field corresponding pin number
		pGPIOx->ODR |= ( 1 << PinNumber );
	} else {
		// Write 0 to the output data register at bit field corresponding pin number
		pGPIOx->ODR &= ~( 1 << PinNumber );
	}
}

/***************************************************************
 * Function name		- GPIO_WriteToOutputPort
 *
 * Brief				- This function writes to output GPIO port.
 *
 * Param				- Base address of the GPIO peripheral
 * Param				- Value to write
 * Param				-
 *
 * Return				- None
 *
 * Note					- None
 *
 **************************************************************/
void GPIO_WriteToOutputPort(GPIO_Regdef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/***************************************************************
 * Function name		- GPIO_ToggleOutputPin
 *
 * Brief				- This function enables or disable peripheral clock for
 * 						the given GPIO port.
 *
 * Param				- Base address of the GPIO peripheral
 * Param				- ENABLE or DISABLE macros
 * Param				-
 *
 * Return				- None
 *
 * Note					- None
 *
 **************************************************************/
void GPIO_ToggleOutputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */
/***************************************************************
 * Function name		- GPIO_IRQInterruptConfig
 *
 * Brief				- This function enables or disable peripheral clock for
 * 						the given GPIO port.
 *
 * Param				- Base address of the GPIO peripheral
 * Param				- ENABLE or DISABLE macros
 * Param				-
 *
 * Return				- None
 *
 * Note					- None
 *
 **************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31) {
			// Program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// Program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );

		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// Program ISER2 register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );

		}
	} else {
		if(IRQNumber <= 31) {
			// Program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// Program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );

		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// Program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 32) );

		}
	}
}

/***************************************************************
 * Function name		- GPIO_IRQPriorityConfig
 *
 * Brief				- This function sets the priority of the interrupt
 *
 * Param				- IRQ number
 * Param				- IRQ priority number
 * Param				-
 *
 * Return				- None
 *
 * Note					- None
 *
 **************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// 1. First find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4))|= (IRQPriority << shift_amount );
}

/***************************************************************
 * Function name		- GPIO_IRQHandling
 *
 * Brief				- This function enables or disable peripheral clock for
 * 						the given GPIO port.
 *
 * Param				-
 * Param				-
 * Param				-
 *
 * Return				- None
 *
 * Note					- None
 *
 **************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear EXTI PR register corresponding to pin number
	if (EXTI->PR & ( 1 << PinNumber ))
	{
		// Clear
		EXTI->PR |= ( 1 << PinNumber );
	}
}















