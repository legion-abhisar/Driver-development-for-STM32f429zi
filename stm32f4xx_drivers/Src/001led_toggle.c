/*
 * 001led_toggle.c
 *
 *  Created on: Jun 23, 2024
 *      Author: abhisar
 */

#include "stm32f429xx.h"


void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t Gpioled;

	Gpioled.pGPIOx = GPIOB;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&Gpioled);

	while(1) {
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);
		delay();
	}

	return 0;
}
