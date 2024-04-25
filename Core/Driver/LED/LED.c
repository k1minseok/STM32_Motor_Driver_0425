/*
 * LED.c
 *
 *  Created on: Apr 23, 2024
 *      Author: k1min
 */

#include "LED.h"


void LED_init(LED_t *LED, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	LED->GPIOx = GPIOx;
	LED->GPIO_Pin = GPIO_Pin;
}

void LED_ON(LED_t *LED)
{
	HAL_GPIO_WritePin(LED->GPIOx, LED->GPIO_Pin, SET);
}

void LED_OFF(LED_t *LED)
{
	HAL_GPIO_WritePin(LED->GPIOx, LED->GPIO_Pin, RESET);
}

void LED_toggle(LED_t *LED)
{
	HAL_GPIO_TogglePin(LED->GPIOx, LED->GPIO_Pin);
}
