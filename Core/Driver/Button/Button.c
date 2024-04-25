/*
 * Button.c
 *
 *  Created on: Apr 23, 2024
 *      Author: k1min
 */
#include "Button.h"


void button_init(Button_t *Button, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	Button->GPIOx = GPIOx;
	Button->GPIO_Pin = GPIO_Pin;
	Button->prevState = RELEASED;
}

button_state_t button_getState(Button_t *Button)
{
	// static int prevState = RELEASED;
	int curState = HAL_GPIO_ReadPin(Button->GPIOx, Button->GPIO_Pin);		// Read pin State

	if((Button->prevState == RELEASED) && (curState == PUSHED))
	{
		HAL_Delay(50);
		Button->prevState = PUSHED;			// PUSH == 0
		return ACT_PUSHED;							// return 3
	}
	else if((Button->prevState == PUSHED) && (curState == RELEASED))
	{
		HAL_Delay(50);
		Button->prevState = RELEASED;
		return ACT_RELEASED;
	}
	return NO_ACT;
}
