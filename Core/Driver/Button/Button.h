/*
 * Button.h
 *
 *  Created on: Apr 23, 2024
 *      Author: k1min
 */

#ifndef DRIVER_BUTTON_BUTTON_H_
#define DRIVER_BUTTON_BUTTON_H_

#include "stm32f4xx_hal.h"

typedef enum {PUSHED, RELEASED, NO_ACT, ACT_PUSHED, ACT_RELEASED} button_state_t;

typedef struct{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	int prevState;
}Button_t;


void button_init(Button_t *Button, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
button_state_t button_getState(Button_t *Button);


#endif /* DRIVER_BUTTON_BUTTON_H_ */
