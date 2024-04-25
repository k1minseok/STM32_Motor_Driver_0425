/*
 * LED.h
 *
 *  Created on: Apr 23, 2024
 *      Author: k1min
 */

#ifndef DRIVER_LED_LED_H_
#define DRIVER_LED_LED_H_

#include "stm32f4xx_hal.h"

typedef struct{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
}LED_t;


void LED_init(LED_t *LED, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void LED_ON(LED_t *LED);
void LED_OFF(LED_t *LED);
void LED_toggle(LED_t *LED);


#endif /* DRIVER_LED_LED_H_ */
