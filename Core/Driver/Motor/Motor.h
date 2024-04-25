/*
 * Motor.h
 *
 *  Created on: Apr 25, 2024
 *      Author: k1min
 */

#ifndef DRIVER_MOTOR_MOTOR_H_
#define DRIVER_MOTOR_MOTOR_H_

#include "stm32f4xx_hal.h"


typedef struct{
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
	GPIO_TypeDef *Dir1_GPIO;
	uint16_t Dir1_GPIO_Pin;
	GPIO_TypeDef *Dir2_GPIO;
	uint16_t Dir2_GPIO_Pin;
}Motor_t;


void Motor_init(Motor_t *Motor,
			TIM_HandleTypeDef *htim,
			uint32_t Channel,
			GPIO_TypeDef *Dir1_GPIO, uint16_t Dir1_GPIO_Pin,
			GPIO_TypeDef *Dir2_GPIO, uint16_t Dir2_GPIO_Pin);
void Motor_Stop(Motor_t *Motor);
void Motor_Forward(Motor_t *Motor);
void Motor_Backward(Motor_t *Motor);
void Motor_SetSpeed(Motor_t *Motor, int speedVal);


#endif /* DRIVER_MOTOR_MOTOR_H_ */
