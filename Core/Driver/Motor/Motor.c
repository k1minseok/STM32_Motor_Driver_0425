/*
 * Motor.c
 *
 *  Created on: Apr 25, 2024
 *      Author: k1min
 */

#include "Motor.h"

void Motor_init(Motor_t *Motor,
			TIM_HandleTypeDef *htim,
			uint32_t Channel,
			GPIO_TypeDef *Dir1_GPIO, uint16_t Dir1_GPIO_Pin,
			GPIO_TypeDef *Dir2_GPIO, uint16_t Dir2_GPIO_Pin)
{
	Motor->htim = htim;
	Motor->Channel = Channel;
	Motor->Dir1_GPIO = Dir1_GPIO;
	Motor->Dir1_GPIO_Pin = Dir1_GPIO_Pin;
	Motor->Dir2_GPIO = Dir2_GPIO;
	Motor->Dir2_GPIO_Pin = Dir2_GPIO_Pin;

}
void Motor_Stop(Motor_t *Motor)
{
	HAL_TIM_PWM_Stop(Motor->htim, Motor->Channel);
}
void Motor_Forward(Motor_t *Motor)
{
	HAL_GPIO_WritePin(Motor->Dir1_GPIO, Motor->Dir1_GPIO_Pin, RESET);
	HAL_GPIO_WritePin(Motor->Dir2_GPIO, Motor->Dir2_GPIO_Pin, SET);
  HAL_TIM_PWM_Start(Motor->htim, Motor->Channel);
}
void Motor_Backward(Motor_t *Motor)
{
	HAL_GPIO_WritePin(Motor->Dir1_GPIO, Motor->Dir1_GPIO_Pin, SET);
	HAL_GPIO_WritePin(Motor->Dir2_GPIO, Motor->Dir2_GPIO_Pin, RESET);
  HAL_TIM_PWM_Start(Motor->htim, Motor->Channel);
}
void Motor_SetSpeed(Motor_t *Motor, int speedVal)
{
	__HAL_TIM_SET_COMPARE(Motor->htim, Motor->Channel, speedVal);
}
