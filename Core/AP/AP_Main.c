/*
 * AP_Main.c
 *
 *  Created on: Apr 23, 2024
 *      Author: k1min
 */

#include "AP_Main.h"
#include "stdio.h"

extern TIM_HandleTypeDef htim1, htim2, htim3;

Button_t Button_1, Button_2, Button_3;
LED_t LED_1, LED_2, LED_3;
Ultrasonic_t Ultrasonic_1, Ultrasonic_2, Ultrasonic_3;
Motor_t hLeftMotor, hRightMotor;

uint8_t mode_State;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	Ultrasonic_ISR_Process(&Ultrasonic_1, GPIO_Pin);
	Ultrasonic_ISR_Process(&Ultrasonic_2, GPIO_Pin);
	Ultrasonic_ISR_Process(&Ultrasonic_3, GPIO_Pin);
}


void sys_init()
{
	button_init(&Button_1, GPIOC, GPIO_PIN_10);
	button_init(&Button_2, GPIOC, GPIO_PIN_11);
	button_init(&Button_3, GPIOC, GPIO_PIN_12);

	LED_init(&LED_1, GPIOC, GPIO_PIN_8);
	LED_init(&LED_2, GPIOC, GPIO_PIN_6);
	LED_init(&LED_3, GPIOC, GPIO_PIN_5);

	Ultraosnic_init(&Ultrasonic_1, &htim1, Ultrasonic1_Trig_Port, Ultrasonic1_Trig_Pin,
			Ultrasonic1_Echo_Port, Ultrasonic1_Echo_Pin);
	Ultraosnic_init(&Ultrasonic_2, &htim2, Ultrasonic2_Trig_Port, Ultrasonic2_Trig_Pin,
			Ultrasonic2_Echo_Port, Ultrasonic2_Echo_Pin);
	Ultraosnic_init(&Ultrasonic_3, &htim3, Ultrasonic3_Trig_Port, Ultrasonic3_Trig_Pin,
			Ultrasonic3_Echo_Port, Ultrasonic3_Echo_Pin);

	Motor_init(&hLeftMotor, &htim3, TIM_CHANNEL_1, LEFT_DIR1_Port,
			LEFT_DIR1_Port_Pin, LEFT_DIR2_Port, LEFT_DIR2_Port_Pin);
	Motor_init(&hRightMotor, &htim3, TIM_CHANNEL_2, RIGHT_DIR1_Port,
			RIGHT_DIR1_Port_Pin, RIGHT_DIR2_Port, RIGHT_DIR2_Port_Pin);

	LCD_init();
	DelayInit();

	mode_State = mode_PWMStop;
}

int APMain()
{
	sys_init();

	LCD_writeStringXY(0, 0, "Hello STM32!");
	LCD_writeStringXY(1, 0, "KCCISTC Harman");

	char buff[50];
	int distance;


	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	Motor_SetSpeed(&hLeftMotor, 850);
	Motor_SetSpeed(&hRightMotor, 850);

	while (1)
	{
		Motor_Backward(&hLeftMotor);
		Motor_Backward(&hRightMotor);
		HAL_Delay(2000);
		Motor_Forward(&hLeftMotor);
		Motor_Forward(&hRightMotor);
		HAL_Delay(2000);

//		modeStateCheck();
//		modeStateRun();

		/*
		if (button_getState(&Button_1) == ACT_RELEASED)
		{


			distance = Ultrasonic_getDistance(&Ultrasonic_1);
			if(distance > 150) distance = 150;
			sprintf(buff, "dis:%dcm       ", distance);
			LCD_writeStringXY(0, 0, buff);
		}
		if (button_getState(&Button_2) == ACT_RELEASED)
		{


			distance = Ultrasonic_getDistance(&Ultrasonic_2);
			if(distance > 150) distance = 150;
			sprintf(buff, "dis:%dcm       ", distance);
			LCD_writeStringXY(1, 0, buff);
		}
		if (button_getState(&Button_3) == ACT_RELEASED)
		{


			distance = Ultrasonic_getDistance(&Ultrasonic_3);
			if(distance > 150) distance = 150;
			sprintf(buff, "%dcm    ", distance);
			LCD_writeStringXY(0, 0, buff);
		}
		*/
	}

	return 0;
}



void modeStateCheck()
{
	if (button_getState(&Button_1) == ACT_PUSHED)
	{
		mode_State = mode_LEDincrease;
		LED_OFF(&LED_1);
		LED_OFF(&LED_2);
		LED_OFF(&LED_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	}
	if (button_getState(&Button_2) == ACT_PUSHED)
	{
		mode_State = mode_LEDdecrease;
		LED_OFF(&LED_1);
		LED_OFF(&LED_2);
		LED_OFF(&LED_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	}
	if (button_getState(&Button_3) == ACT_PUSHED)
	{
		mode_State = mode_PWMStop;
		LED_OFF(&LED_1);
		LED_OFF(&LED_2);
		LED_OFF(&LED_3);
	}
}
void modeStateRun()
{
	static uint32_t prevTime = 0;
	uint32_t curTime = HAL_GetTick();
	static uint16_t i = 0;

	switch (mode_State)
	{
		case mode_LEDincrease:
			if(curTime - prevTime < 10)
				return;
			prevTime = curTime;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, i);
			i += 10;
			if (i >= 999)
				i = 0;
			break;

		case mode_LEDdecrease:
			if(curTime - prevTime < 10)
				return;
			prevTime = curTime;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, i);
			i -= 10;
			if (i <= 0)
				i = 999;
			break;

		case mode_PWMStop:
//			if(curTime - prevTime < 1000)
//				return;
//			prevTime = curTime;
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			break;
	}
}
