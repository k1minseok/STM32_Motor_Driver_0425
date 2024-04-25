/*
 * Ultrasonic.c
 *
 *  Created on: Apr 24, 2024
 *      Author: k1min
 */

#include "Ultrasonic.h"
#include "../Common/delayUS/delayUS.h"


void Ultraosnic_init(
		Ultrasonic_t *Ultrasonic,
		TIM_HandleTypeDef *TIM_handle,
		GPIO_TypeDef *GPIO_Trig_Port,
		uint16_t GPIO_Trig_Pin,
		GPIO_TypeDef *GPIO_Echo_Port,
		uint16_t GPIO_Echo_Pin)
{
	Ultrasonic->TIM_handle = TIM_handle;
	Ultrasonic->GPIO_Trig_Port = GPIO_Trig_Port;
	Ultrasonic->GPIO_Trig_Pin = GPIO_Trig_Pin;
	Ultrasonic->GPIO_Echo_Port = GPIO_Echo_Port;
	Ultrasonic->GPIO_Echo_Pin = GPIO_Echo_Pin;
	Ultrasonic->echoFlag = 0;
	Ultrasonic->TIM_Counter = 0;
}


int Ultrasonic_getDistance(Ultrasonic_t *Ultrasonic)
{
	int timeout = 0;

	Ultrasonic_startTrig(Ultrasonic);
	while (!Ultrasonic_isCmpltRecvEcho(Ultrasonic))		// Falling edge까지 대기
	{
		timeout++;
		if (timeout > 20)			// 타임아웃 처리
			return 0;
		HAL_Delay(1);
	}
	Ultrasonic_clearEchoFlag(Ultrasonic);

	return Ultrasonic->TIM_Counter * 0.034 / 2;		 //cm distance
}


void Ultrasonic_ISR_Process(Ultrasonic_t *Ultrasonic, uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Ultrasonic->GPIO_Echo_Pin)
	{
		// Echo pin High 유지시간 측정
		if (Ultrasonic_getEchopinState(Ultrasonic))		// Rising edge
		{
			//TIM1->CNT
			Ultrasonic_clearTimer(Ultrasonic);					// TIM1 동작 전 CNT 초기화
			Ultrasonic_startTimer(Ultrasonic);					// Rising edge이면 TIM1 동작 시작
			Ultrasonic_clearEchoFlag(Ultrasonic);
		}
		else		// Falling edge
		{
			Ultrasonic_stopTimer(Ultrasonic);			// Falling edge이면 TIM1 동작 종료
			Ultrasonic->TIM_Counter = Ultrasonic_getTimerCounter(Ultrasonic);		// TIM1 CNT 값 읽기
			Ultrasonic_setEchoFlag(Ultrasonic);
		}
	}
}



void Ultrasonic_clearTimer(Ultrasonic_t *Ultrasonic)
{
	__HAL_TIM_SET_COUNTER(Ultrasonic->TIM_handle, 0);
}

void Ultrasonic_startTimer(Ultrasonic_t *Ultrasonic)
{
	HAL_TIM_Base_Start(Ultrasonic->TIM_handle);
}

void Ultrasonic_stopTimer(Ultrasonic_t *Ultrasonic)
{
	HAL_TIM_Base_Stop(Ultrasonic->TIM_handle);
}

uint16_t Ultrasonic_getTimerCounter(Ultrasonic_t *Ultrasonic)
{
	return __HAL_TIM_GET_COUNTER(Ultrasonic->TIM_handle);
}

int Ultrasonic_getEchopinState(Ultrasonic_t *Ultrasonic)
{
	return HAL_GPIO_ReadPin(Ultrasonic->GPIO_Echo_Port, Ultrasonic->GPIO_Echo_Pin);
}

void Ultrasonic_startTrig(Ultrasonic_t *Ultrasonic)
{
	HAL_GPIO_WritePin(Ultrasonic->GPIO_Trig_Port, Ultrasonic->GPIO_Trig_Pin, SET);
	//HAL_Delay(1);
	DelayUS(15);
	HAL_GPIO_WritePin(Ultrasonic->GPIO_Trig_Port, Ultrasonic->GPIO_Trig_Pin, RESET);
}



int Ultrasonic_isCmpltRecvEcho(Ultrasonic_t *Ultrasonic)
{
	return Ultrasonic->echoFlag;
}
void Ultrasonic_clearEchoFlag(Ultrasonic_t *Ultrasonic)
{
	Ultrasonic->echoFlag = 0;
}
void Ultrasonic_setEchoFlag(Ultrasonic_t *Ultrasonic)
{
	Ultrasonic->echoFlag = 1;
}
