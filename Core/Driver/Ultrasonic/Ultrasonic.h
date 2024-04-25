/*
 * Ultrasonic.h
 *
 *  Created on: Apr 24, 2024
 *      Author: k1min
 */

#ifndef DRIVER_ULTRASONIC_ULTRASONIC_H_
#define DRIVER_ULTRASONIC_ULTRASONIC_H_

#include "stm32f4xx_hal.h"

// Trig Port, Trig Pin, Echo Port, Echo Pin, TIMER, Flag
typedef struct{
	TIM_HandleTypeDef *TIM_handle;
	GPIO_TypeDef *GPIO_Trig_Port;
	uint16_t GPIO_Trig_Pin;
	GPIO_TypeDef *GPIO_Echo_Port;
	uint16_t GPIO_Echo_Pin;
	int echoFlag;

	uint16_t TIM_Counter;		// distance measurement counter
}Ultrasonic_t;


void Ultraosnic_init(
		Ultrasonic_t *Ultrasonic,
		TIM_HandleTypeDef *TIM_handle,
		GPIO_TypeDef *GPIO_Trig_Port,
		uint16_t GPIO_Trig_Pin,
		GPIO_TypeDef *GPIO_Echo_Port,
		uint16_t GPIO_Echo_Pin);

int Ultrasonic_getDistance(Ultrasonic_t *Ultrasonic);
void Ultrasonic_ISR_Process(Ultrasonic_t *Ultrasonic, uint16_t GPIO_Pin);

void Ultrasonic_clearTimer(Ultrasonic_t *Ultrasonic);
void Ultrasonic_startTimer(Ultrasonic_t *Ultrasonic);

void Ultrasonic_stopTimer(Ultrasonic_t *Ultrasonic);
uint16_t Ultrasonic_getTimerCounter(Ultrasonic_t *Ultrasonic);
int Ultrasonic_getEchopinState(Ultrasonic_t *Ultrasonic);
void Ultrasonic_startTrig(Ultrasonic_t *Ultrasonic);

int Ultrasonic_isCmpltRecvEcho(Ultrasonic_t *Ultrasonic);
void Ultrasonic_clearEchoFlag(Ultrasonic_t *Ultrasonic);
void Ultrasonic_setEchoFlag(Ultrasonic_t *Ultrasonic);



#endif /* DRIVER_ULTRASONIC_ULTRASONIC_H_ */
