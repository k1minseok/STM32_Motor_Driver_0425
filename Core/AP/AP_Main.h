/*
 * AP_Main.h
 *
 *  Created on: Apr 23, 2024
 *      Author: k1min
 */

#ifndef AP_AP_MAIN_H_
#define AP_AP_MAIN_H_

#include "stm32f4xx_hal.h"

#include "../Driver/Button/Button.h"
#include "../Driver/LED/LED.h"
#include "../Driver/CLCD/CLCD.h"
#include "../Driver/Ultrasonic/Ultrasonic.h"
#include "../Driver/Motor/Motor.h"


#define Ultrasonic1_Trig_Port			GPIOC
#define Ultrasonic1_Trig_Pin			GPIO_PIN_0
#define Ultrasonic1_Echo_Port			GPIOC
#define Ultrasonic1_Echo_Pin			GPIO_PIN_1

#define Ultrasonic2_Trig_Port			GPIOB
#define Ultrasonic2_Trig_Pin			GPIO_PIN_0
#define Ultrasonic2_Echo_Port			GPIOA
#define Ultrasonic2_Echo_Pin			GPIO_PIN_4

#define Ultrasonic3_Trig_Port			GPIOC
#define Ultrasonic3_Trig_Pin			GPIO_PIN_3
#define Ultrasonic3_Echo_Port			GPIOC
#define Ultrasonic3_Echo_Pin			GPIO_PIN_2

#define LEFT_DIR1_Port						GPIOC
#define LEFT_DIR1_Port_Pin				GPIO_PIN_7
#define LEFT_DIR2_Port						GPIOA
#define LEFT_DIR2_Port_Pin				GPIO_PIN_9

#define RIGHT_DIR1_Port						GPIOA
#define RIGHT_DIR1_Port_Pin				GPIO_PIN_8
#define RIGHT_DIR2_Port						GPIOB
#define RIGHT_DIR2_Port_Pin				GPIO_PIN_10



typedef enum{
	mode_LEDincrease,
	mode_LEDdecrease,
	mode_PWMStop,
}mode_t;


void sys_init();
int APMain();

void modeStateCheck();
void modeStateRun();

#endif /* AP_AP_MAIN_H_ */
