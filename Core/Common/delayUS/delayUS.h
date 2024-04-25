/*
 * delayUS.h
 *
 *  Created on: Apr 25, 2024
 *      Author: k1min
 */

#ifndef COMMON_DELAYUS_DELAYUS_H_
#define COMMON_DELAYUS_DELAYUS_H_

#include "stm32f4xx_hal.h"


void DelayInit(void);
void DelayUS(uint32_t us);

#endif /* COMMON_DELAYUS_DELAYUS_H_ */
