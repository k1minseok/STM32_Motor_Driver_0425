/*
 * delayUS.c
 *
 *  Created on: Apr 25, 2024
 *      Author: k1min
 */

#include "delayUS.h"


void DelayInit(void)
{
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

	DWT->CYCCNT = 0;

	/* 3 NO OPERATION instructions */
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
}
// us delay
void DelayUS(uint32_t us)
{
	uint32_t cycles = (SystemCoreClock / 1000000L) * us;
	uint32_t start = DWT->CYCCNT;
	volatile uint32_t cnt;

	do
	{
		cnt = DWT->CYCCNT - start;
	} while (cnt < cycles);
}
