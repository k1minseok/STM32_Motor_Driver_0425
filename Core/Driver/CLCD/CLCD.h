/*
 * CLCD.h
 *
 *  Created on: Apr 23, 2024
 *      Author: k1min
 */

#ifndef DRIVER_CLCD_CLCD_H_
#define DRIVER_CLCD_CLCD_H_

#include "stm32f4xx_hal.h"


#define LCD_RS				0
#define LCD_RW				1
#define LCD_E					2
#define LCD_BACKLIGHT							3
#define lcdDevAddr								0x27
#define lcdDevAddrRW							(0x27 << 1)
#define LCD_4BIT_FUNCTION_SET			0x28
#define LCD_DISPLAY_OFF						0x08
#define LCD_DISPLAY_ON						0x0C
#define LCD_DISPLAY_CLEAR					0x01
#define LCD_ENTRY_MODE_SET				0X06


void LCD_init();
void LCD_delay(uint32_t Delay);
void LCD_backLightON();

void LCD_writeCmdData(uint8_t data);
void LCD_writeCharData(uint8_t data);
void LCD_writeString(char *str);

void LCD_gotoXY(uint8_t row, uint8_t col);
void LCD_writeStringXY(uint8_t row, uint8_t col, char *str);

void LCD_cmdMode();
void LCD_charMode();
void LCD_writeMode();

void LCD_sendByte(uint8_t data);
void LCD_sendData(uint8_t data);

void LCD_sendHighNibble(uint8_t data);
void LCD_sendLowNibble(uint8_t data);

void LCD_E_High();
void LCD_E_Low();


#endif /* DRIVER_CLCD_CLCD_H_ */
