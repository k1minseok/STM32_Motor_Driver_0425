/*
 * CLCD.c
 *
 *  Created on: Apr 23, 2024
 *      Author: k1min
 */

#include "CLCD.h"

extern I2C_HandleTypeDef hi2c1;
uint8_t lcdData = 0;


void LCD_init()		// RESET FUNCTION
{
	LCD_delay(15);
	LCD_cmdMode();
	LCD_writeMode();
	LCD_sendHighNibble(0x30);
	LCD_delay(5);
	LCD_sendHighNibble(0x30);
	LCD_delay(1);
	LCD_sendHighNibble(0x30);
	LCD_sendHighNibble(0x20);
	LCD_sendByte(LCD_4BIT_FUNCTION_SET);		// Function set 4bit interface, 2line, 5x8font
	LCD_sendByte(LCD_DISPLAY_OFF);					// display OFF
	LCD_sendByte(LCD_DISPLAY_CLEAR);				// display clear
	LCD_sendByte(LCD_ENTRY_MODE_SET);				// entry mode set
	LCD_sendByte(LCD_DISPLAY_ON);						// display ON
	LCD_backLightON();											// backlight ON
}
void LCD_delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}
void LCD_backLightON()
{
	lcdData |= (1 << LCD_BACKLIGHT);
	LCD_sendData(lcdData);
}


/*
 * LCD_writeCmdData(uint8_t data), LCD_writeCharData(uint8_t data)
 * => RS, RW bit setup하고 1byte 전송
 *
 * LCD_cmdMode(), LCD_charMode()
 * => lcdData 변수의 RS 비트만 바꿔서 저장 (전송 X)
 *
 * LCD_writeMode()
 * => lcdData 변수의 RW 비트만 바꿔서 저장 (전송 X)
 *
 * LCD_sendByte(uint8_t data)
 * => 전송하고 싶은 data를 상위 4bit, 하위 4bit 나누어 전송
 *
 * LCD_sendData(uint8_t data)
 * => 전송하고 싶은 data 1byte 전송(HighNibble, LowNibble 사용)
 *
 * LCD_sendHighNibble(uint8_t data), LCD_sendLowNibble(uint8_t data)
 * => lcdData 변수의 상위 4bit를 data로 저장하고 하위 4bit를 control bit(RS,RW,E)로 저장하고 전송
 *
 * LCD_E_High(), LCD_E_Low()
 * => lcdData 변수의 E 비트만 바꿔서 저장 (전송 X)
 *
 */

void LCD_writeCmdData(uint8_t data)
{
	LCD_cmdMode();
	LCD_writeMode();
	LCD_sendByte(data);
}
void LCD_writeCharData(uint8_t data)
{
	LCD_charMode();
	LCD_writeMode();
	LCD_sendByte(data);
}
void LCD_writeString(char *str)
{
	for (int i = 0; str[i]; i++)
	{
		LCD_writeCharData(str[i]);
	}
}


void LCD_gotoXY(uint8_t row, uint8_t col)
{
	col %= 16;
	row %= 2;

	uint8_t lcdRegAddr = (0x40 * row) + col;
	uint8_t command = 0x80 + lcdRegAddr;
	LCD_writeCmdData(command);
}
void LCD_writeStringXY(uint8_t row, uint8_t col, char *str)
{
	LCD_gotoXY(row, col);
	LCD_writeString(str);
}


void LCD_cmdMode()
{
	lcdData &= ~(1<<LCD_RS);			// RS bit set to 0 -> Instruction Register
}
void LCD_charMode()
{
	lcdData |= (1<<LCD_RS);			// RS bit set to 1 -> Data Register
}

void LCD_writeMode()
{
	lcdData &= ~(1<<LCD_RW);			// RW bit set to 0 -> write mode
}


void LCD_sendByte(uint8_t data)			// Send twice upper 4 bits & lower 4 bits
{
	LCD_sendHighNibble(data);				// 상위 4bit 전송
	LCD_sendLowNibble(data);				// 하위 4bit 전송
}
void LCD_sendData(uint8_t data)
{
	HAL_I2C_Master_Transmit(&hi2c1, lcdDevAddrRW, &data, 1, 1000);
	// I2C 통신으로 lcdDevAddrRW 주소 1byte 전송 + data 1Byte 전송
}


void LCD_sendHighNibble(uint8_t data)
{
	LCD_E_High();
	lcdData = (lcdData & 0x0f) | (data & 0xf0);	// 매개변수 data의 상위 4bit를 lcData의 상위 4bit로
	LCD_sendData(lcdData);			// MCU send data to LCD (Data Setup)

	HAL_Delay(1);			// for tr(pin E rise time)

	LCD_E_Low();
	LCD_sendData(lcdData);			// MCU send data to LCD (Data Hold)

	HAL_Delay(1);			// for tf(pin E fall time) (Data Hold)
}
void LCD_sendLowNibble(uint8_t data)
{
	LCD_E_High();
	lcdData = (lcdData & 0x0f) | ((data & 0x0f) << 4);	// 매개변수 data의 하위 4bit lcData의 상위 4bit로
	LCD_sendData(lcdData);			// MCU send data to LCD (Data Setup)

	HAL_Delay(1);			// for tr(pin E rise time)

	LCD_E_Low();
	LCD_sendData(lcdData);			// MCU send data to LCD (Data Hold)

	HAL_Delay(1);			// for tf(pin E fall time)
}


void LCD_E_High()
{
	lcdData |= (1<<LCD_E);
}
void LCD_E_Low()
{
	lcdData &= ~(1<<LCD_E);
}














