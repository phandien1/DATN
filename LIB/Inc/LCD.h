/*
 * LCD.h
 *
 *  Created on: May 15, 2023
 *      Author: Administrator
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_
#include "main.h"
#include "string.h"
#include "74HC595.h"


#define EN_PIN      (_LCD->en_pin)
#define RS_PIN      (_LCD->rs_pin)
#define RW_PIN      (_LCD->rw_pin)
#define BL_PIN      (_LCD->bl_pin)
#define D4_PIN      (_LCD->d4_pin)
#define D5_PIN      (_LCD->d5_pin)
#define D6_PIN      (_LCD->d6_pin)
#define D7_PIN      (_LCD->d7_pin)

typedef enum{
	LCD_EN_PIN,
	LCD_RS_PIN,
	LCD_RW_PIN,
	LCD_BL_PIN,
	LCD_DB4,
	LCD_DB5,
	LCD_DB6,
	LCD_DB7
}LCD_pinName;

typedef struct LCD{
	uint8_t en_pin;
	uint8_t rs_pin;
	uint8_t rw_pin;
	uint8_t bl_pin;
	uint8_t d4_pin;
	uint8_t d5_pin;
	uint8_t d6_pin;
	uint8_t d7_pin;
}LCD;

void lcd_Init();
void lcd_Send_Data(char data);
void lcd_send_cmd (char cmd);
void LCD_AssignPin(LCD* LCD,uint8_t pin, LCD_pinName pinName);
void lcd_Send_String(char *str);
void lcd_Put_Cur(int row,int col);

#endif /* INC_LCD_H_ */
