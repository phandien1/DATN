/*
 * LCD.c
 *
 *  Created on: May 15, 2023
 *      Author: Administrator
 */
#include "LCD.h"

LCD *_LCD=NULL;

void LCD_AssignPin(LCD* LCD,uint8_t pin, LCD_pinName pinName)
{
	switch(pinName){
	case LCD_EN_PIN:
		LCD->en_pin = pin;
		break;
	case LCD_RS_PIN:
		LCD->rs_pin = pin;
		break;
	case LCD_RW_PIN:
		LCD->rw_pin = pin;
		break;
	case LCD_BL_PIN:
		LCD->bl_pin = pin;
		break;
	case LCD_DB4:
		LCD->d4_pin = pin;
		break;
	case LCD_DB5:
		LCD->d5_pin = pin;
		break;
	case LCD_DB6:
		LCD->d6_pin = pin;
		break;
	case LCD_DB7:
		LCD->d7_pin = pin;
		break;
	}
	_LCD = LCD;
}

void lcd_Init()
{
	// Khởi tạo 4 bit
	// 4 bit initialisation
	// Khởi tạo 4 bit
		HAL_Delay(50);
		lcd_send_cmd(0x30);
		HAL_Delay(5);
		lcd_send_cmd(0x30);
		HAL_Delay(1);
		lcd_send_cmd(0x30);
		HAL_Delay(10);
		lcd_send_cmd(0x20);
		HAL_Delay(10);
		// Khởi tạo hiển thị
		//00 00 101 00
		lcd_send_cmd(0x28); //function set bảng 6/tr23: set data interface 4bits 2lines 5x8 bit
		HAL_Delay(1);
		lcd_send_cmd(0x08); //display on/off control
		HAL_Delay(1);
		lcd_send_cmd(0x01); //clear display
		HAL_Delay(1);
		HAL_Delay(1);
		lcd_send_cmd(0x06); //entry mode set: i/d=1 increment s=0
		HAL_Delay(1);
		lcd_send_cmd(0x0C); // cho phep hien thi man hinh
}

void lcd_send_cmd (char cmd)
{
	char data_u, data_l; // vi du 0x30
	uint8_t data_t[4];
	data_u = (cmd >> 4) & 0x0f; // data_u =0x03
	data_l = (cmd & 0x0f); // data_l = 0x00

	data_t[0] = (data_u & 0x01) << D4_PIN | (data_u & 0x02 ) << (D5_PIN-1)  | (data_u & 0x04) << (D6_PIN-2)  | (data_u & 0x08) << (D7_PIN-3) | (1<<EN_PIN)| (0<<RS_PIN)  | (1<< BL_PIN);
	data_t[1] = (data_u & 0x01) << D4_PIN | (data_u & 0x02 ) << (D5_PIN-1)  | (data_u & 0x04) << (D6_PIN-2)  | (data_u & 0x08) << (D7_PIN-3) | (0<<EN_PIN)| (0<<RS_PIN)  | (1<< BL_PIN);
	data_t[2] = (data_l & 0x01) << D4_PIN | (data_l & 0x02 ) << (D5_PIN-1)  | (data_l & 0x04) << (D6_PIN-2)  | (data_l & 0x08) << (D7_PIN-3) | (1<<EN_PIN)| (0<<RS_PIN)  | (1<< BL_PIN);
	data_t[3] = (data_l & 0x01) << D4_PIN | (data_l & 0x02 ) << (D5_PIN-1)  | (data_l & 0x04) << (D6_PIN-2)  | (data_l & 0x08) << (D7_PIN-3) | (0<<EN_PIN)| (0<<RS_PIN)  | (1<< BL_PIN);

	for(int i = 0;i<4;i++)
	{
		HC595_ShiftOut(data_t+i,1,0);
	}
}

void lcd_Send_Data(char data)
{
	char data_u,data_l;
	uint8_t data_t[4];
	data_u = (data >> 4) & 0x0f;
	data_l = (data & 0x0f);
	data_t[0] = (data_u & 0x01) << D4_PIN | (data_u & 0x02 ) << (D5_PIN-1)  | (data_u & 0x04) << (D6_PIN-2)  | (data_u & 0x08) << (D7_PIN-3) | (1<<EN_PIN)| (1<<RS_PIN)  | (1<< BL_PIN);
	data_t[1] = (data_u & 0x01) << D4_PIN | (data_u & 0x02 ) << (D5_PIN-1)  | (data_u & 0x04) << (D6_PIN-2)  | (data_u & 0x08) << (D7_PIN-3) | (0<<EN_PIN)| (1<<RS_PIN)  | (1<< BL_PIN);
	data_t[2] = (data_l & 0x01) << D4_PIN | (data_l & 0x02 ) << (D5_PIN-1)  | (data_l & 0x04) << (D6_PIN-2)  | (data_l & 0x08) << (D7_PIN-3) | (1<<EN_PIN)| (1<<RS_PIN)  | (1<< BL_PIN);
	data_t[3] = (data_l & 0x01) << D4_PIN | (data_l & 0x02 ) << (D5_PIN-1)  | (data_l & 0x04) << (D6_PIN-2)  | (data_l & 0x08) << (D7_PIN-3) | (0<<EN_PIN)| (1<<RS_PIN)  | (1<< BL_PIN);

	for(int i = 0;i<4;i++)
	{
		HC595_ShiftOut(data_t+i,1,0);
	}
	//HAL_I2C_Master_Transmit(&hi2c1, 0x27 << 1,(uint8_t *)data_t, 4, 100);
}

void lcd_Send_String(char *str)
{
	while(*str) lcd_Send_Data(*str++);
}

void lcd_Put_Cur(int row,int col)
{
	switch(row)
	{
		case 0:
			col |= 0x80; //0x80 1 000 000x
			break;
		case 1:
			col |= 0xC0; // 0xC1 1 100 000x
			break;
	}
	//Gửi lệnh ứng với vị trí cần xuất hiện con trỏ Set DDRAM address tr24
	lcd_send_cmd(col);
}
