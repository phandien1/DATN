/*
 * PMS7003.c
 *
 *  Created on: Mar 17, 2023
 *      Author: Dien
 */
#include "PMS7003.h"

static	uint8_t txbuffer[7];

PMS7003 newPMS7003(void)
{
	PMS7003 new_PMS7003;
	new_PMS7003.MODE = DATAL_passive;
	return new_PMS7003;
}

void checksum(void){
	uint16_t sum = 0;
	for (int i = 0 ; i<5 ; i++){
		sum = sum + txbuffer[i];
	}
	txbuffer[6] = sum;
	txbuffer[5] = sum >> 8;
}

void PMS7003_init_setmode (PMS7003* _PMS7003,  uint8_t mode)
{

		txbuffer[0] = byte1;
		txbuffer[1] = byte2;
		txbuffer[3] = byte4;

		if (mode == DATAL_passive){
			txbuffer[2] = CMD_change;
			txbuffer[4] = DATAL_passive;
			checksum();
			HAL_UART_Transmit(_PMS7003->hUARTx, txbuffer, 7, 100);
		}
		if (mode == DATAL_active){
			txbuffer[2] = CMD_change;
			txbuffer[4] = DATAL_active;
			checksum();
			HAL_UART_Transmit(_PMS7003->hUARTx, txbuffer, 7, 100);
		}
}

HAL_StatusTypeDef PMS7003_transmit(PMS7003* _PMS7003){
	txbuffer[2] = CMD_read;
	checksum();
	return HAL_UART_Transmit(_PMS7003->hUARTx, txbuffer, 7, 100);
}

HAL_StatusTypeDef PMS7003_start_receive(PMS7003* _PMS7003){
	return HAL_UART_Receive_IT(_PMS7003->hUARTx, _PMS7003->buffer, 32);
}

HAL_StatusTypeDef PMS7003_receive(PMS7003* _PMS7003){
	return HAL_UART_Receive_IT(_PMS7003->hUARTx, _PMS7003->buffer, 32); // bật cờ ngắt
}

uint16_t uint8_to_uint16(uint8_t msb, uint8_t lsb)
{
	return (uint16_t)((uint16_t)msb << 8u) | lsb;
}

uint16_t calculator (PMS7003* _PMS7003, uint8_t parameter){
	switch (parameter) {
		case PM1p0:
			return uint8_to_uint16(_PMS7003->buffer[4], _PMS7003->buffer[5]);
			break;
		case PM2p5:
			return uint8_to_uint16(_PMS7003->buffer[6], _PMS7003->buffer[7]);
			break;
		case PM10:
			return uint8_to_uint16(_PMS7003->buffer[8], _PMS7003->buffer[9]);
			break;
		}
	return 0;
}
