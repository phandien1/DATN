/*
 * PMS7003.h
 *
 *  Created on: May 11, 2023
 *      Author: DIEN
 */

#ifndef PMS7003_PMS7003_H_
#define PMS7003_PMS7003_H_

#include "main.h"
#include "stdio.h"

#define CMD_read	    0xe2
#define CMD_change      0xe1
#define CMD_sleep       0xe4
#define DATAL_passive   0x00
#define DATAL_active    0x01


#define byte1	0x42;
#define byte2	0x4d;
#define byte4	0x01;

#define PM1p0   10
#define PM2p5   25
#define PM10    100

typedef struct{
	UART_HandleTypeDef* hUARTx;
	uint8_t MODE;
	uint8_t buffer[32];
}PMS7003;

HAL_StatusTypeDef PMS7003_transmit(PMS7003* _PMS7003);
HAL_StatusTypeDef PMS7003_start_receive(PMS7003* _PMS7003);
HAL_StatusTypeDef PMS7003_receive(PMS7003* _PMS7003);
void PMS7003_init_setmode (PMS7003* _PMS7003,  uint8_t mode);
void checksum(void);
PMS7003 newPMS7003(void);
uint16_t uint8_to_uint16(uint8_t msb, uint8_t lsb);
uint16_t calculator (PMS7003* _PMS7003, uint8_t parameter);

#endif /* PMS7003_PMS7003_H_ */
