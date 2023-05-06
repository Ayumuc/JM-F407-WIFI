/*
 * rs485.h
 *
 *  Created on: Sep 30, 2021
 *      Author: Administrator
 */

#ifndef INC_RS485_H_
#define INC_RS485_H_
#include "main.h"
typedef struct LP20{
	uint8_t Data_Head;
	uint8_t Data_Key;
	uint8_t Data_Value[4];
	uint8_t Data_Crc;
	uint8_t Data_End;
	int Data_Value_Sum;
}L20;

L20 LP20_TOF;
extern int rs485_rec_flag;
extern char speed_value[4];
void LP20_Data_Handle();
void rs485DataHandler(void *pvParameters);
void setZeroTask(void *pvParameters);
uint8_t crc_high_first(uint8_t *ptr, uint8_t len);

#endif /* INC_RS485_H_ */
