/*
 * display.h
 *
 *  Created on: Sep 23, 2021
 *      Author: Administrator
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_


#include "MachineStatusManager.h"

typedef enum
{
	IDLE = 0,
	READ_RTDATA_MODE,
	INTO_CONFIG_MODE,
	EXIT_CONFIG_MODE,
	READ_MACHINE_CODE,
	READ_PRODUCT_CODE,
	READ_PRODUCT_COMPLETE_CODE,
	GET_PARAMETER_SUCCESS,
	GET_PARAMETER_FAIL
}DISPLAY_DATA_TYPE;

typedef struct BAR_CODE
{
	unsigned char id;
	unsigned char code[20];
}BarCode;

typedef union DISPLAY_DATA
{
	CHANNEL_PARAMETER parameter[2];
	BarCode barCode;
}DisplayData;

typedef struct DISPLAYDATA
{
	unsigned char type;
	DisplayData data;
}DisplayPakage;

void sendDisplayMsg(DISPLAY_DATA_TYPE type, void *data, char size, unsigned char isInterrupt);
void initDisplayTask(void);
void displayTask(void *pvParameters);



#endif /* INC_DISPLAY_H_ */
