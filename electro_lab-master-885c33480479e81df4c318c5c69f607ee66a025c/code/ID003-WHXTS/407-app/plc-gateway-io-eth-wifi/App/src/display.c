///*
// * display.c
// *
// *  Created on: Sep 23, 2021
// *      Author: Administrator
// */
//
//#include <string.h>
//#include <stdio.h>
//#include "oled.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "MachineStatusManager.h"
//#include "dataCollector.h"
//#include "MCP3208.h"
//#include "display.h"
//#include "cJSON.h"
//
//#define READ_RTDATA_RESPONSE       12
//
//xQueueHandle xQueueDisplay;
//extern TaskHandle_t displayTaskHandle;
//
//void initDisplayTask(void)
//{
//	xQueueDisplay = xQueueCreate(1, sizeof(DisplayPakage));
//}
//
////4ms执行完成
//void displayValues(double* values)//显示8个AD通道的值
//{
//	uint8_t buf[32];
//	char i;
//	OLED_Fill(0);
//
//	for(i=0; i<4; i++)
//	{
//		sprintf((char *)buf, "%.1f", values[i]);
//		OLED_P8x16Str(0, i*2, buf, 0);
//	}
//
//	for(i=4; i<8; i++)
//	{
//		sprintf((char *)buf, "%.1f", values[i]);
//		OLED_P8x16Str(72, (i-4)*2, buf, 0);
//	}
//}
//
//
//void sendDisplayMsg(DISPLAY_DATA_TYPE type, void *data, char size, uint8_t isInterrupt)//发送显示信息
//{
//	DisplayPakage package;
//	package.type = type;
//	printf("display type:%d\r\n", type);
//	memcpy(&package.data, data, size);
//
//	if(xQueueDisplay == 0)
//	{
//		return;
//	}
//
//	vTaskSuspend(displayTaskHandle);
//	vTaskResume(displayTaskHandle);
//
//	if(isInterrupt)
//	{
//		xQueueSendFromISR( xQueueDisplay, ( void * ) &package, (TickType_t)0);
//	}
//	else
//	{
//		xQueueSend( xQueueDisplay, ( void * ) &package, 0);
//	}
//}
//
//void sendRtdata(double* values)//发送实时数据
//{
//	cJSON *rtdata;
//	int errorCode = 0;
//	char* out;
//	rtdata=cJSON_CreateObject();
//	cJSON_AddNumberToObject(rtdata, "value1", values[0]);
//	cJSON_AddNumberToObject(rtdata, "value2", values[1]);
//	cJSON_AddNumberToObject(rtdata, "value3", values[2]);
//	cJSON_AddNumberToObject(rtdata, "value4", values[3]);
//	cJSON_AddNumberToObject(rtdata, "value5", values[4]);
//	cJSON_AddNumberToObject(rtdata, "value6", values[5]);
//	cJSON_AddNumberToObject(rtdata, "value7", values[6]);
//	cJSON_AddNumberToObject(rtdata, "value8", values[7]);
//
////	cJSON_AddNumberToObject(rtdata, "input1", readDigitInputStatus(0));
////	cJSON_AddNumberToObject(rtdata, "input2", readDigitInputStatus(1));
////	cJSON_AddNumberToObject(rtdata, "input3", readDigitInputStatus(2));
////	cJSON_AddNumberToObject(rtdata, "input4", readDigitInputStatus(3));
////
////	cJSON_AddNumberToObject(rtdata, "output1", readDigitOutputStatus(0));
////	cJSON_AddNumberToObject(rtdata, "output2", readDigitOutputStatus(1));
////	cJSON_AddNumberToObject(rtdata, "output3", readDigitOutputStatus(2));
////	cJSON_AddNumberToObject(rtdata, "output4", readDigitOutputStatus(3));
//
//	responseAck(READ_RTDATA_RESPONSE,errorCode,"rtdata",rtdata);
//}
//
//void displayTask(void *pvParameters)
//{
//	uint8_t buf[20];
//	double *adcValues;
//	DisplayPakage displayPakage={0};
////	UBaseType_t taskHeap;
//
////	vTaskDelay(2);
//	printf("displayTask start\r\n");
//
//	while (1)
//	{
//		if(displayPakage.type == IDLE)
//		{
//			adcValues = getAnalogValues();
//			displayValues(adcValues);
//		}
//
//		if(xQueueReceive(xQueueDisplay, &displayPakage, (50)))
//		{
//			OLED_Fill(0x00);
//			switch(displayPakage.type)
//			{
//				case IDLE:
//					break;
//
//				case INTO_CONFIG_MODE:
//					OLED_P8x16Str(20, 2, "Config mode", 0);
//					break;
//
//				case READ_MACHINE_CODE:
//					sprintf((char *)buf, "Machine[%d]", displayPakage.data.barCode.id);
//					OLED_P8x16Str(0, 2, buf, 0);
//					OLED_P8x16Str(0, 4, displayPakage.data.barCode.code, 0);
//					break;
//
//				case READ_PRODUCT_CODE:
//					sprintf((char *)buf, "Machine[%d]", displayPakage.data.barCode.id);
//					OLED_P8x16Str(0, 0, buf, 0);
//					OLED_P8x16Str(0, 2, "Product code", 0);
//					OLED_P8x16Str(0, 4, displayPakage.data.barCode.code, 0);
//					OLED_P8x16Str(20, 6, "Waitting...", 0);
//					break;
//
//				case READ_PRODUCT_COMPLETE_CODE:
//					sprintf((char *)buf, "Machine[%d]", displayPakage.data.barCode.id);
//					OLED_P8x16Str(0, 0, buf, 0);
//					OLED_P8x16Str(0, 2, "Complete code", 0);
//					OLED_P8x16Str(0, 4, displayPakage.data.barCode.code, 0);
//					vTaskDelay((portTickType ) 500);
//					displayPakage.type = IDLE;
//					break;
//
//				case GET_PARAMETER_SUCCESS:
//					OLED_Fill(0x00);
//					sprintf((char *)buf, "v1std:%.2f", displayPakage.data.parameter[0].standard);
//					OLED_P8x16Str(0, 0, buf, 0);
//					sprintf((char *)buf, "v2std:%.2f", displayPakage.data.parameter[1].standard);
//					OLED_P8x16Str(0, 4, buf, 0);
//					vTaskDelay((portTickType ) 500);
//					displayPakage.type = IDLE;
//					break;
//
//				case GET_PARAMETER_FAIL:
//					OLED_Fill(0x00);
//					OLED_P8x16Str(24, 4, "Wait fail!", 0);
//					vTaskDelay((portTickType ) 500);
//					displayPakage.type = IDLE;
//					break;
//			}
//		}
////		taskHeap = uxTaskGetStackHighWaterMark(NULL);
////		printf("\r\n[displayTask logoinfo]:		displayTask available task heap:%d bytes\r\n" ,taskHeap*4);
//
//	}
//}
//
//
