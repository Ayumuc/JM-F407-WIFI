/*
 * dataCollector.c
 *
 *  Created on: Sep 23, 2021
 *      Author: Administrator
 */


#include "stdint.h"
#include "MachineStatusManager.h"
#include "dataCollector.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <time.h>
#include "MCP3208.h"
#include "dataFilter.h"
#include "delay.h"
#include "Temperature.h"
#include "rs485.h"
float Key_Start_Stop(void);
xQueueHandle collectionDataQueue;
SIMPLE_QUEUE dataQueue[8];

extern MACHINE machines[4];
uint8_t key[2];


double analogValues[20];
unsigned char analogValueBytes[72];

double *getAnalogValues(void)
{
	return analogValues;
}

void floatToByte(float f,unsigned char *byte)
{
	unsigned char *pdata;
	pdata = (unsigned char*)&f;
	byte[1] = *pdata++;
	byte[0] = *pdata++;
	byte[3] = *pdata++;
	byte[2] = *pdata;
}


unsigned char* getAnalogValueBytes(void)
{
	int8_t i;
//	Key_Start_Stop();
//	analogValues[2] = key[0];
//	analogValues[3] = key[1];
	analogValues[8] = (double)atoi(speed_value);
//	analogValues[9] = (double)LP20_TOF.Data_Value_Sum;
	//printf("Data_Value_Sum:%f\r\n",analogValues[9]);
	//printf("analogValues:%f speed_value:%s\r\n",analogValues[8],speed_value);
	for(i=0;i<18;i++)
	{
		floatToByte(analogValues[i], &analogValueBytes[i*4]);
	}
	return analogValueBytes;
}


void initDataCollector(void)
{
	int8_t i;
	collectionDataQueue = xQueueCreate(30, sizeof(COLLECTION_DATA));
	for(i=0; i<8; i++)
	{
		simpleQueueInit(&dataQueue[i]);
	}
}

#define ADC_4MA 720
double getChannelValue(int8_t ch, double scale, int32_t minAdcValue,double maxAdcValue)
{
	uint16_t adcValue;

	uint16_t val = mcpSample(ch);
	double ret = 0;
	double Temperature;
	double MaxTemperature = maxAdcValue;
	double PerAdc;
	adcValue = enQueueAndCalcAverage(&dataQueue[ch], val); //fix me:输出的adc值最大只能到18.4ma，当模拟值输入18.4ma已经到达最大值4095 ---------->查出是硬件电路问题
	if(adcValue == 0){return 0;}
#if 0
	if(val != 0 && ch == 4 | ch == 5 | ch == 6 | ch == 7){
		PerAdc = ((3604 - ADC_4MA)/16)/(12.5);
		if(ch == 5){
			//PerAdc = ((3604 - ADC_4MA)/14)/(1.875);
			PerAdc = (3604 - ADC_4MA)/30;
		}
		else{
			//PerAdc = ((3604 - ADC_4MA)/14)/(5);
			PerAdc = (3604 - ADC_4MA)/80;
		}
		Temperature = (val-ADC_4MA)/PerAdc;
		if(Temperature < 0 ){
			Temperature = 0 ;
		}
		if(Temperature > MaxTemperature){
			Temperature = MaxTemperature;
		}
		Tem.My_Temperature = Temperature;
		return Tem.My_Temperature;
	}
#endif
	if(adcValue < minAdcValue)
	{
		adcValue = minAdcValue;
	}
	ret = (double)(((adcValue-minAdcValue)*scale) );
	if(ret > (maxAdcValue)){
		return (maxAdcValue);
	}
	else{
		return (ret);
	}

}

//20ms周期，200us执行完成
void collectDataHandler(void)
{
	int8_t i;

	for(i=0; i<4; i++)
	{
		analogValues[i*2]   =  getChannelValue(i*2,   machines[i].channelParameter[0].scale, machines[i].channelParameter[0].minAdcValue,machines[i].channelParameter[0].maxAdcValue);
		analogValues[i*2+1] =  getChannelValue(i*2+1, machines[i].channelParameter[1].scale, machines[i].channelParameter[1].minAdcValue,machines[i].channelParameter[1].maxAdcValue);
	}
}


#define KEY_START()                    !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)
#define KEY_STOP()                     !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9)
float Key_Start_Stop(void){
	float Key_Start = KEY_START();
	float Key_Stop = KEY_STOP();
	key[0] = Key_Start;
	key[1] = Key_Stop;
	printf("key[0]:%f\r\n",key[0]);
	printf("key[1]:%f\r\n",key[1]);
	return 0;
}
