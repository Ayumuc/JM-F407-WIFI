/*
 * MachineStatusManager.c
 *
 *  Created on: Sep 23, 2021
 *      Author: Administrator
 */

#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "cJSON.h"
#include "MachineStatusManager.h"
#include "stdio.h"
#include "display.h"
#include "FreeRTOS.h"
#include "dataCollector.h"
#include "queue.h"
#include "boardConfig.h"
#include "sd2078.h"
#include "rtc.h"
#include "myfunction.h"

//static int waitParameterMachine = 0;
extern xQueueHandle collectionDataQueue;
MACHINE machines[4] = {0};


void printConfig(DEVICE_CONFIG *config)//打印配置
{
	unsigned char i;
	printf("-------------------------config info-------------------------\r\n");
 	for(i=0; i<4; i++)
 	{
 		printf("machine[%d],range0:%f,value0Type:%d, range1:%f, value1Type:%d\r\n",i,
		config->machineInfo.channelInfo[i].range0,
		config->machineInfo.channelInfo[i].value0Type,
		config->machineInfo.channelInfo[i].range1,
		config->machineInfo.channelInfo[i].value1Type);
 	}
	printf("------------------------------------------------------------\r\n\r\n");
}

//4~20mA Vref=2.5V
//110R*4mA=440mV
//110*20mA=2200mV
//440mV/2500mV*4095=720
//2200mV/2500mV*4095=3603
//100/(3603-720)=0.034686

//0~10V Vref=2.5V
//(39/(150+39+0.02)*10/2500)*4095=3379
void initMachineStatus(DEVICE_CONFIG* config)
{
	int i=0;
	int adcTopValue=0;

	//config = readConfig();
	printf("init machine status\r\n");

	printf("serverip:%d.%d.%d.%d serverport:%d\r\n",config->server_ip.ip_cfg[0],config->server_ip.ip_cfg[1],config->server_ip.ip_cfg[2],config->server_ip.ip_cfg[3],config->server_ip.server_port_cfg);
	printf("wifi_ip:%s  wifi_port:%s\r\n",config->wifi_ip.wip,config->wifi_ip.wport);
	printf("wifi_ssid:%s\r\nwifi_pwd:%s\r\n",config->wifi_info.ssid,config->wifi_info.pwd);

#if LOG_PRINTF_DEF  //ip等信息
	snprintf(log_msg_global, sizeof(log_msg_global), "ip:%d.%d.%d.%d",config->eth_info.ip_cfg[0],config->eth_info.ip_cfg[1],config->eth_info.ip_cfg[2],config->eth_info.ip_cfg[3]);
	LogPrint(log_msg_global);

	snprintf(log_msg_global, sizeof(log_msg_global), "netmask:%d.%d.%d.%d",	config->eth_info.netmask_cfg[0],config->eth_info.netmask_cfg[1],config->eth_info.netmask_cfg[2],config->eth_info.netmask_cfg[3]);
	LogPrint(log_msg_global);

	snprintf(log_msg_global, sizeof(log_msg_global), "gateway:%d.%d.%d.%d",config->eth_info.gateway_cfg[0],	config->eth_info.gateway_cfg[1],config->eth_info.gateway_cfg[2],config->eth_info.gateway_cfg[3]);
	LogPrint(log_msg_global);
#endif

#if LOG_PRINTF_DEF //模拟量等信息
	sprintf(log_msg_global, "A10 range: %lf    A10 type: %d", config->analog_info.a10_range, config->analog_info.a10_type);
	LogPrint(log_msg_global);
	sprintf(log_msg_global, "A11 range: %lf    A11 type: %d", config->analog_info.a11_range, config->analog_info.a11_type);
	LogPrint(log_msg_global);
	sprintf(log_msg_global, "A20 range: %lf    A20 type: %d", config->analog_info.a20_range, config->analog_info.a20_type);
	LogPrint(log_msg_global);
	sprintf(log_msg_global, "A21 range: %lf    A21 type: %d", config->analog_info.a21_range, config->analog_info.a21_type);
	LogPrint(log_msg_global);
	sprintf(log_msg_global, "A30 range: %lf    A30 type: %d", config->analog_info.a30_range, config->analog_info.a30_type);
	LogPrint(log_msg_global);
	sprintf(log_msg_global, "A31 range: %lf    A31 type: %d", config->analog_info.a31_range, config->analog_info.a31_type);
	LogPrint(log_msg_global);
	sprintf(log_msg_global, "A40 range: %lf    A40 type: %d", config->analog_info.a40_range, config->analog_info.a40_type);
	LogPrint(log_msg_global);
	sprintf(log_msg_global, "A41 range: %lf    A41 type: %d", config->analog_info.a41_range, config->analog_info.a41_type);
	LogPrint(log_msg_global);
//	LogPrint("-------------------------------------------------------------\r\n");
#else
		printf("----------------------analog input info----------------------\r\n");
		printf("A10 range: %lf    A10 type: %d\r\n", config->machineInfo.channelInfo[0].range0, config->machineInfo.channelInfo[0].value0Type);
		printf("A11 range: %lf    A11 type: %d\r\n", config->machineInfo.channelInfo[0].range1, config->machineInfo.channelInfo[0].value1Type);
		printf("A20 range: %lf    A20 type: %d\r\n", config->machineInfo.channelInfo[1].range0, config->machineInfo.channelInfo[1].value0Type);
		printf("A21 range: %lf    A21 type: %d\r\n", config->machineInfo.channelInfo[1].range1, config->machineInfo.channelInfo[1].value1Type);
		printf("A30 range: %lf    A30 type: %d\r\n", config->machineInfo.channelInfo[2].range0, config->machineInfo.channelInfo[2].value0Type);
		printf("A31 range: %lf    A31 type: %d\r\n", config->machineInfo.channelInfo[2].range1, config->machineInfo.channelInfo[2].value1Type);
		printf("A40 range: %lf    A40 type: %d\r\n", config->machineInfo.channelInfo[3].range0, config->machineInfo.channelInfo[3].value0Type);
		printf("A41 range: %lf    A41 type: %d\r\n", config->machineInfo.channelInfo[3].range1, config->machineInfo.channelInfo[3].value1Type);
		printf("-------------------------------------------------------------\r\n\r\n");
#endif

	config->machineInfo.channelInfo[0].range0 = config->analog_info.a10_range;
	config->machineInfo.channelInfo[0].range1 = config->analog_info.a11_range;
	config->machineInfo.channelInfo[1].range0 = config->analog_info.a20_range;
	config->machineInfo.channelInfo[1].range1 = config->analog_info.a21_range;
	config->machineInfo.channelInfo[2].range0 = config->analog_info.a30_range;
	config->machineInfo.channelInfo[2].range1 = config->analog_info.a31_range;
	config->machineInfo.channelInfo[3].range0 = config->analog_info.a40_range;
	config->machineInfo.channelInfo[3].range1 = config->analog_info.a41_range;

	config->machineInfo.channelInfo[0].value0Type = config->analog_info.a10_type;
	config->machineInfo.channelInfo[0].value1Type = config->analog_info.a11_type;
	config->machineInfo.channelInfo[1].value0Type = config->analog_info.a20_type;
	config->machineInfo.channelInfo[1].value1Type = config->analog_info.a21_type;
	config->machineInfo.channelInfo[2].value0Type = config->analog_info.a30_type;
	config->machineInfo.channelInfo[2].value1Type = config->analog_info.a31_type;
	config->machineInfo.channelInfo[3].value0Type = config->analog_info.a40_type;
	config->machineInfo.channelInfo[3].value1Type = config->analog_info.a41_type;

	printConfig(config);

	for(i=0;i<4;i++)
	{
//		machines[i].minCollectionValue[0] = 300;
//		machines[i].minCollectionValue[1] = 300;

		machines[i].channelParameter[0].minAdcValue = 0;
		if(config->machineInfo.channelInfo[i].value0Type == IS4TO20MA)
		{
			machines[i].channelParameter[0].minAdcValue = (720);
		}
		adcTopValue = 3604;
		if(config->machineInfo.channelInfo[i].value0Type == IS0TO10V)
		{
			adcTopValue = 3379;
		}
		machines[i].channelParameter[0].maxAdcValue = config->machineInfo.channelInfo[i].range0;
		machines[i].channelParameter[0].scale = (double)(config->machineInfo.channelInfo[i].range0) /
																						(double)(adcTopValue-machines[i].channelParameter[0].minAdcValue);

		machines[i].channelParameter[1].minAdcValue = 0;
		if(config->machineInfo.channelInfo[i].value1Type == IS4TO20MA)
		{
			machines[i].channelParameter[1].minAdcValue = (720);
		}
		adcTopValue = 3604;
		if(config->machineInfo.channelInfo[i].value1Type == IS0TO10V)
		{
			adcTopValue = 3379;
		}
		machines[i].channelParameter[1].maxAdcValue = config->machineInfo.channelInfo[i].range1;
		machines[i].channelParameter[1].scale = (double)(config->machineInfo.channelInfo[i].range1) /
																						(double)(adcTopValue-machines[i].channelParameter[1].minAdcValue);
		printf("machine[%d]scale[0]= %.5lf, scale[1]= %.5lf\r\n",i,machines[i].channelParameter[0].scale , machines[i].channelParameter[1].scale );
	}
}
