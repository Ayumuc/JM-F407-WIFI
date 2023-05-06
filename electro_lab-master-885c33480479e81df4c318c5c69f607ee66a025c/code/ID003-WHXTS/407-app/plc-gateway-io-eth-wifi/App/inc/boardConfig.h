/*
 * boardConfig.h
 *
 *  Created on: Oct 9, 2021
 *      Author: Administrator
 */

#ifndef INC_BOARDCONFIG_H_
#define INC_BOARDCONFIG_H_

#include "stdint.h"


//
//typedef struct
//{
//	int         fisrtBoot;
//	MAC_INFO    mac_info;
//	ETH_INFO    eth_info;
//	ANALOG_INFO analog_info;
//	SERVER_IP   server_ip;
//}CONFIG_INFO;

typedef enum
{
	IS4TO20MA = 0,
	IS0TO20MA = 1,
	IS0TO10V  = 2
}VALUE_TYPE;

typedef struct
{
	double range0;
	char value0Type;
	double range1;
	char value1Type;
}CHANNEL_INFO;

typedef struct
{
	CHANNEL_INFO channelInfo[4];
}MACHINE_INFO;

typedef struct
{
	char mac_name[20];
}MAC_INFO;

typedef struct
{
	uint8_t ip_cfg[4];
	uint8_t gateway_cfg[4];
	uint8_t netmask_cfg[4];
	int     port_cfg;
}ETH_INFO;


typedef struct{
	double min;
	double max;

	double A10_ma4;
	double A11_ma4;
	double A20_ma4;
	double A21_ma4;
	double A30_ma4;
	double A31_ma4;
	double A40_ma4;
	double A41_ma4;

	double A10_ma20;
	double A11_ma20;
	double A20_ma20;
	double A21_ma20;
	double A30_ma20;
	double A31_ma20;
	double A40_ma20;
	double A41_ma20;

}RANG;

typedef struct
{
	double a10_range;
	double a11_range;
	double a20_range;
	double a21_range;
	double a30_range;
	double a31_range;
	double a40_range;
	double a41_range;

	char a10_type;
	char a11_type;
	char a20_type;
	char a21_type;
	char a30_type;
	char a31_type;
	char a40_type;
	char a41_type;
}ANALOG_INFO;

typedef struct
{
	uint8_t ip_cfg[4];
	int     server_port_cfg;
}SERVER_IP;

typedef struct
{
	char ssid[20];
	char pwd[20];
}WIFI_INFO;

typedef struct
{
	char wip[16];
	char wsip[16];
	char wport[6];
}WIFI_IP;

typedef struct
{
	WIFI_IP     wifi_ip;
	WIFI_INFO	wifi_info;
	ETH_INFO    eth_info;

	int firstBoot;
	MACHINE_INFO machineInfo;

	MAC_INFO    mac_info;
	RANG        range_info;
	ANALOG_INFO analog_info;
	SERVER_IP   server_ip;
}DEVICE_CONFIG;



//extern CONFIG_INFO         config_info;
extern DEVICE_CONFIG  config;

char isFirstBoot(void);
char saveConfig(DEVICE_CONFIG *s);
DEVICE_CONFIG* readConfig(void);
char LoadDefaultConfig(void);
void configRecv(uint8_t *ch, uint16_t len);
void configHandler(uint8_t *buf);



#endif /* INC_BOARDCONFIG_H_ */
