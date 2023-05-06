/*
 * boardConfig.c
 *
 *  Created on: Oct 9, 2021
 *      Author: Administrator
 */


#include "boardConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "flash.h"
#include <strings.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "sd2078.h"
#include "myfunction.h"

#define CONFIG_DATA_ADDR					 0x080F0000
#define FIRSTBOOT                            0xaa


uint8_t config_buff[100];


char isFirstBoot(void)
{
	DEVICE_CONFIG* config = readConfig();
	if(config->firstBoot != FIRSTBOOT)
	{
		return 1;
	}
	return 0;
}


char saveConfig(DEVICE_CONFIG *s)//保存配置
{
	//printf("size of DEVICE_CONFIG %d bytes\r\n", sizeof(DEVICE_CONFIG));
	return STM32FLASH_Write(CONFIG_DATA_ADDR, (uint32_t *)s, sizeof(DEVICE_CONFIG)/4);
}

DEVICE_CONFIG* readConfig(void)//读出配置
{
	STM32FLASH_Read(CONFIG_DATA_ADDR, (uint32_t *)&config, sizeof(DEVICE_CONFIG)/4);
	//memcpy(&config,(unsigned int *)CONFIG_DATA_ADDR,sizeof(DEVICE_CONFIG));
	return &config;
}

char LoadDefaultConfig(void)//加载默认配置
{
	memset(&config, 0, sizeof(DEVICE_CONFIG));
	printf("load default config\r\n");

	config.machineInfo.channelInfo[0].range0 = 500;config.machineInfo.channelInfo[0].value0Type = IS4TO20MA;
	config.machineInfo.channelInfo[0].range1 = 100;config.machineInfo.channelInfo[0].value1Type = IS4TO20MA;
	config.machineInfo.channelInfo[1].range0 = 100;config.machineInfo.channelInfo[1].value0Type = IS4TO20MA;
	config.machineInfo.channelInfo[1].range1 = 100;config.machineInfo.channelInfo[1].value1Type = IS4TO20MA;
	config.machineInfo.channelInfo[2].range0 = 240;config.machineInfo.channelInfo[2].value0Type = IS0TO20MA;
	config.machineInfo.channelInfo[2].range1 = 100;config.machineInfo.channelInfo[2].value1Type = IS4TO20MA;
	config.machineInfo.channelInfo[3].range0 = 100;config.machineInfo.channelInfo[3].value0Type = IS4TO20MA;
	config.machineInfo.channelInfo[3].range1 = 100;config.machineInfo.channelInfo[3].value1Type = IS4TO20MA;

	config.analog_info.a10_range = 100;config.analog_info.a10_type = IS4TO20MA;
	config.analog_info.a11_range = 100;config.analog_info.a11_type = IS4TO20MA;
	config.analog_info.a20_range = 100;config.analog_info.a20_type = IS4TO20MA;
	config.analog_info.a21_range = 100;config.analog_info.a21_type = IS4TO20MA;
	config.analog_info.a30_range = 240;config.analog_info.a30_type = IS4TO20MA;
	config.analog_info.a31_range = 240;config.analog_info.a31_type = IS4TO20MA;
	config.analog_info.a40_range = 240;config.analog_info.a40_type = IS4TO20MA;
	config.analog_info.a41_range = 240;config.analog_info.a41_type = IS4TO20MA;

	config.eth_info.ip_cfg[0] = 192;
	config.eth_info.ip_cfg[1] = 168;
	config.eth_info.ip_cfg[2] = 20;
	config.eth_info.ip_cfg[3] = 191;
	config.eth_info.gateway_cfg[0] = 192;
	config.eth_info.gateway_cfg[1] = 168;
	config.eth_info.gateway_cfg[2] = 20;
	config.eth_info.gateway_cfg[3] = 1;
	config.eth_info.netmask_cfg[0] = 255;
	config.eth_info.netmask_cfg[1] = 255;
	config.eth_info.netmask_cfg[2] = 255;
	config.eth_info.netmask_cfg[3] = 0;
	config.eth_info.port_cfg = 502;

	strcpy(config.mac_info.mac_name, "JMKJ0760");

	config.server_ip.ip_cfg[0] = 192;
	config.server_ip.ip_cfg[0] = 168;
	config.server_ip.ip_cfg[0] = 20;
	config.server_ip.ip_cfg[0] = 100;
	config.server_ip.server_port_cfg = 8088;

	strcpy(config.wifi_ip.wip, "192.168.1.110");
	strcpy(config.wifi_ip.wsip, "192.168.1.101");
	strcpy(config.wifi_ip.wport, "4110");
	strcpy(config.wifi_info.ssid, "WHXTS001");
	strcpy(config.wifi_info.pwd, "JMKJ0760");

	config.firstBoot = FIRSTBOOT;
	saveConfig(&config);

	printf("load default config ok\r\n");

    return 0;
}

static void UartSendBytes(uint8_t *send_buff,uint32_t length)
{
  uint32_t i = 0;
  for(i=0;i<length;i++)
  {
    while((UART4->SR&0x40)==0);
    UART4->DR=send_buff[i];
  }
}

unsigned short ModBusCRC (unsigned char *ptr,unsigned char size)
{
  unsigned short a,b,tmp,CRC16,V;
  CRC16=0xffff;
  for (a=0; a<size; a++)
  {
    CRC16 =* ptr^CRC16;
    for (b=0; b<8; b++)
    {
	    tmp = CRC16 & 0x0001;
	    CRC16 = CRC16 >>1;
	    if (tmp)
		    CRC16=CRC16 ^ 0xa001;
    }
    *ptr++;
  }
  V = ((CRC16 & 0x00FF) << 8) | ((CRC16 & 0xFF00) >> 8) ;//éŤä˝ĺ­čč˝Źć˘
  return V;
}


static void localipStrHandler(uint8_t *buf)
{
  char *tmp_str;
  char _ip[20];
  char _port[20];
  char _netmask[30];
  char _gateway[30];

  uint8_t _ip_cfg[4];
  uint8_t _netmask_cfg[4];
  uint8_t _gateway_cfg[4];
  int     _port_cfg;

  if (!strncasecmp((char*)buf, "IP", 2)) //配置ip
  {
    { //一阶段
      tmp_str = strtok((char*)buf, ";");  //printf("%s\r\n", tmp_str);//"IP"192.168.3.47"
      strcpy(_ip, tmp_str);

      tmp_str = strtok(NULL, ";");  //printf("%s\r\n", tmp_str);//"PORT:5506"
      strcpy(_port, tmp_str);       //printf("%s\r\n", _port);//"PORT:5506"

      tmp_str = strtok(NULL, ";");  //printf("%s\r\n", tmp_str);//"NETMASK:255.255.255.255"
      strcpy(_netmask, tmp_str);    //printf("%s\r\n", _netmask);//"NETMASK:255.255.255.255"

      tmp_str = strtok(NULL, ";");  //printf("%s\r\n", tmp_str);//"GATEWAY:192.168.3.1"
      strcpy(_gateway, tmp_str);    //printf("%s\r\n", _gateway);//"GATEWAY:192.168.3.1"
    }

    { //二阶段
      //ip
      {
      tmp_str = strtok(_ip, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
      tmp_str = strtok(NULL, ":");  //printf("%d %s\r\n", __LINE__, tmp_str);  //"192.168.3.47"
      tmp_str = strtok(tmp_str, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"192"
      _ip_cfg[0] = atoi(tmp_str);  //printf("%d %d\r\n", __LINE__, _ip_cfg[0]); //192

      tmp_str = strtok(NULL, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"168"
      _ip_cfg[1] = atoi(tmp_str);  //printf("%d %d\r\n", __LINE__, _ip_cfg[1]); //168

      tmp_str = strtok(NULL, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"3"
      _ip_cfg[2] = atoi(tmp_str);  //printf("%d %d\r\n", __LINE__, _ip_cfg[2]); //3

      tmp_str = strtok(NULL, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"3"
      _ip_cfg[3] = atoi(tmp_str);  //printf("%d %d\r\n", __LINE__, _ip_cfg[3]); //3
      }

      //port
      {
      tmp_str = strtok(_port, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"PORT"
      tmp_str = strtok(NULL, ":");    //printf("%d %s\r\n", __LINE__, tmp_str);   //"5506"
      _port_cfg = atoi(tmp_str);       //printf("%d %d\r\n", __LINE__, _port_cfg); //5506
      }

      //netmask
      {
      tmp_str = strtok(_netmask, ":");  // printf("%d %s\r\n", __LINE__, tmp_str); //"NETMASK"
      tmp_str = strtok(NULL, ":");  //printf("%d %s\r\n", __LINE__, tmp_str);  //"255.255.255.255"
      tmp_str = strtok(tmp_str, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"255"
      _netmask_cfg[0] = atoi(tmp_str); // printf("%d %d\r\n", __LINE__, _netmask_cfg[0]); //255

      tmp_str = strtok(NULL, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"255"
      _netmask_cfg[1] = atoi(tmp_str);  //printf("%d %d\r\n", __LINE__, _netmask_cfg[1]); //255

      tmp_str = strtok(NULL, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"255"
      _netmask_cfg[2] = atoi(tmp_str);  //printf("%d %d\r\n", __LINE__, _netmask_cfg[2]); //255

      tmp_str = strtok(NULL, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"255"
      _netmask_cfg[3] = atoi(tmp_str);  //printf("%d %d\r\n", __LINE__, _netmask_cfg[3]); //255
      }

      //gateway
      {
      tmp_str = strtok(_gateway, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"GATEWAY"
      tmp_str = strtok(NULL, ":");       //printf("%d %s\r\n", __LINE__, tmp_str);  //"192.168.3.47"
      tmp_str = strtok(tmp_str, ".");    //printf("%d %s\r\n", __LINE__, tmp_str); //"192"
      _gateway_cfg[0] = atoi(tmp_str);    //printf("%d %d\r\n", __LINE__, _gateway_cfg[0]); //192

      tmp_str = strtok(NULL, ".");       //printf("%d %s\r\n", __LINE__, tmp_str); //"168"
      _gateway_cfg[1] = atoi(tmp_str);    //printf("%d %d\r\n", __LINE__, _gateway_cfg[1]); //168

      tmp_str = strtok(NULL, ".");       //printf("%d %s\r\n", __LINE__, tmp_str); //"3"
      _gateway_cfg[2] = atoi(tmp_str);    //printf("%d %d\r\n", __LINE__, _gateway_cfg[2]); //3

      tmp_str = strtok(NULL, ".");       //printf("%d %s\r\n", __LINE__, tmp_str); //"3"
      _gateway_cfg[3] = atoi(tmp_str);    //printf("%d %d\r\n", __LINE__, _gateway_cfg[3]); //1
      }


    }


    { //三阶段
      // ip
      config.eth_info.ip_cfg[0] = _ip_cfg[0];
      config.eth_info.ip_cfg[1] = _ip_cfg[1];
      config.eth_info.ip_cfg[2] = _ip_cfg[2];
      config.eth_info.ip_cfg[3] = _ip_cfg[3];

      //port
      config.eth_info.port_cfg = _port_cfg;

      //netmask
      config.eth_info.netmask_cfg[0] = _netmask_cfg[0];
      config.eth_info.netmask_cfg[1] = _netmask_cfg[1];
      config.eth_info.netmask_cfg[2] = _netmask_cfg[2];
      config.eth_info.netmask_cfg[3] = _netmask_cfg[3];

      //gateway
      config.eth_info.gateway_cfg[0] = _gateway_cfg[0];
      config.eth_info.gateway_cfg[1] = _gateway_cfg[1];
      config.eth_info.gateway_cfg[2] = _gateway_cfg[2];
      config.eth_info.gateway_cfg[3] = _gateway_cfg[3];
    }
  }
}


static void analogStrHandler(uint8_t *buf)
{
	char *tmp_str;

	char A10_range[20], A11_range[20], A20_range[20], A21_range[20], A30_range[20], A31_range[20], A40_range[20], A41_range[20];

	char A10_type[10], A11_type[10], A20_type[10], A21_type[10], A30_type[10], A31_type[10], A40_type[10], A41_type[10];

	double _A10_range, _A11_range, _A20_range, _A21_range, _A30_range, _A31_range, _A40_range, _A41_range;

	char _A10_type, _A11_type, _A20_type, _A21_type, _A30_type, _A31_type, _A40_type, _A41_type;

	if (!strncasecmp((char*)buf, "A10 range", 9)) //配置模拟量
	{
		tmp_str = strtok((char*)buf, ";");  //printf("%s\r\n", tmp_str);//A10 range
		strcpy(A10_range, tmp_str);

		tmp_str = strtok(NULL, ";");        //printf("%s\r\n", tmp_str);//A10 type
		strcpy(A10_type, tmp_str);

		tmp_str = strtok(A10_range, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A10_range = atof(tmp_str);
		config.analog_info.a10_range = _A10_range;		printf("a10 range: %lf\t", config.analog_info.a10_range);

		tmp_str = strtok(A10_type, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A10_type = atoi(tmp_str);
		config.analog_info.a10_type  = _A10_type;		printf("a10 type: %d\r\n", config.analog_info.a10_type);
	}
	else if (!strncasecmp((char*)buf, "A11 range", 9)) //配置模拟量
	{
		tmp_str = strtok((char*)buf, ";");  //printf("%s\r\n", tmp_str);//A10 range
		strcpy(A11_range, tmp_str);

		tmp_str = strtok(NULL, ";");        //printf("%s\r\n", tmp_str);//A10 type
		strcpy(A11_type, tmp_str);

		tmp_str = strtok(A11_range, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A11_range = atof(tmp_str);
		config.analog_info.a11_range = _A11_range;		printf("a11 range: %lf\t", config.analog_info.a11_range);

		tmp_str = strtok(A11_type, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A11_type = atoi(tmp_str);
		config.analog_info.a11_type  = _A11_type;		printf("a11 type: %d\r\n", config.analog_info.a11_type);
	}
	else if (!strncasecmp((char*)buf, "A20 range", 9)) //配置模拟量
	{
		tmp_str = strtok((char*)buf, ";");  //printf("%s\r\n", tmp_str);//A10 range
		strcpy(A20_range, tmp_str);

		tmp_str = strtok(NULL, ";");        //printf("%s\r\n", tmp_str);//A10 type
		strcpy(A20_type, tmp_str);

		tmp_str = strtok(A20_range, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A20_range = atof(tmp_str);
		config.analog_info.a20_range = _A20_range;		printf("a20 range: %lf\t", config.analog_info.a20_range);

		tmp_str = strtok(A20_type, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A20_type = atoi(tmp_str);
		config.analog_info.a20_type  = _A20_type;		printf("a20 type: %d\r\n", config.analog_info.a20_type);
	}
	else if (!strncasecmp((char*)buf, "A21 range", 9)) //配置模拟量
	{
		tmp_str = strtok((char*)buf, ";");  //printf("%s\r\n", tmp_str);//A10 range
		strcpy(A21_range, tmp_str);

		tmp_str = strtok(NULL, ";");        //printf("%s\r\n", tmp_str);//A10 type
		strcpy(A21_type, tmp_str);

		tmp_str = strtok(A21_range, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A21_range = atof(tmp_str);
		config.analog_info.a21_range = _A21_range;		printf("a21 range: %lf\t", config.analog_info.a21_range);

		tmp_str = strtok(A21_type, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A21_type = atoi(tmp_str);
		config.analog_info.a21_type  = _A21_type;		printf("a21 type: %d\r\n", config.analog_info.a21_type);
	}
	else if (!strncasecmp((char*)buf, "A30 range", 9)) //配置模拟量
	{
		tmp_str = strtok((char*)buf, ";");  //printf("%s\r\n", tmp_str);//A10 range
		strcpy(A30_range, tmp_str);

		tmp_str = strtok(NULL, ";");        //printf("%s\r\n", tmp_str);//A10 type
		strcpy(A30_type, tmp_str);

		tmp_str = strtok(A30_range, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A30_range = atof(tmp_str);
		config.analog_info.a30_range = _A30_range;		printf("a30 range: %lf\t", config.analog_info.a30_range);

		tmp_str = strtok(A30_type, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A30_type = atoi(tmp_str);
		config.analog_info.a30_type  = _A30_type;		printf("a30 type: %d\r\n", config.analog_info.a30_type);
	}
	else if (!strncasecmp((char*)buf, "A31 range", 9)) //配置模拟量
	{
		tmp_str = strtok((char*)buf, ";");  //printf("%s\r\n", tmp_str);//A10 range
		strcpy(A31_range, tmp_str);

		tmp_str = strtok(NULL, ";");        //printf("%s\r\n", tmp_str);//A10 type
		strcpy(A31_type, tmp_str);

		tmp_str = strtok(A31_range, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A31_range = atof(tmp_str);
		config.analog_info.a31_range = _A31_range;		printf("a31 range: %lf\t", config.analog_info.a31_range);

		tmp_str = strtok(A31_type, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A31_type = atoi(tmp_str);
		config.analog_info.a31_type  = _A31_type;		printf("a31 type: %d\r\n", config.analog_info.a31_type);
	}
	else if (!strncasecmp((char*)buf, "A40 range", 9)) //配置模拟量
	{
		tmp_str = strtok((char*)buf, ";");  //printf("%s\r\n", tmp_str);//A10 range
		strcpy(A40_range, tmp_str);

		tmp_str = strtok(NULL, ";");        //printf("%s\r\n", tmp_str);//A10 type
		strcpy(A40_type, tmp_str);

		tmp_str = strtok(A40_range, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A40_range = atof(tmp_str);
		config.analog_info.a40_range = _A40_range;		printf("a40 range: %lf\t", config.analog_info.a40_range);

		tmp_str = strtok(A40_type, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A40_type = atoi(tmp_str);
		config.analog_info.a40_type  = _A40_type;		printf("a40 type: %d\r\n", config.analog_info.a40_type);
	}
	else if (!strncasecmp((char*)buf, "A41 range", 9)) //配置模拟量
	{
		tmp_str = strtok((char*)buf, ";");  //printf("%s\r\n", tmp_str);//A10 range
		strcpy(A41_range, tmp_str);

		tmp_str = strtok(NULL, ";");        //printf("%s\r\n", tmp_str);//A10 type
		strcpy(A41_type, tmp_str);

		tmp_str = strtok(A41_range, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A41_range = atof(tmp_str);
		config.analog_info.a41_range = _A41_range;		printf("a41 range: %lf\t", config.analog_info.a41_range);

		tmp_str = strtok(A41_type, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
		tmp_str = strtok(NULL, ":");
		_A41_type = atoi(tmp_str);
		config.analog_info.a41_type  = _A41_type;		printf("a41 type: %d\r\n", config.analog_info.a41_type);
	}
}

static void macStrHandler(uint8_t *buf)
{
	char *tmp_str;
//	char _mac_name[10];

	if (!strncasecmp((char*)buf, "mac_name", 8)) //配置模拟量
	{
		tmp_str = strtok((char*)buf, ":");
		tmp_str = strtok(NULL, ":");
		strcpy(config.mac_info.mac_name, tmp_str);
	}
}

//{"wifi_ssid:JM3399;wifi_pwd:12345678;"}
static void wifiStrHandler(uint8_t *buf)
{
	char *tmp_str;
	char ssid[30], pwd[30];

	if (!strncasecmp((char*)buf, "wifi_ssid", 9))
	{
		tmp_str = strtok((char*)buf, ";");
		printf("%s\r\n", tmp_str);
		strcpy(ssid, tmp_str);

		tmp_str = strtok(NULL, ";");
		printf("%s\r\n", tmp_str);
		strcpy(pwd, tmp_str);

		tmp_str = strtok(ssid, ":");
		tmp_str = strtok(NULL, ":");
		printf("%s\r\n", tmp_str);
		strcpy(config.wifi_info.ssid, tmp_str);

		tmp_str = strtok(pwd, ":");
		tmp_str = strtok(NULL, ":");
		printf("%s\r\n", tmp_str);
		strcpy(config.wifi_info.pwd, tmp_str);
	}
}

#define CONFIG_KEY_DOWN()                    !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)
#define CONFIG_KEY_UP()                       HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)

static void Set_Handle(uint8_t *buf){
	if (!strncasecmp((char*)buf, "Set", 3)){
	char txt_buf[30] = {0};uint16_t ret = 0;uint16_t val = 0 ;uint16_t val_before = 0;
	int flag= 0;int ch = 0;int flag1 = 0;int cont = 0;int con1 = 0;int count1 = 10;int count2 = 4;int key_flag_down = 0;int key_flag_up = 0;
		for(uint8_t channel = 0 ; channel < 16 ; channel++){
			sprintf(txt_buf,"Please set A%d %dMA",count1,count2);
			printf("%s\r\n",txt_buf);
			bzero(txt_buf,sizeof(txt_buf));
			val = mcpSample(ch);
			val_before = val ;

			while(1){
				val = mcpSample(ch);
				if(val == 0){con1++;}
				if(count2 == 4 && ((val-val_before) < (4) && (val-val_before) > (-4)) && val >= 850 && val <= 1000){cont++;con1=0;}
				else if(count2 == 20 && ((val-val_before) < (4) && (val-val_before) > (-4)) && val >= 3900 && val <= 4095){cont++;con1=0;}
				else{cont = 0;}
				val_before = val ;
				printf("ch:[%d]  Val:[%d]\r\n",ch,val);
				for_delay_us(200000);
				if(con1 >= 10){printf("Your interface may not have access devices!\r\n");con1=0;}
				if(cont >= 10){
					cont = 0;
					switch(flag){
					case 0:config.range_info.A10_ma4 = atoi(val); cont = 0;   printf("A10_ma4 success\r\n");break;
					case 1:config.range_info.A10_ma20 = atoi(val); cont = 0;   printf("A10_ma20 success\r\n");break;
					case 2:config.range_info.A11_ma4 = atoi(val); cont = 0;   printf("A11_ma4 success\r\n");break;
					case 3:config.range_info.A11_ma20 = atoi(val); cont = 0;   printf("A11_ma20 success\r\n");break;
					case 4:config.range_info.A20_ma4 = atoi(val); cont = 0;   printf("A20_ma4 success\r\n");break;
					case 5:config.range_info.A20_ma20 = atoi(val); cont = 0;   printf("A20_ma20 success\r\n");break;
					case 6:config.range_info.A21_ma4 = atoi(val); cont = 0;   printf("A21_ma4 success\r\n");break;
					case 7:config.range_info.A21_ma20 = atoi(val); cont = 0;   printf("A21_ma20 success\r\n");break;
					case 8:config.range_info.A30_ma4 = atoi(val); cont = 0;   printf("A30_ma4 success\r\n");break;
					case 9:config.range_info.A30_ma20 = atoi(val); cont = 0;   printf("A30_ma20 success\r\n");break;
					case 10:config.range_info.A31_ma4 = atoi(val); cont = 0;   printf("A31_ma4 success\r\n");break;
					case 11:config.range_info.A31_ma20 = atoi(val); cont = 0;   printf("A31_ma20 success\r\n");break;
					case 12:config.range_info.A40_ma4 = atoi(val); cont = 0;   printf("A40_ma4 success\r\n");break;
					case 13:config.range_info.A40_ma20 = atoi(val); cont = 0;   printf("A40_ma20 success\r\n");break;
					case 14:config.range_info.A41_ma4 = atoi(val); cont = 0;   printf("A41_ma4 success\r\n");break;
					case 15:config.range_info.A41_ma20 = atoi(val); cont = 0;   printf("A41_ma20 success\r\n");break;
					default:break;
					}
					break;
				}
			}
			printf("cont:%d\r\n",cont);
			if(((channel+1)%2) == 0){ch++;}
			flag++;
			flag1++;
			if((flag)%2 != 0){count2 += 16;}
			else{count2 -= 16;}
			if(flag1%2 == 0 && flag1%4 != 0){count1 += 1;}
			else if(flag1%4 == 0){count1 += 9;}
			printf("Please press the key to set next AM\r\n");
			while(1){
				ret = CONFIG_KEY_UP();
				if(ret == GPIO_PIN_SET){key_flag_down = 1;}
				else if(ret == GPIO_PIN_RESET){key_flag_up = 1;}
				if(key_flag_up == 1 && key_flag_down == 1){
					key_flag_up = 0;
					key_flag_down = 0;
					break;
				}
			}
		}
	}
}


//{"RS"}
static Set_radar(uint8_t* buf){

}

//{"serverip:192.168.1.100;port:5506"}
static void serveripStrHandler(uint8_t *buf)
{
	char *tmp_str;
	char _ip[40];
	char _port[40];

	uint8_t _ip_cfg[4];
	int     _port_cfg;

	if (!strncasecmp((char*)buf, "serverip", 8)) //配置模拟量
	{
		{ //一阶段
			tmp_str = strtok((char*)buf, ";");  //printf("%s\r\n", tmp_str);//"IP"192.168.3.47"
			strcpy(_ip, tmp_str);

			tmp_str = strtok(NULL, ";");  //printf("%s\r\n", tmp_str);//"PORT:5506"
			strcpy(_port, tmp_str);       //printf("%s\r\n", _port);//"PORT:5506"
		}

		{  //二阶段
			//ip
			{
			tmp_str = strtok(_ip, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"IP"
			tmp_str = strtok(NULL, ":");  //printf("%d %s\r\n", __LINE__, tmp_str);  //"192.168.3.47"
			tmp_str = strtok(tmp_str, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"192"
			_ip_cfg[0] = atoi(tmp_str);  //printf("%d %d\r\n", __LINE__, _ip_cfg[0]); //192

			tmp_str = strtok(NULL, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"168"
			_ip_cfg[1] = atoi(tmp_str);  //printf("%d %d\r\n", __LINE__, _ip_cfg[1]); //168

			tmp_str = strtok(NULL, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"3"
			_ip_cfg[2] = atoi(tmp_str);  //printf("%d %d\r\n", __LINE__, _ip_cfg[2]); //3

			tmp_str = strtok(NULL, ".");  //printf("%d %s\r\n", __LINE__, tmp_str); //"3"
			_ip_cfg[3] = atoi(tmp_str);  //printf("%d %d\r\n", __LINE__, _ip_cfg[3]); //3
			}

			//port
			{
			tmp_str = strtok(_port, ":");   //printf("%d %s\r\n", __LINE__, tmp_str); //"PORT"
			tmp_str = strtok(NULL, ":");    //printf("%d %s\r\n", __LINE__, tmp_str);   //"5506"
			_port_cfg = atoi(tmp_str);       //printf("%d %d\r\n", __LINE__, _port_cfg); //5506
			}
		}

		{
			config.server_ip.ip_cfg[0] = _ip_cfg[0];
			config.server_ip.ip_cfg[1] = _ip_cfg[1];
			config.server_ip.ip_cfg[2] = _ip_cfg[2];
			config.server_ip.ip_cfg[3] = _ip_cfg[3];

			config.server_ip.server_port_cfg = _port_cfg;
		}
	}
}

//{"wifi_ip:192.168.20.201;wifi_port:501;"}
static void wifiserveripStrHandler(uint8_t *buf)
{
	char *tmp_str;
	char wip[25], wsip[25], wport[20];

	if (!strncasecmp((char*)buf, "wifi_ip", 7)) //配置模拟量
	{
		tmp_str = strtok((char*)buf, ";");
		printf("%s\r\n", tmp_str);
		strcpy(wip, tmp_str);

		tmp_str = strtok(NULL, ";");
		printf("%s\r\n", tmp_str);
		strcpy(wsip, tmp_str);

		tmp_str = strtok(NULL, ";");
		printf("%s\r\n", tmp_str);
		strcpy(wport, tmp_str);

		tmp_str = strtok(wip, ":");
		tmp_str = strtok(NULL, ":");
		printf("%s\r\n", tmp_str);
		strcpy(config.wifi_ip.wip, tmp_str);

		tmp_str = strtok(wsip, ":");
		tmp_str = strtok(NULL, ":");
		printf("%s\r\n", tmp_str);
		strcpy(config.wifi_ip.wsip, tmp_str);

		tmp_str = strtok(wport, ":");
		tmp_str = strtok(NULL, ":");
		printf("%s\r\n", tmp_str);
		strcpy(config.wifi_ip.wport, tmp_str);
	}
}



//年-月-日-时-分-秒-星期
//{"sd2078rtc:21-12-29-10-05-00-03;"}
static void sd2078rtcStrHandler(uint8_t *buf)
{
	Time_Def time_init={0x00,0x31,0x09,0x04,0x16,0x12,0x21};	//初始化实时时间
	Time_Def sysTime = {0};
	char *tmp_str;
	char vlauestr[28];
	char sendbuf[64] = {0};

	if (!strncasecmp((char*)buf, "sd2078rtc", strlen("sd2078rtc"))) //配置模拟量
	{
		{ //一阶段
			tmp_str = strtok((char*)buf, ":");  //sd2078rtc
			tmp_str = strtok(NULL, ";");        //21-12-29-10-05-00-03
			strcpy(vlauestr, tmp_str);          //21-12-29-10-05-00-03
		}
		{// 二阶段
			tmp_str = strtok(vlauestr, "-");
			time_init.year = atoi(tmp_str);//21
			time_init.year = decimal_bcd_code(time_init.year);

			tmp_str = strtok(NULL, "-");
			time_init.month = atoi(tmp_str); //12
			time_init.month = decimal_bcd_code(time_init.month);

			tmp_str = strtok(NULL, "-");
			time_init.day = atoi(tmp_str); //29
			time_init.day = decimal_bcd_code(time_init.day);

			tmp_str = strtok(NULL, "-");
			time_init.hour = atoi(tmp_str); //10
			time_init.hour = decimal_bcd_code(time_init.hour);

			tmp_str = strtok(NULL, "-");
			time_init.minute = atoi(tmp_str); //05
			time_init.minute = decimal_bcd_code(time_init.minute);

			tmp_str = strtok(NULL, "-");
			time_init.second = atoi(tmp_str); //00
			time_init.second = decimal_bcd_code(time_init.second);

			tmp_str = strtok(NULL, "-");
			time_init.week = atoi(tmp_str); //03
			time_init.week = decimal_bcd_code(time_init.week);
		}



		RTC_WriteDate(&time_init);
		RTC_ReadDate(&sysTime);
		snprintf(sendbuf, sizeof(sendbuf), "SD2078RTC:%02X-%02X-%02X  %02X:%02X:%02X  week:%02X",
		sysTime.year, sysTime.month, sysTime.day, sysTime.hour,	sysTime.minute, sysTime.second, sysTime.week);

		UartSendBytes((uint8_t*)sendbuf, strlen(sendbuf));
		printf(sendbuf);
	}
}


//{"IP:192.168.3.47;PORT:5506;NETMASK:255.255.255.0;GATEWAY:192.168.3.1;"}
//
//{"A10 range:500;A10 type:0"}
//{"A11 range:400;A11 type:0"}
//{"A20 range:300;A20 type:0"}
//{"A21 range:100;A21 type:0"}
//{"A30 range:90;A30 type:0"}
//{"A31 range:80;A31 type:0"}
//{"A40 range:70;A40 type:0"}
//{"A41 range:60;A41 type:0"}
//
//{"mac_name:JM001"}
//
//{"serverip:192.168.1.100;port:5506"}
//
//	type 一共三种类型：
//	0:4~20mA
//	1:0~20mA
//	2:0~10V
static char strHandler(uint8_t *buf)
{
	char ret = -10;

	localipStrHandler(buf);

	analogStrHandler(buf);

	macStrHandler(buf);

	Set_radar(buf);

	Set_Handle(buf);

	serveripStrHandler(buf);

	wifiserveripStrHandler(buf);

	wifiStrHandler(buf);

	sd2078rtcStrHandler(buf);

	ret = saveConfig(&config);

	return ret;
}

static void handler_1(uint8_t *buf)
{
	UartSendBytes((uint8_t *)"hello", sizeof("hello"));
}

static void handler_2(uint8_t *buf)
{
	UartSendBytes((uint8_t *)"bye", sizeof("bye"));
}

static void handler_3(uint8_t *buf)
{
	UartSendBytes(buf, strlen((char *)buf));
	if (strHandler(buf) == 0)
	{
		UartSendBytes((uint8_t *)"\r\nsave done\r\n", sizeof("\r\nsave done\r\n"));
	}
	else
	{
		UartSendBytes((uint8_t *)"\r\nsave fail\r\n", sizeof("\r\nsave fail\r\n"));
	}
}

typedef void (*HANDLER)(uint8_t *buf);
typedef struct{
	char *str;
	HANDLER handler;
}HANDLER_TAB;

void configHandler(uint8_t *buf)
{
	int i= 0;
	HANDLER_TAB handler_tab[11] = {
			                      {"hello",    handler_1},
								  {"bye",      handler_2},
								  {"IP",       handler_3},
								  {"RS",       handler_3},
								  {"SET",      handler_3},
								  {"A",        handler_3},
								  {"mac",      handler_3},
			                      {"serverip", handler_3},
								  {"wifi_ip", handler_3},
								  {"wifi_ssid", handler_3},
								  {"sd2078rtc",handler_3}
	                             };
	for( i = 0; i < 11; i++)
	{
		if (!strncasecmp((char *)buf, handler_tab[i].str, strlen(handler_tab[i].str)))
		{
			handler_tab[i].handler(buf);
		}
	}
	bzero(config_buff,sizeof(config_buff));
}


void configRecv(uint8_t *ch, uint16_t len)
{
	if (ch[0] != '{')
	{
		printf("first byte err\r\n");
		return;
	}

	if (ch[1] != '"')
	{
		printf("first byte err\r\n");
		return;
	}
	if (ch[len-2] != '"')
	{
		printf("last byte:%c\r\n",ch[len-4]);
		printf("last byte err\r\n");
		return;
	}
	if (ch[len-1] != '}')
	{
		printf("last byte2:%c\r\n",ch[len-3]);
		printf("last byte2 err\r\n");
		return;
	}
	memcpy( config_buff, uart4_rx_buffer+2, len-4);
}



