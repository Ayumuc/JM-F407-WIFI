/*
 * MCP3208.c
 *
 *  Created on: Sep 23, 2021
 *      Author: Administrator
 */


#include "stm32f4xx.h"
#include "delay.h"


#define MCP_CS_0   HAL_GPIO_WritePin( GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define MCP_CS_1   HAL_GPIO_WritePin( GPIOB, GPIO_PIN_1, GPIO_PIN_SET)

#define MCP_MISO_0 HAL_GPIO_WritePin( GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define MCP_MISO_1 HAL_GPIO_WritePin( GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

#define MCP_MISI   HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_6)

#define MCP_CLK_1	 HAL_GPIO_WritePin( GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define MCP_CLK_0	 HAL_GPIO_WritePin( GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

void initMCP(void)
{
	MCP_CS_1;
}

 void delay(unsigned int num)
 {
    int i;
    int cnt = 0;
	while(num--)
	{
		 for(i=0;i<10;i++)
		 {
			 cnt++;
			 if (cnt == 65535)
			 {
				 cnt = 0;
			 }
		 }
	}
 }

int mcpSample(unsigned char chx)
{
	unsigned char i;
	unsigned int x = 0;
	MCP_CS_1;
	for_delay_us(10);
	MCP_CLK_0;        // clk low first
    MCP_CS_0;         // chxip select
	MCP_MISO_0;

    MCP_MISO_1;
    MCP_CLK_1;
    for_delay_us(1);
    MCP_CLK_0;
    for_delay_us(1);     // start bit
    MCP_MISO_1;
    MCP_CLK_1;
    for_delay_us(1);
    MCP_CLK_0;
    for_delay_us(1);      // SGL/DIFF = 1

	if((chx&0x4)>>2==1)
	{
		MCP_MISO_1;
	}
	else
	{
		MCP_MISO_0;
	}

	MCP_CLK_1;
	for_delay_us(1);//delay_us(1);//delay(1);
	MCP_CLK_0;
	for_delay_us(1);//delay(1);      // D2

	if((chx&0x2)>>1 == 1)
	{
		MCP_MISO_1;
	}
	else
	{
		MCP_MISO_0;
	}

    MCP_CLK_1;
    for_delay_us(1);//delay(1);
    MCP_CLK_0;
    for_delay_us(1);//delay(1);      // D1

    if((chx&0x1) == 1)
    {
  	    MCP_MISO_1;
    }
	else
	{
		MCP_MISO_0;
	}

    MCP_CLK_1;
    for_delay_us(1);//delay(1);
    MCP_CLK_0;
    for_delay_us(1);//delay(1);      // D0
    for_delay_us(100);//delay(100);

    MCP_CLK_1;
    for_delay_us(1);//delay(1);
    MCP_CLK_0;
    for_delay_us(1);//delay(1);     // null
    for_delay_us(100);//delay(100);

	for (i=0;i<12;i++)  // 12 bit data
	{
		MCP_CLK_1;
		for_delay_us(1);//delay(1);
		MCP_CLK_0;
		for_delay_us(1);//delay(1);//up scl
		x <<= 1;
		x = x | MCP_MISI;
	}
	MCP_CS_1;
	return x;
}
/*--------------------------------------------------------------------
	函数名：滤波函数
	函数功能：读ad值并做滤波处理
	输入参数：待转换的通道号
	输出参数：转换后经滤波后的结果
---------------------------------------------------------------------*/
 /*unsigned int mcpdata[100];
 unsigned int mcpdata1[100];
 unsigned int mcpdata2[100];
 unsigned int mcpdata3[100];
 unsigned int mcpdata4[100];
unsigned int mcpdata5[100];
 unsigned int mcpdata6[100];
 unsigned int mcpdata7[100];

void mcp_String_bubble_sort(int size, unsigned int tmp[100])
{
    int adc1temp=0;
    int i=0;
    int j=0;

    if(0>=size)
         return;

    for(;i<size-1;++i)
    {
       for(j=0;j<size-i-1;++j)
       {
					//传感器数值
					if(tmp[j]>tmp[j+1])
					{
						adc1temp=tmp[j];
						tmp[j]=tmp[j+1];
						tmp[j+1]=adc1temp;
					}
       }
    }
 }

int mcp_DealBigAndSmallData(int size,unsigned int tmp[100])
{
	int i=0;
	long adctemp=0;
	unsigned int  tempavg=0;

	//采样数值排序
	mcp_String_bubble_sort(size,tmp);

	//中间值做为基础平方
	for(i=size/2-5;i<size/2+5;i++)
	{
		adctemp+=tmp[i]*tmp[i];
	}
    tempavg = (unsigned int)sqrt(adctemp/10);


	return 	tempavg;
}

#define MAX_ADV 50
unsigned int adtmp=0,adtmp1=0,adtmp2=0,adtmp3=0,adtmp4=0,adtmp5=0,adtmp6=0,adtmp7=0;
unsigned int Avgadc=0;
void MCP3208_Filter(unsigned char chal)
{
    unsigned char i=0;
    unsigned char str1[200];

    for(i=0;i<MAX_ADV;i++)
    {
         mcpdata[i]= mcpSample(0x00);
	     mcpdata1[i]= mcpSample(0x01);
	     mcpdata2[i]= mcpSample(0x02);
    }

   adtmp =mcp_DealBigAndSmallData(MAX_ADV,mcpdata)*1.3870;

}*/
