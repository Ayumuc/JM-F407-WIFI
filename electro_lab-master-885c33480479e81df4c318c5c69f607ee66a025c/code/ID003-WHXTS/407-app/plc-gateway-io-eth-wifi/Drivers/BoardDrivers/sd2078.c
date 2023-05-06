/*
 * sd2078.c
 *
 *  Created on: Dec 10, 2021
 *      Author: Administrator
 */


/**
  ******************************************************************************
  * @file    rtc.c
  * @author  FAE
  * @version V1.0
  * @date    2018-04-10
  * @brief   i2c RTC应用函数
  ******************************************************************************
  * @attention
  *
  * Website: http://www.whwave.com.cn
  * E-mail : fae@whwave.com.cn
  * Tel    : 0755-83114387
  *
  ******************************************************************************
  */

#include "sd2078.h"
#include "delay.h"
#include "rtc.h"
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define WAITEDELAY  50

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configure the used I/O ports pin
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void GPIO_Configuration(void)
{
	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_9 | GPIO_PIN_8);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/*********************************************
 * 函数名：I2Cdelay
 * 描  述：I2C延时函数
 * 输  入：无
 * 输  出：无
 ********************************************/
static void I2Cdelay(void)
{
//   uint8_t i=WAITEDELAY; //这里可以优化速度，通常延时3~10us，可以用示波器看波形来调试
//   while(i)
//   {
//     i--;
//   }
	for_delay_us(3);
}

/*********************************************
 * 函数名：IIC_Init
 * 描  述：I2C端口初始化
 * 输  入：无
 * 输  出：无
 ********************************************/
static void IIC_Init(void)
{
  /* GPIO configuration */
  GPIO_Configuration();  //第二次初始化会不会有问题
}

/*********************************************
 * 函数名：I2CStart
 * 描  述：开启I2C总线
 * 输  入：无
 * 输  出：TRUE:操作成功，FALSE:操作失败
 ********************************************/
static uint8_t I2CStart(void)
{
    SDA_H;
	I2Cdelay();
    SCL_H;
    I2Cdelay();
    if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
    SDA_L;
    I2Cdelay();
    SCL_L;
    I2Cdelay();
    return TRUE;
}

/*********************************************
 * 函数名：I2CStop
 * 描  述：释放I2C总线
 * 输  入：无
 * 输  出：无
 ********************************************/
static void I2CStop(void)
{
    SCL_L;
    I2Cdelay();
    SDA_L;
    I2Cdelay();
    SCL_H;
    I2Cdelay();
    SDA_H;
    I2Cdelay();
}

/*********************************************
 * 函数名：I2CAck
 * 描  述：发送ASK
 * 输  入：无
 * 输  出：无
 ********************************************/
static void I2CAck(void)
{
    SCL_L;
    I2Cdelay();
    SDA_L;
    I2Cdelay();
    SCL_H;
    I2Cdelay();
    SCL_L;
    I2Cdelay();
}

/*********************************************
 * 函数名：I2CNoAck
 * 描  述：发送NOASK
 * 输  入：无
 * 输  出：无
 ********************************************/
static void I2CNoAck(void)
{
    SCL_L;
    I2Cdelay();
    SDA_H;
    I2Cdelay();
    SCL_H;
    I2Cdelay();
    SCL_L;
    I2Cdelay();
}

/*********************************************
 * 函数名：I2CWaitAck
 * 描  述：读取ACK信号
 * 输  入：无
 * 输  出：TRUE=有ACK,FALSE=无ACK
 ********************************************/
static uint8_t I2CWaitAck(void)
{
    SCL_L;
    I2Cdelay();
    SDA_H;
    I2Cdelay();
    SCL_H;
    I2Cdelay();
    if(SDA_read)
    {
		SCL_L;
		return FALSE;
    }
    SCL_L;
    return TRUE;
}

/*********************************************
 * 函数名：I2CSendByte
 * 描  述：MCU发送一个字节
 * 输  入：无
 * 输  出：无
 ********************************************/
static void I2CSendByte(uint8_t SendByte) //数据从高位到低位
{
    uint8_t i=8;
		while(i--)
		{
			SCL_L;
			I2Cdelay();
			if(SendByte&0x80)
			SDA_H;
			else
			SDA_L;
			SendByte<<=1;
			I2Cdelay();
			SCL_H;
			I2Cdelay();
		}
    SCL_L;
}

/*********************************************
 * 函数名：I2CReceiveByte
 * 描  述：MCU读入一个字节
 * 输  入：无
 * 输  出：ReceiveByte
 ********************************************/
static uint8_t I2CReceiveByte(void)
{
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H;
    while(i--)
    {
      ReceiveByte<<=1;
      SCL_L;
      I2Cdelay();
      SCL_H;
      I2Cdelay();
      if(SDA_read)
      {
        ReceiveByte|=0x1;
      }
    }
    SCL_L;
    return ReceiveByte;
}

/*********************************************
 * 函数名：WriteRTC_Enable
 * 描  述：RTC写允许程序
 * 输  入：无
 * 输  出：TRUE:操作成功，FALSE:操作失败
 ********************************************/
static uint8_t WriteRTC_Enable(void)
{
    if(FALSE == I2CStart())
      return FALSE;

    I2CSendByte(RTC_Address);

    if(I2CWaitAck()== FALSE)
    {
      I2CStop();
      return FALSE;
    }

    I2CSendByte(CTR2);

    I2CWaitAck();

    I2CSendByte(0x80);//置WRTC1=1

    I2CWaitAck();

    I2CStop();

    I2CStart();

    I2CSendByte(RTC_Address);

    I2CWaitAck();

    I2CSendByte(CTR1);

    I2CWaitAck();

    I2CSendByte(0x84);//置WRTC2,WRTC3=1

    I2CWaitAck();

    I2CStop();

    return TRUE;
}

/*********************************************
 * 函数名：WriteRTC_Disable
 * 描  述：RTC写禁止程序
 * 输  入：无
 * 输  出：TRUE:操作成功，FALSE:操作失败
 ********************************************/
static uint8_t WriteRTC_Disable(void)
{
    if(FALSE == I2CStart()) return FALSE;
    I2CSendByte(RTC_Address);
    if(!I2CWaitAck()){I2CStop(); return FALSE;}
    I2CSendByte(CTR1);//设置写地址0FH
    I2CWaitAck();
    I2CSendByte(0x0) ;//置WRTC2,WRTC3=0
    I2CWaitAck();
    I2CSendByte(0x0) ;//置WRTC1=0(10H地址)
    I2CWaitAck();
    I2CStop();
    return TRUE;
}

/*********************************************
 * 函数名：RTC_WriteDate
 * 描  述：写RTC实时数据寄存器
 * 输  入：时间结构体指针
 * 输  出：TRUE:操作成功，FALSE:操作失败
 ********************************************/
uint8_t RTC_WriteDate(Time_Def	*psRTC)	//写时间操作要求一次对实时时间寄存器(00H~06H)依次写入，
{                               //不可以单独对7个时间数据中的某一位进行写操作,否则可能会引起时间数据的错误进位.
                                //要修改其中某一个数据 , 应一次性写入全部 7 个实时时钟数据.

	WriteRTC_Enable();				//使能，开锁

	I2CStart();
	I2CSendByte(RTC_Address);
	if(!I2CWaitAck()){I2CStop(); return FALSE;}
	I2CSendByte(0);			//设置写起始地址
	I2CWaitAck();
	I2CSendByte(psRTC->second);		//second
	I2CWaitAck();
	I2CSendByte(psRTC->minute);		//minute
	I2CWaitAck();
	I2CSendByte(psRTC->hour|0x80);//hour ,同时设置小时寄存器最高位（0：为12小时制，1：为24小时制）
	I2CWaitAck();
	I2CSendByte(psRTC->week);		//week
	I2CWaitAck();
	I2CSendByte(psRTC->day);		//day
	I2CWaitAck();
	I2CSendByte(psRTC->month);		//month
	I2CWaitAck();
	I2CSendByte(psRTC->year);		//year
	I2CWaitAck();
	I2CStop();

	WriteRTC_Disable();				//关锁
	return	TRUE;
}

/*********************************************
 * 函数名：RTC_ReadDate
 * 描  述：读RTC实时数据寄存器
 * 输  入：时间结构体指针
 * 输  出：TRUE:操作成功，FALSE:操作失败
 ********************************************/
uint8_t RTC_ReadDate(Time_Def	*psRTC)
{
	I2CStart();

	I2CSendByte(RTC_Address);

	if (!I2CWaitAck())
    {
		I2CStop();
		return FALSE;
    }

    I2CSendByte(0);

    I2CWaitAck();

    I2CStart();

    I2CSendByte(RTC_Address+1);

    I2CWaitAck();

    psRTC->second=I2CReceiveByte();

    I2CAck();

    psRTC->minute=I2CReceiveByte();

  	I2CAck();

  	psRTC->hour=I2CReceiveByte() & 0x7F;

  	I2CAck();

  	psRTC->week=I2CReceiveByte();

  	I2CAck();

  	psRTC->day=I2CReceiveByte();

  	I2CAck();

  	psRTC->month=I2CReceiveByte();

  	I2CAck();

  	psRTC->year=I2CReceiveByte();

  	I2CNoAck();

  	I2CStop();

  	return	TRUE;
}
/*********************************************
 * 函数名     ：I2CWriteSerial
 * 描  述     ：I2C在指定地址写一字节数据
 * Device_Addr：I2C设备地址
 * Address    ：内部地址
 * length     ：字节长度
 * ps         ：缓存区指针
 * 输出       ：TRUE 成功，FALSE 失败
 ********************************************/
uint8_t I2CWriteSerial(uint8_t DeviceAddress, uint8_t Address, uint8_t length, uint8_t *ps)
{
	if (DeviceAddress == RTC_Address)
		WriteRTC_Enable();

	I2CStart();

    I2CSendByte(DeviceAddress);

    if (!I2CWaitAck())
    {
      I2CStop();
      return FALSE;
    }

    I2CSendByte(Address);

    I2CWaitAck();

    for(;(length--)>0;)
	{
		I2CSendByte(*(ps++));
		I2CAck();
	}

    I2CStop();

    if (DeviceAddress == RTC_Address)
    	WriteRTC_Disable();

    return	TRUE;
}

/*********************************************
 * 函数名     ：I2CReadSerial
 * 描  述     ：I2C在指定地址写一字节数据
 * Device_Addr：I2C设备地址
 * Address    ：内部地址
 * length     ：字节长度
 * ps         ：缓存区指针
 * 输出       ：TRUE 成功，FALSE 失败
 ********************************************/
uint8_t I2CReadSerial(uint8_t DeviceAddress, uint8_t Address, uint8_t length, uint8_t *ps)
{
	I2CStart();

	I2CSendByte(DeviceAddress);

    if (!I2CWaitAck())
    {
      I2CStop();
      return FALSE;
    }

	I2CSendByte(Address);

    I2CWaitAck();

    I2CStart();

    I2CSendByte(DeviceAddress+1);

    I2CWaitAck();

    for(;--length>0;ps++)
	{
		*ps = I2CReceiveByte();
		I2CAck();
	}

    *ps = I2CReceiveByte();

    I2CNoAck();

    I2CStop();

    return	TRUE;
}

/*********************************************
 * 函数名：Set_CountDown
 * 描  述：设置倒计时中断
 * 输  入：CountDown_Init 倒计时中断结构体指针
 * 输  出：无
 ********************************************/
void Set_CountDown(CountDown_Def *CountDown_Init)
{
	uint8_t buf[6];
	uint8_t tem=0xF0;
	buf[0] = (CountDown_Init->IM<<6)|0xB4;							//10H
	buf[1] = CountDown_Init->d_clk<<4;									//11H
	buf[2] = 0;																					//12H
	buf[3] = CountDown_Init->init_val & 0x0000FF;				//13H
	buf[4] = (CountDown_Init->init_val & 0x00FF00) >> 8;//14H
	buf[5] = (CountDown_Init->init_val & 0xFF0000) >> 16;//15H
	I2CWriteSerial(RTC_Address,CTR2,1,&tem);
	I2CWriteSerial(RTC_Address,CTR2,6,buf);
}

/*********************************************
 * 函数名：Set_Alarm
 * 描  述：设置报警中断（闹钟功能）
 * Enable_config：使能设置
 * psRTC：报警时间的时间结构体指针
 * 输  出：无
 ********************************************/
void Set_Alarm(uint8_t Enable_config, Time_Def *psRTC)
{
	uint8_t buf[10];
	buf[0] = psRTC->second;
	buf[1] = psRTC->minute;
	buf[2] = psRTC->hour;
	buf[3] = 0;
	buf[4] = psRTC->day;
	buf[5] = psRTC->month;
	buf[6] = psRTC->year;
	buf[7] = Enable_config;
	buf[8] = 0xFF;
	buf[9] = 0x92;
	I2CWriteSerial(RTC_Address,Alarm_SC,10,buf);
}

/*********************************************
 * 函数名：SetFrq
 * 描  述：设置RTC频率中断，从INT脚输出频率方波
 * 输  入：频率值
 * 输  出：无
 ********************************************/
void SetFrq(enum Freq F_Out)
{
	uint8_t buf[2];
	buf[0] = 0xA1;
	buf[1] = F_Out;
	I2CWriteSerial(RTC_Address,CTR2,2,buf);
}

/*********************************************
 * 函数名：ClrINT
 * 描  述：禁止中断
 * int_EN：中断类型 INTDE、INTDE、INTDE
 * 输  出：无
 ********************************************/
void ClrINT(uint8_t int_EN)
{
	uint8_t buf;
	buf = 0x80 & (~int_EN);
	I2CWriteSerial(RTC_Address,CTR2,1,&buf);
}


static void initSD2078(void)
{
	IIC_Init();
}


void USER_RTC_Init(void)
{
	uint8_t ret = -1;
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    RTC_TimeTypeDef gTime = {0};
	RTC_DateTypeDef gDate = {0};
	Time_Def sysTime;

	initSD2078(); //初始化iic


	//init stm32 rtc
	RTC_ReadDate(&sysTime);
	sTime.Hours = sysTime.hour;
	sTime.Minutes = sysTime.minute;
	sTime.Seconds = sysTime.second;
	ret = HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);  //设置 stm32 的时间
	if (ret != HAL_OK)
	{
		#if LOG_PRINTF_DEF
		LogPrint("HAL_RTC_SetTime error");
		#else
		printf("HAL_RTC_SetTime error\r\n");
		#endif
	}

	sDate.Year = sysTime.year;
	sDate.Month = sysTime.month;
	sDate.Date = sysTime.day;
	sDate.WeekDay = sysTime.week;
	ret = HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);  //设置 stm32 的日期
	if (ret != HAL_OK)
	{
		#if LOG_PRINTF_DEF
		LogPrint("HAL_RTC_SetDate error");
		#else
		printf("HAL_RTC_SetDate error\r\n");
		#endif
	}


//	RTC_ReadDate(&sysTime);



	printf("SD2078 RTC:%02X-%02X-%02X  %02X:%02X:%02X  week:%02X\r\n", \
																	sysTime.year, sysTime.month, sysTime.day, sysTime.hour,\
																	sysTime.minute, sysTime.second, sysTime.week);

	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BCD);


	printf("STM32 RTC:%02X-%02X-%02X  %02X:%02X:%02X:%02d  week:%02X\r\n", \
																	gDate.Year, gDate.Month, gDate.Date, gTime.Hours,\
																	gTime.Minutes,gTime.Seconds, (int)gTime.SubSeconds,gDate.WeekDay);

}
/*********************************************END OF FILE**********************/
