/*
 * sd2078.h
 *
 *  Created on: Dec 10, 2021
 *      Author: Administrator
 */

#ifndef BOARDDRIVERS_SD2078_H_
#define BOARDDRIVERS_SD2078_H_


#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

//#define  SCL_H         GPIOB->BSRR = GPIO_Pin_6
//#define  SCL_L         GPIOB->BRR  = GPIO_Pin_6
#define  SCL_H         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET)
#define  SCL_L         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET)


//#define  SDA_H         GPIOB->BSRR = GPIO_Pin_7
//#define  SDA_L         GPIOB->BRR  = GPIO_Pin_7
#define  SDA_H         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET)
#define  SDA_L         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET)


#define SDA_read        (GPIOD->IDR  & GPIO_PIN_8)

enum Freq{F_0Hz, F32KHz, F4096Hz, F1024Hz, F64Hz, F32Hz, F16Hz, F8Hz, \
					F4Hz, F2Hz, F1Hz, F1_2Hz, F1_4Hz, F1_8Hz, F1_16Hz, F_1s};

enum clk_Souce{S_4096Hz, S_1024Hz, S_1s, S_1min};

/*此结构体定义了时间信息包括年、月、日、星期、时、分、秒*/
typedef	struct
{
	uint8_t	second;
	uint8_t	minute;
	uint8_t	hour;
	uint8_t	week;
	uint8_t	day;
	uint8_t	month;
	uint8_t	year;
} Time_Def;

/*此结构体定义了倒计时中断可供配置的频率源、IM和初值主要参数*/
typedef	struct
{
	enum clk_Souce d_clk;
	uint8_t   IM;	//IM=1:周期性中断
	uint32_t  init_val;
} CountDown_Def;

/*
 * 通过宏定义选择使用的RTC型号
 * SD30XX_25XX -> 使用SD30或SD25系列
 * SD20XX_24XX -> 使用SD20或SD24系列
 */
#define		SD30XX_25XX
#undef		SD20XX_24XX

/********************************************************/
#define		TRUE            1
#define		FALSE           0
#define		H               1
#define		L               0
#define		Chg_enable			0x82
#define		Chg_disable			0

/******************** Device Address ********************/
#define		RTC_Address     0x64
//                          01100100

/******************** Alarm register ********************/
#define		Alarm_SC				0x07
#define		Alarm_MN				0x08
#define		Alarm_HR				0x09
#define		Alarm_WK				0x0A
#define		Alarm_DY				0x0B
#define		Alarm_MO				0x0C
#define		Alarm_YR				0x0D
#define		Alarm_EN				0x0E

/******************** Control Register *******************/
#define		CTR1            0x0F
#define		CTR2            0x10
#define		CTR3            0x11

/***************** Timer Counter Register ****************/
#define		Timer_Counter1	0x13
#define		Timer_Counter2	0x14
#define		Timer_Counter3	0x15

/******************** Battery Register ********************/
#define		Chg_MG          0x18		//充电管理寄存器地址
#define		Bat_H8          0x1A		//电量最高位寄存器地址
#define		Bat_L8          0x1B		//电量低八位寄存器地址

/*********************** ID Register **********************/
#define		ID_Address			0x72		//ID号起始地址

/********************** 报警中断宏定义 *********************/
#define		sec_ALM					0x01
#define		min_ALM					0x02
#define		hor_ALM					0x04
#define		wek_ALM					0x08
#define		day_ALM					0x10
#define		mon_ALM					0x20
#define		yar_ALM					0x40

/********************** 中断使能宏定义 **********************/
#define		INTDE						0x04		//倒计时中断
#define		INTAE						0x02		//报警中断
#define		INTFE						0x01		//频率中断

/********************** 中断演示宏定义 **********************/
#define 	FREQUENCY				0				//频率中断
#define 	ALARM				    1				//报警中断
#define 	COUNTDOWN				2				//倒计时中断
#define 	DISABLE					3				//禁止中断

/*************** 中断输出类型选择，请自行选择 ****************/
#define 	INT_TYPE			COUNTDOWN //定义中断输出类型

/***********读写时间函数*************/
uint8_t RTC_WriteDate(Time_Def	*psRTC);
uint8_t RTC_ReadDate(Time_Def	*psRTC);

/*******I2C多字节连续读写函数********/
uint8_t I2CWriteSerial(uint8_t DeviceAddress,uint8_t Address,uint8_t length,uint8_t *ps);
uint8_t I2CReadSerial(uint8_t DeviceAddress,uint8_t Address,uint8_t length,uint8_t *ps);

/*********I2C端口初始化函数*********/


/*********RTC中断配置函数*********/
void Set_CountDown(CountDown_Def *CountDown_Init);
void Set_Alarm(uint8_t Enable_config, Time_Def *psRTC);
void SetFrq(enum Freq F_Out);
void ClrINT(uint8_t int_EN);


void USER_RTC_Init();

#endif /* BOARDDRIVERS_SD2078_H_ */
