/*
 * flash.h
 *
 *  Created on: Sep 24, 2021
 *      Author: Administrator
 */

#ifndef BOARDDRIVERS_FLASH_H_
#define BOARDDRIVERS_FLASH_H_

#include "main.h"
//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 		//STM32 FLASH的起始地址
//FLASH解锁键值
//#define FLASH_KEY1               0X45670123
//#define FLASH_KEY2               0XCDEF89AB

//STM32F4 Flash操作状态
typedef enum
{
	STM32FLASH_OK			=	0,	//操作完成
	STM32FLASH_BUSY			=	1,	//忙
	STM32FLASH_WPRERR		=	2,	//写保护错误
	STM32FLASH_PGAERR		=	3,	//编程对齐错误，必须128位对齐
	STM32FLASH_PGPERR		=	4,	//并行位数错误
	STM32FLASH_PGSERR		=	5,	//编程顺序错误
}STM32FLASH_STATUS;



//相关接口函数
void STM32FLASH_Unlock(void);															//FLASH解锁
void STM32FLASH_Lock(void);					  											//FLASH上锁
STM32FLASH_STATUS STM32FLASH_GetStatus(void);				  							//获得状态

STM32FLASH_STATUS STM32FLASH_EraseSector(uint8_t sectoraddr);			  					//擦除扇区


STM32FLASH_STATUS STM32FLASH_WriteWord(uint32_t faddr, uint32_t data);							//在FLASH指定地址写入一个字的数据（32bit写入） 【警告：地址必须是4的整数倍】

uint32_t STM32FLASH_ReadWord(uint32_t faddr);														//读取指定地址的一个字(32位数据) 【警告：地址必须是4的整数倍】

STM32FLASH_STATUS STM32FLASH_Write_NoCheck(uint32_t WriteAddr,uint32_t *pBuffer,uint16_t NumToWrite) ; //不检查的写入

STM32FLASH_STATUS STM32FLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint16_t NumToWrite);			//从指定地址开始写入指定长度的数据（数据长度字为单位）

uint16_t STM32FLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint16_t NumToRead);							//从指定地址开始读出指定长度的数据







#endif /* BOARDDRIVERS_FLASH_H_ */
