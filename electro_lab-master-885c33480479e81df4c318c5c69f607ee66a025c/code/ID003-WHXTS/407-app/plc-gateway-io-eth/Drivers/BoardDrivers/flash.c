/*
 * flash.c
 *
 *  Created on: Sep 24, 2021
 *      Author: Administrator
 */

//#include "stm32f4xx_flash.h"

#include "flash.h"



//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	//扇区0起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 	//扇区1起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 	//扇区2起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 	//扇区3起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 	//扇区4起始地址, 64 Kbytes
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 	//扇区5起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 	//扇区6起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//扇区7起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 	//扇区8起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) 	//扇区9起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) 	//扇区10起始地址,128 Kbytes
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) 	//扇区11起始地址,128 Kbytes


void Delay_US(unsigned int dly)
{
	int i;
	while(dly--)
	{
		i=2000;
		while(i--);
	}
}


/*************************************************************************************************************************
* 函数			:	void STM32FLASH_Unlock(void)
* 功能			:	解锁STM32的FLASH
* 参数			:	无
* 返回			:	无
* 依赖			:	底层
* 作者			:	cp1300@139.com
* 时间			:	2013-10-20
* 最后修改时间 	: 	2013-10-20
* 说明			:
*************************************************************************************************************************/
void STM32FLASH_Unlock(void)
{
	FLASH->KEYR=FLASH_KEY1;	//写入解锁序列.
	FLASH->KEYR=FLASH_KEY2;
}



/*************************************************************************************************************************
* 函数			:	void STM32FLASH_Lock(void)
* 功能			:	上锁STM32的FLASH
* 参数			:	无
* 返回			:	无
* 依赖			:	底层
* 作者			:	cp1300@139.com
* 时间			:	2013-10-20
* 最后修改时间 	: 	2018-01-13
* 说明			:
*************************************************************************************************************************/
void STM32FLASH_Lock(void)
{
	FLASH->CR|=(uint32_t)1<<31;			//上锁
}




/*************************************************************************************************************************
* 函数			:	STM32FLASH_STATUS STM32FLASH_GetStatus(void)
* 功能			:	得到FLASH状态
* 参数			:	无
* 返回			:	STM32FLASH_STATUS
* 依赖			:	底层
* 作者			:	cp1300@139.com
* 时间			:	2013-10-20
* 最后修改时间 	: 	2018-01-03
* 说明			:
*************************************************************************************************************************/
STM32FLASH_STATUS STM32FLASH_GetStatus(void)
{
	uint32_t temp=FLASH->SR;

	if(temp&(1<<16))return STM32FLASH_BUSY;			//忙
	else if(temp&(1<<4))return STM32FLASH_WPRERR;	//写保护错误
	else if(temp&(1<<5))return STM32FLASH_PGAERR;	//编程对齐错误
	else if(temp&(1<<6))return STM32FLASH_PGPERR;	//并行位数错误
	else if(temp&(1<<7))return STM32FLASH_PGSERR;	//编程顺序错误
	return STM32FLASH_OK;							//操作完成
}



/*************************************************************************************************************************
* 函数			:	STM32FLASH_STATUS STM32FLASH_WaitDone(uint32_t time)
* 功能			:	等待操作完成
* 参数			:	time:要延时的长短,单位us
* 返回			:	STM32FLASH_STATUS
* 依赖			:	底层
* 作者			:	cp1300@139.com
* 时间			:	2013-10-20
* 最后修改时间 	: 	2018-01-03
* 说明			:
*************************************************************************************************************************/
STM32FLASH_STATUS STM32FLASH_WaitDone(uint32_t time)
{
	STM32FLASH_STATUS status;

	if(time < 1) time = 1;
	do
	{
		status=STM32FLASH_GetStatus();			//获取状态
		if(status!=STM32FLASH_BUSY)break;		//非忙,无需等待了,直接退出.
		Delay_US(1);
		time--;
	 }while(time);

	 return status;
}



/*************************************************************************************************************************
* 函数			:	STM32FLASH_STATUS STM32FLASH_EraseSector(uint8_t sectoraddr)
* 功能			:	扇区擦除
* 参数			:	sectoraddr：扇区地址，0-11
* 返回			:	STM32FLASH_STATUS
* 依赖			:	底层
* 作者			:	cp1300@139.com
* 时间			:	2018-01-03
* 最后修改时间 	: 	2018-01-03
* 说明			: 	扇区大小：0~3：16KB扇区;4：64KB扇区;5~11：128KB扇区
					128KB扇区擦除足最大2秒
					2018-01-16:修复扇区擦除时没有解锁问题
*************************************************************************************************************************/
STM32FLASH_STATUS STM32FLASH_EraseSector(uint8_t sectoraddr)
{
	STM32FLASH_STATUS status;

	status=STM32FLASH_WaitDone(200000);		//等待上次操作完成，最大时间2秒，如果已经操作完成了会直接跳过

	__disable_irq();
	STM32FLASH_Unlock();					//解锁
	if(status==STM32FLASH_OK)				//没有错误
	{
		FLASH->CR&=~(3<<8);					//清除PSIZE原来的设置
		FLASH->CR|=2<<8;					//设置为32bit宽,确保VCC=2.7~3.6V之间!!
		FLASH->CR&=~(0X1F<<3);				//清除原来的设置
		FLASH->CR|=sectoraddr<<3;			//设置要擦除的扇区
		FLASH->CR|=1<<1;					//扇区擦除
		FLASH->CR|=1<<16;					//开始擦除
		status=STM32FLASH_WaitDone(200000);	//等待操作结束,最大2s
		if(status!=STM32FLASH_BUSY)			//非忙
		{
			FLASH->CR&=~(1<<1);				//清除扇区擦除标志.
		}
	}
	STM32FLASH_Lock();						//上锁
	__enable_irq();

	return status;							//返回状态
}




/*************************************************************************************************************************
* 函数			:	STM32FLASH_STATUS STM32FLASH_WriteWord(uint32_t addr, uint32_t data)
* 功能			:	在FLASH指定地址写入一个字的数据（32bit写入）
* 参数			:	faddr:指定地址(此地址必须为4的倍数!!);data:要写入的数据
* 返回			:	STM32FLASH_STATUS
* 依赖			:	底层
* 作者			:	cp1300@139.com
* 时间			:	2018-01-03
* 最后修改时间 	: 	2018-01-03
* 说明			: 	警告，地址必须为4的倍数
*************************************************************************************************************************/
STM32FLASH_STATUS STM32FLASH_WriteWord(uint32_t faddr, uint32_t data)
{
	STM32FLASH_STATUS status;

	status=STM32FLASH_WaitDone(200);	 	//判断状态
	if(status==STM32FLASH_OK)				//没有错误
	{
		FLASH->CR&=~(3<<8);					//清除PSIZE原来的设置
		FLASH->CR|=2<<8;					//设置为32bit宽,确保VCC=2.7~3.6V之间!!
 		FLASH->CR|=1<<0;					//编程使能
		*(__IO uint32_t*)faddr=data;					//写入数据
		status=STM32FLASH_WaitDone(200);	//等待操作完成,一个字编程,最多100us.
		if(status!=STM32FLASH_BUSY)			//非忙
		{
			FLASH->CR&=~(1<<0);				//清除PG位.
		}
	}
	return status;
}



/*************************************************************************************************************************
* 函数			:	uint32_t STM32FLASH_ReadWord(uint32_t faddr)
* 功能			:	读取指定地址的一个字(32位数据)
* 参数			:	faddr:指定地址(此地址必须为4的倍数!!);
* 返回			:	数据
* 依赖			:	底层
* 作者			:	cp1300@139.com
* 时间			:	2018-01-03
* 最后修改时间 	: 	2018-01-03
* 说明			: 	警告：地址必须是4的整数倍
*************************************************************************************************************************/
uint32_t STM32FLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t*)faddr;
}


/*************************************************************************************************************************
* 函数			:	uint8_t STM32FLASH_GetFlashSector(uint32_t faddr)
* 功能			:	获取某个地址所在的扇区
* 参数			:	faddr:指定地址
* 返回			:	删除编号，0-11，如果超出了也会限制到11
* 依赖			:	底层
* 作者			:	cp1300@139.com
* 时间			:	2018-01-03
* 最后修改时间 	: 	2018-01-03
* 说明			: 	地址请不要超出范围
*************************************************************************************************************************/
uint8_t STM32FLASH_GetFlashSector(uint32_t faddr)
{
	if(faddr<ADDR_FLASH_SECTOR_1)		return 0;
	else if(faddr<ADDR_FLASH_SECTOR_2)	return 1;
	else if(faddr<ADDR_FLASH_SECTOR_3)	return 2;
	else if(faddr<ADDR_FLASH_SECTOR_4)	return 3;
	else if(faddr<ADDR_FLASH_SECTOR_5)	return 4;
	else if(faddr<ADDR_FLASH_SECTOR_6)	return 5;
	else if(faddr<ADDR_FLASH_SECTOR_7)	return 6;
	else if(faddr<ADDR_FLASH_SECTOR_8)	return 7;
	else if(faddr<ADDR_FLASH_SECTOR_9)	return 8;
	else if(faddr<ADDR_FLASH_SECTOR_10)	return 9;
	else if(faddr<ADDR_FLASH_SECTOR_11)	return 10;
	else return 11;
}


/*************************************************************************************************************************
* 函数			:	STM32FLASH_STATUS STM32FLASH_Write_NoCheck(uint32_t WriteAddr,uint32_t *pBuffer,uint16_t NumToWrite)
* 功能			:	不检查的写入
* 参数			:	WriteAddr:起始地址(必须是4个整数倍);pBuffer:数据指针;NumToWrite:半(32位)数
* 返回			:	状态
* 依赖			:	底层
* 作者			:	cp1300@139.com
* 时间			:	2013-10-20
* 最后修改时间 	: 	2018-01-03
* 说明			: 	地址必须为4对齐，如果出现了非FF写入程序会报错并返回（由于STM32硬件会对写入区域进行检查，非FF区域不允许写入）
*************************************************************************************************************************/
STM32FLASH_STATUS STM32FLASH_Write_NoCheck(uint32_t WriteAddr,uint32_t *pBuffer,uint16_t NumToWrite)
{
	uint16_t i;
	STM32FLASH_STATUS status;

	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return STM32FLASH_PGPERR ;	//非法地址,如果不是4的倍数,返回并行错误

	__disable_irq();
	STM32FLASH_Unlock();						//解锁

	for(i=0;i<NumToWrite;i++)
	{
		status = STM32FLASH_WriteWord(WriteAddr,pBuffer[i]);
		if(status != STM32FLASH_OK) break;		//写入出错，直接返回
	    WriteAddr+=4;							//地址增加4.
	}

	STM32FLASH_Lock();							//上锁
	__enable_irq();
	return status;
}




/*************************************************************************************************************************
* 函数			:	STM32FLASH_STATUS STM32FLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint16_t NumToWrite)
* 功能			:	从指定地址开始写入指定长度的数据（数据长度字为单位）
* 参数			:	WriteAddr:起始地址(此地址必须为4的倍数!!);pBuffer:数据指针;NumToWrite:字(32位)数(就是要写入的32位数据的个数.)
* 返回			:	0:完成;1:忙;2:错误;3:写保护
* 依赖			:	底层
* 作者			:	cp1300@139.com
* 时间			:	2013-10-20
* 最后修改时间 	: 	2016-01-07
* 说明			: 	警告：由于STM32F4的扇区太大，无法使用内存进行缓存，如果要写入的位置有非FF，会直接将整个扇区进行擦除，并丢失其它的数据
*************************************************************************************************************************/
STM32FLASH_STATUS STM32FLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint16_t NumToWrite)
{
	STM32FLASH_STATUS status=STM32FLASH_OK;
	uint32_t addrx=0;
	uint32_t endaddr=0;

	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return STM32FLASH_PGPERR;	//非法地址

 	FLASH->ACR&=~(1<<10);								//FLASH擦除期间,必须禁止数据缓存!!!搞了我两晚上才发现这个问题!
	addrx=WriteAddr;									//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;						//写入的结束地址

	if(addrx<0X1FFF0000)								//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)							//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STM32FLASH_ReadWord(addrx)!=0XFFFFFFFF)	//有非0XFFFFFFFF的地方,要擦除这个扇区
			{
				status=STM32FLASH_EraseSector(STM32FLASH_GetFlashSector(addrx));
				if(status != STM32FLASH_OK)break;		//发生错误了
			}else addrx+=4;
		}
	}
	if(status==STM32FLASH_OK)
	{
		Delay_US(1000);

		__disable_irq();
		STM32FLASH_Unlock();							//解锁
		while(WriteAddr<endaddr)						//写数据
		{
			status = STM32FLASH_WriteWord(WriteAddr,*pBuffer);
			if(status != STM32FLASH_OK) break;			//写入出错，直接返回
			WriteAddr+=4;
			pBuffer++;
		}
		STM32FLASH_Lock();								//上锁
		__enable_irq();

	}
	FLASH->ACR|=1<<10;									//FLASH擦除结束,开启数据fetch
	return status;
}





/*************************************************************************************************************************
* 函数			:	uint16_t STM32FLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint16_t NumToRead)
* 功能			:	从指定地址开始读出指定长度的数据
* 参数			:	ReadAddr:起始地址;pBuffer:数据指针;NumToWrite:字(32位)数
* 返回			:	读取的数据长度（字为单位）
* 依赖			:	底层
* 作者			:	cp1300@139.com
* 时间			:	2013-10-20
* 最后修改时间 	: 	2016-01-07
* 说明			: 	地址必须为4对齐
*************************************************************************************************************************/
uint16_t STM32FLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint16_t NumToRead)
{
	uint16_t i;
	uint16_t cnt = 0;

	if(ReadAddr<STM32_FLASH_BASE||ReadAddr%4)return 0;	//非法地址

	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STM32FLASH_ReadWord(ReadAddr);		//读取4个字节.
		cnt++;
		ReadAddr+=4;									//偏移4个字节.
	}
	return cnt;
}

