/*
 * oled.c
 *
 *  Created on: Sep 23, 2021
 *      Author: Administrator
 */


#include "stm32f4xx.h"
#include "stdlib.h"
#include "oled.h"
#include "oledLib.h"

/************************************************************************
     OLED 一行可以显示16个字符，可以显示8行。
************************************************************************/
#define OLED_DC_PIN  GPIO_PIN_14
#define OLED_RST_PIN GPIO_PIN_15
#define OLED_SDA_PIN GPIO_PIN_8
#define OLED_SCL_PIN GPIO_PIN_9



#define OLED_SCL_HIGH    HAL_GPIO_WritePin(GPIOD, OLED_SCL_PIN, GPIO_PIN_SET)
#define OLED_SCL_LOW     HAL_GPIO_WritePin(GPIOD, OLED_SCL_PIN, GPIO_PIN_RESET)
#define OLED_SDA_HIGH    HAL_GPIO_WritePin(GPIOD, OLED_SDA_PIN, GPIO_PIN_SET)
#define OLED_SDA_LOW     HAL_GPIO_WritePin(GPIOD, OLED_SDA_PIN, GPIO_PIN_RESET)


#define OLED_RST_HIGH    HAL_GPIO_WritePin(GPIOB, OLED_RST_PIN, GPIO_PIN_SET)
#define OLED_RST_LOW     HAL_GPIO_WritePin(GPIOB, OLED_RST_PIN, GPIO_PIN_RESET)
#define OLED_DC_HIGH     HAL_GPIO_WritePin(GPIOB, OLED_DC_PIN,  GPIO_PIN_SET)
#define OLED_DC_LOW      HAL_GPIO_WritePin(GPIOB, OLED_DC_PIN,  GPIO_PIN_RESET)



#define X_WIDTH 128
#define Y_WIDTH 64

/*
4线SPI使用说明：
VBT 供内部DC-DC电压，3.3~4.3V，如果使用5V电压，为保险起见串一个100~500欧的电阻
VCC 供内部逻辑电压 1.8~6V
GND 地

BS0 低电平
BS1 低电平
BS2 低电平

CS  片选管脚
DC  命令数据选择管脚
RES 模块复位管脚
D0（SCLK） ，时钟脚，由MCU控制
D1（MOSI） ，主输出从输入数据脚，由MCU控制

D2 悬空
D3-D7 ， 低电平 ， 也可悬空，但最好设为低电平
RD  低电平 ，也可悬空，但最好设为低电平
RW  低电平 ，也可悬空，但最好设为低电平
RD  低电平 ，也可悬空，但最好设为低电平
*/

void OLED_WrDat(unsigned char data)
{
	unsigned char i=8;
	OLED_DC_HIGH;

    OLED_SCL_LOW;
  while(i--)
  {
    if(data&0x80){OLED_SDA_HIGH;}
    else{OLED_SDA_LOW;}
    OLED_SCL_HIGH;
    //OLED_DLY_us(2);
    OLED_SCL_LOW;
    data<<=1;
  }
}

void OLED_WrCmd(unsigned char cmd)
{
  unsigned char i=8;
  OLED_DC_LOW;

  OLED_SCL_LOW;

  while(i--)
  {
    if(cmd&0x80){OLED_SDA_HIGH;}
    else{OLED_SDA_LOW;}
    OLED_SCL_HIGH;
    //OLED_DLY_us(2);

    OLED_SCL_LOW;
    cmd<<=1;
  }

}

void OLED_Set_Pos(unsigned char x, unsigned char y)
{
  OLED_WrCmd(0xb0+y);
  OLED_WrCmd(((x&0xf0)>>4)|0x10);
  OLED_WrCmd((x&0x0f)|0x01);
}

void OLED_Fill(unsigned char bmp_data)
{
	unsigned char y,x;

	for(y=0;y<8;y++)
	{
		OLED_WrCmd(0xb0+y);
		OLED_WrCmd(0x01);
		OLED_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
			OLED_WrDat(bmp_data);
	}
}

void OLED_DLY_ms(unsigned short ms)
{
	unsigned short a;
	unsigned char cnt = 0;
	while(ms)
	{
		a=16000;
		while(a--)
		{
			cnt++;
		}
		ms--;
	}
}

void initOled(void)
{
  OLED_SCL_HIGH;
  OLED_RST_LOW;
  OLED_DLY_ms(50);
  OLED_RST_HIGH;
}

void init_iic(void)
{
  OLED_SCL_HIGH;
  OLED_RST_LOW;
  OLED_DLY_ms(50);
  OLED_RST_HIGH;
}

void setContrast(unsigned char contrast)
{
	OLED_WrCmd(0x81);//--set contrast control register
	OLED_WrCmd(contrast);
}

//==============================================================
//函数名：LCD_P8x16Str(u8 x,u8 y,u8 *p)
//功能描述：写入一组标准ASCII字符串
//参数：显示的位置（x,y），y为页范围0～7，要显示的字符串
//返回：无
//==============================================================
void OLED_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[],unsigned char state)
{
  unsigned char c=0,i=0,j=0;
  while (ch[j]!='\0')
  {
    if(state==0)
    {
      c =ch[j]-32;
      if(x>120)
      {
      	x=0;y+=2;
      }
      OLED_Set_Pos(x,y);
      for(i=0;i<8;i++)
        OLED_WrDat(F8X16[c*16+i]);
      OLED_Set_Pos(x,y+1);
      for(i=0;i<8;i++)
        OLED_WrDat(F8X16[c*16+i+8]);
      x+=8;
      j++;
    }
    else
    {
      c =ch[j]-32;
      if(x>120){x=0;y+=2;}
      OLED_Set_Pos(x,y);
      for(i=0;i<8;i++)
       OLED_WrDat(~(F8X16[c*16+i]));
      OLED_Set_Pos(x,y+1);
      for(i=0;i<8;i++)
  	   OLED_WrDat(~(F8X16[c*16+i+8]));
      x+=8;
      j++;
    }
  }
}

