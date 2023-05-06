/*
 * oled.h
 *
 *  Created on: Sep 23, 2021
 *      Author: Administrator
 */

#ifndef BOARDDRIVERS_OLED_H_
#define BOARDDRIVERS_OLED_H_


/********************************************************************/
/*-----------------------------------------------------------------------
LCD_init          : OLED初始化

编写日期          ：2012-11-01
最后修改日期      ：2012-11-01
-----------------------------------------------------------------------*/
 void initOled(void);
 void init_iic(void);
 void OLED_CLS(void);
 void OLED_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[],unsigned char state);
 void OLED_PutPixel(unsigned char x,unsigned char y);
 void OLED_Set_Pos(unsigned char x, unsigned char y);
 void OLED_WrDat(unsigned char data);
 void OLED_Fill(unsigned char dat);
 void Dly_ms(unsigned short ms);
 void setContrast(unsigned char contrast);
/********************************************************************/




#endif /* BOARDDRIVERS_OLED_H_ */
