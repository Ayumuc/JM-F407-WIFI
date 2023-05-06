/*
 * iic.h
 *
 *  Created on: Dec 9, 2021
 *      Author: Administrator
 */

#ifndef IIC_H_
#define IIC_H_

#include "stm32f4xx_hal.h"


/* 定义控制 SDA SCL 的宏 标准库版
#define I2C_SDA_UP        GPIO_SetBits  (GPIOC,GPIO_PIN_8)    //SDA高电平
#define I2C_SDA_LOW        GPIO_ResetBits(GPIOC,GPIO_PIN_8)    //SDA低电平
#define I2C_SCL_UP        GPIO_SetBits  (GPIOC,GPIO_PIN_9)    //SCL高电平
#define I2C_SCL_LOW        GPIO_ResetBits(GPIOC,GPIO_PIN_9)    //SCL低电平
*/


 /* 定义控制 SDA SCL 的宏 HAL库版    */

#define I2C_SDA_UP         HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET)              //SDA高电平
#define I2C_SDA_LOW        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_RESET)            //SDA低电平
#define I2C_SCL_UP         HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_SET)              //SCL高电平
#define I2C_SCL_LOW        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_RESET)            //SCL低电平



/* 获取 SDA SCL 引脚状态，标准库版也是用相同逻辑    */

#define I2C_SDA        HAL_GPIO_ReadPin(GPIOD ,GPIO_PIN_8)        //获取SDA引脚状态
#define I2C_SCL        HAL_GPIO_ReadPin(GPIOD ,GPIO_PIN_9)        //获取SCL引脚状态


void         i2c_start(void);                      //*    开始信号
void         i2c_ack(void);                        //*    应答信号
void         I2C_No_ack(void);                     //*    非应答信号
void         i2c_stop(void);                       //*    停止信号
char         i2c_wit_ack(void);                    //*    等待应答
void         i2c_send(uint8_t dat);                //*    发送数据
uint8_t i2c_read(void);                            //*    接收数据

void SDA_OUT(void);                                //*    SDA 设置为输出
void SDA_IN(void);                                 //*    SDA 设置为输入

//void delay_us(uint32_t i);                          //*延时


#endif /* IIC_H_ */
