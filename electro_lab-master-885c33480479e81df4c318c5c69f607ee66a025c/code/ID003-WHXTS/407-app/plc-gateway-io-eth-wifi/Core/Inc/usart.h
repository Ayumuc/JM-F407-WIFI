/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define        UART2_BUF_SIZE       (1400)
#define        UART4_BUF_SIZE       (100)


extern     volatile uint16_t  uart4_rx_len;
extern     volatile uint8_t   uart4_recv_end_flag;
extern              uint8_t   uart4_rx_buffer[UART4_BUF_SIZE];

extern     volatile  uint16_t  uart2_rx_len;
extern     volatile  uint8_t   uart2_recv_end_flag;
extern               uint8_t   uart2_rx_buffer[UART2_BUF_SIZE];

//typedef struct
//{
//	uint8_t recbuf[RS485_BUF_SIZE];
//	uint8_t reclen;
//	uint8_t recflag;
//}RS485;

/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void Uart1RegisterIdleIrqCallback(void (*Callback)(UART_HandleTypeDef *huart));
UART_HandleTypeDef *Uart1GetHandle(void);
void uart1_poll_send(void);
uint32_t uart1_read(void *buf, uint32_t len);
void uart1_write(const void *data, uint32_t len);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
