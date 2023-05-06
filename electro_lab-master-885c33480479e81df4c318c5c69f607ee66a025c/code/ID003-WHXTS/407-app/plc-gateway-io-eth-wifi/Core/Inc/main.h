/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "logQueue.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCP_CLK_Pin GPIO_PIN_5
#define MCP_CLK_GPIO_Port GPIOA
#define MCP_MISI_Pin GPIO_PIN_6
#define MCP_MISI_GPIO_Port GPIOA
#define MCP_MISO_Pin GPIO_PIN_0
#define MCP_MISO_GPIO_Port GPIOB
#define MCP_CS_Pin GPIO_PIN_1
#define MCP_CS_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_7
#define SW2_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOE
#define OLED_DC_PIN_Pin GPIO_PIN_14
#define OLED_DC_PIN_GPIO_Port GPIOB
#define OLED_RST_PIN_Pin GPIO_PIN_15
#define OLED_RST_PIN_GPIO_Port GPIOB
#define Wifi_TX_Pin GPIO_PIN_10
#define Wifi_TX_GPIO_Port GPIOC
#define Wifi_RX_Pin GPIO_PIN_11
#define Wifi_RX_GPIO_Port GPIOC
#define Debug_Pin GPIO_PIN_12
#define Debug_GPIO_Port GPIOC
#define PD0_DOUT1_Pin GPIO_PIN_0
#define PD0_DOUT1_GPIO_Port GPIOD
#define PD1_DOUT2_Pin GPIO_PIN_1
#define PD1_DOUT2_GPIO_Port GPIOD
#define PD3_DOUT3_Pin GPIO_PIN_3
#define PD3_DOUT3_GPIO_Port GPIOD
#define PD4_DOUT4_Pin GPIO_PIN_4
#define PD4_DOUT4_GPIO_Port GPIOD
#define PB6_DIN4_Pin GPIO_PIN_6
#define PB6_DIN4_GPIO_Port GPIOB
#define PB7_DIN3_Pin GPIO_PIN_7
#define PB7_DIN3_GPIO_Port GPIOB
#define PB8_DIN2_Pin GPIO_PIN_8
#define PB8_DIN2_GPIO_Port GPIOB
#define PB9_DIN1_Pin GPIO_PIN_9
#define PB9_DIN1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define LOG_PRINTF_DEF 1

extern char log_msg_global[256];

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
