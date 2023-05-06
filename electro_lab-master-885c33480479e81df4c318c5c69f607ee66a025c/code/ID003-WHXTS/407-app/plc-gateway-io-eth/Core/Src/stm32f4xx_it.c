/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "tim.h"
#include "lwip.h"
#include "dma.h"
#include <stdio.h>
#include <string.h>
#include "cJSON.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef struct
{
	uint32_t R0;
	uint32_t R1;
	uint32_t R2;
	uint32_t R3;
	uint32_t R12;
	uint32_t LR;
	uint32_t PC;
	uint32_t xPSR;
}STACK_DATA_TYPE;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//uint8_t recv;
//uint8_t flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void NotOSHardFault_Handler(uint32_t msp_addr);
void OSHardFault_Handler(uint32_t psp_addr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern      uint8_t  uart2_rx_buffer[UART2_BUF_SIZE];



/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */
//extern uint8_t usart2RxBuffer;
//extern __IO uint8_t recv_end_flag;
//extern __IO uint8_t rx_len = 0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
//#define ERR_INFO "\r\nEnter HardFault_Handler, System restart!!\r\n"

//	uint32_t msp_addr = __get_MSP();		//获取线程模式下堆栈指针位�????????
	uint32_t psp_addr = __get_PSP();		//获取中断下的堆栈指针位置-用于OS启动�????????

#if(1)						                //使能了操作系统的�????????个宏定义，自己去定义
		OSHardFault_Handler(psp_addr);
#else //没有使能操作系统
	NotOSHardFault_Handler(msp_addr);
#endif //UCOS_II_EN

//	HAL_Delay(500); 					    //延时�????????下，防止重启速度太快

	HAL_NVIC_SystemReset();

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */



    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =  __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE);
	if((tmp_flag != RESET))
    {
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);
			temp = huart2.Instance->SR;
			temp = huart2.Instance->DR;
			HAL_UART_DMAStop(&huart2);
//			temp  = hdma_uart4_rx.Instance->NDTR;
			// !!重要 停用CPU缓存，CPU能得到正确的DMA数据
			// 地址要求32字节对齐，长度是32的位�?????????

			temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
			uart2_rx_len =  UART2_BUF_SIZE - temp;
			uart2_recv_end_flag = 1;
	}


  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  /* USER CODE BEGIN UART4_IRQn 1 */


	uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =  __HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE);
	if((tmp_flag != RESET))
    {
			__HAL_UART_CLEAR_IDLEFLAG(&huart4);
			temp = huart4.Instance->SR;
			temp = huart4.Instance->DR;
			HAL_UART_DMAStop(&huart4);
//			temp  = hdma_uart4_rx.Instance->NDTR;
			// !!重要 停用CPU缓存，CPU能得到正确的DMA数据
			// 地址要求32字节对齐，长度是32的位�?????????

			temp = __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
			uart4_rx_len =  UART4_BUF_SIZE - temp;
			uart4_recv_end_flag = 1;
	}

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles Ethernet global interrupt.
  */
void ETH_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}

/* USER CODE BEGIN 1 */

extern int  log_client_sock;
void HardFault_Send_str(char *str)
{
	int length = strlen(str);
	sendToTcpServer(log_client_sock, (unsigned char *)str, length, 1);
}

void HardFault_Send_json(char* key,char *value)
{
	cJSON *root = NULL;
	char* out = NULL;
	int length = 0;

	root = cJSON_CreateObject(); //
	if (root == NULL)
	{
		printf("line:%d ----------- cJSON_CreateObject error\r\n", __LINE__);
		return ;
	}

	cJSON_AddStringToObject(root, key,  value);
	out = cJSON_Print(root);
	length = strlen(out);
	sendToTcpServer(log_client_sock, (unsigned char *)out, length, 1);
	free(out);
	cJSON_Delete(root);
}

//有操作系统时的异常处�????????
void OSHardFault_Handler(uint32_t psp_addr)
{
	STACK_DATA_TYPE *p;										//堆栈中存储的数据

	if((psp_addr >> 20) != 0x200)		//判断地址范围，必须是0x200xxxxx 范围
	{
//		printf("警告：堆栈指针被破坏，无法记录现场！\r\n");
		printf("warning:Stack pointer corrupted, unable to record live!\r\n");
		HardFault_Send_json("error","Stack pointer corrupted");
		return;
	}


	p = (STACK_DATA_TYPE *)psp_addr;

	snprintf(log_msg_global, sizeof(log_msg_global), "0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X ",
			(unsigned int)p->R0,  (unsigned int)p->R1,  (unsigned int)p->R2,  (unsigned int)p->R3,  (unsigned int)p->R12,  (unsigned int)p->LR,  (unsigned int)p->PC, (unsigned int)p->xPSR);
	HardFault_Send_json("Reg", log_msg_global);

	printf("R0:0x%08X\r\n", (unsigned int)p->R0);
	printf("R1:0x%08X\r\n", (unsigned int)p->R1);
	printf("R2:0x%08X\r\n", (unsigned int)p->R2);
	printf("R3:0x%08X\r\n", (unsigned int)p->R3);
	printf("R12:0x%08X\r\n", (unsigned int)p->R12);
	printf("LR:0x%08X\r\n", (unsigned int)p->LR);
	printf("PC:0x%08X\r\n", (unsigned int)p->PC);
	printf("xPSR:0x%08X\r\n", (unsigned int)p->xPSR);

}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
