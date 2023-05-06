/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "task.h"
#include "queue.h"
#include "tim.h"
#include "oled.h"
#include "MCP3208.h"
#include "rtc.h"
#include "usart.h"
#include "semphr.h"
#include "boardConfig.h"
#include "display.h"
#include "rs485.h"
#include "tcpServer.h"
#include "cJSON.h"
#include "ring_fifo.h"
#include "DigitIO.h"
#include "logQueue.h"
#include <string.h>
#include "sd2078.h"
#include "cpu_utils.h"
#include "flash.h"
#include "Temperature.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONFIG_KEY_DOWN()                    !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)
#define CONFIG_KEY_UP()                       HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

DEVICE_CONFIG  config;

TaskHandle_t   setZeroTaskHandle;
TaskHandle_t   logUploadTaskHandle;       //日志任务句柄
TaskHandle_t   daemonTaskHandle;          // led任务句柄
TaskHandle_t   tcpServerTaskHandle;       //modbus tcp 任务句柄
TaskHandle_t   wifiClientTaskHandle;
TaskHandle_t   rs485TaskHandle;           //rs485任务句柄
TaskHandle_t   taskHeapDetectTaskHandle;  //


extern      uint8_t        config_buff[100];

/* USER CODE END Variables */
/* Definitions for startupTask */
osThreadId_t startupTaskHandle;
const osThreadAttr_t startupTask_attributes = {
  .name = "startupTask",
  .stack_size = 2256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void daemonTask(void *pvParameters);
void taskHeapDetectTask(void *pvParameters);

/* USER CODE END FunctionPrototypes */

void StartupTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 2 */
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
/* USER CODE END 3 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of startupTask */
  startupTaskHandle = osThreadNew(StartupTask, NULL, &startupTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartupTask */
static unsigned char isConfigMode(void)
{
	char i=0;
	while(CONFIG_KEY_DOWN())
	{
		vTaskDelay(pdMS_TO_TICKS(100));//100ms
		if(++i > 20)return 1;
	}
	return 0;
}

static void configMode()
{
	if (isConfigMode())
	{
		printf("---------------------------------------\r\n");
		printf("config mode: \r\n1.Please use usb2ttl tool, send config to uart4, Baud rate is 115200bit/s\r\n");
		printf("example:\r\n");
		printf("set eth send:\r\n{\"IP:192.168.1.49;PORT:502;NETMASK:255.255.255.0;GATEWAY:192.168.1.1;\"}\r\n");
		printf("set analog A10 send:\r\n{\"A10 range:500;A10 type:0\"}\r\n");
		printf("set analog A11 send:\r\n{\"A11 range:400;A11 type:0\"}\r\n");
		printf("set analog A20 send:\r\n{\"A20 range:300;A20 type:0\"}\r\n");
		printf("set analog A21 send:\r\n{\"A21 range:200;A21 type:0\"}\r\n");
		printf("set analog A30 send:\r\n{\"A30 range:100;A30 type:0\"}\r\n");
		printf("set analog A31 send:\r\n{\"A31 range:90;A31 type:0\"}\r\n");
		printf("set analog A40 send:\r\n{\"A40 range:80;A40 type:0\"}\r\n");
		printf("set analog A41 send:\r\n{\"A41 range:70;A41 type:0\"}\r\n");

		while(1)
		{
	        if(uart4_recv_end_flag ==1)                                           //uart4_recv_end_flag 结束标志
	        {
				printf("uart4_rx_len = %d\r\n",uart4_rx_len);                    //uart4_rx_len 此次接收到了多少数据

				configRecv(uart4_rx_buffer, uart4_rx_len);
				configHandler(config_buff);
				uart4_rx_len=0;
				uart4_recv_end_flag=0;
				HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer, UART4_BUF_SIZE);//这句放if里面还是外面�????????
	        }
		}
	}
}



/**
  * @brief  Function implementing the startupTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartupTask */
void StartupTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartupTask */
    BaseType_t  ret;

    initLogQueue();

    init_iic();

    initMCP();

    USER_RTC_Init();

#if LOG_PRINTF_DEF
    LogPrint("StartupTask start");
#else
    printf("StartupTask start\r\n");
#endif

    initMachineStatus(readConfig());

  	//write config or not;
    configMode();

	ret = xTaskCreate(setZeroTask, "setZeroTask", 512, NULL, 3, &setZeroTaskHandle);
	if (ret != pdTRUE)
	{
#if LOG_PRINTF_DEF
		LogPrint("setZeroTask create err\r\n");
#else
		printf("setZeroTask create err\r\n");
#endif
	}

	ret = xTaskCreate(logUploadTask, "logUploadTask", 512, NULL, 10, &logUploadTaskHandle);
	if (ret != pdTRUE)
	{
#if LOG_PRINTF_DEF
		LogPrint("logUploadTask create err\r\n");
#else
		printf("logUploadTask create err\r\n");
#endif
	}

	ret = xTaskCreate(daemonTask, "daemonTask", 512, NULL, 2, &daemonTaskHandle);
	if (ret != pdTRUE)
	{
#if LOG_PRINTF_DEF
		LogPrint("daemonTask create err");
#else
		printf("daemonTask create err\r\n");
#endif
	}

	ret = xTaskCreate(tcpServerTask, "tcpServerTask", 512, NULL, 7, &tcpServerTaskHandle);
	if (ret != pdTRUE)
	{
#if LOG_PRINTF_DEF
		LogPrint("tcpServerTask create err");
#else
		printf("tcpServerTask create err\r\n");
#endif
	}

	ret = xTaskCreate(wifiClientTask, "wifiClientTask", 512, NULL, 6, &wifiClientTaskHandle);
	if (ret != pdTRUE)
	{
#if LOG_PRINTF_DEF
		LogPrint("wifiClientTask create err");
#else
		printf("wifiClientTask create err\r\n");
#endif
	}

	ret = xTaskCreate(rs485DataHandler, "rs485DataHandler", 512, NULL, 3, &rs485TaskHandle);
	if (ret != pdTRUE)
	{
#if LOG_PRINTF_DEF
		LogPrint("rs485DataHandler task create err");
#else
		printf("rs485DataHandler task create err\r\n");
#endif
	}

	ret = xTaskCreate( taskHeapDetectTask, "taskHeapDetectTask", 512, NULL, 7, &taskHeapDetectTaskHandle);
	if (ret != pdTRUE)
	{
#if LOG_PRINTF_DEF
		LogPrint("taskHeapDetectTask create err");
#else
		printf("taskHeapDetectTask create err\r\n");
#endif
	}

  /* Infinite loop */

#if LOG_PRINTF_DEF
	LogPrint("StartupTask exit");
#else
	printf("StartupTask exit\r\n");
#endif

    osThreadExit();
  /* USER CODE END StartupTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#include "delay.h"
void daemonTask(void *pvParameters)
{
	static char cnt;

#if LOG_PRINTF_DEF
	LogPrint("daemonTask start");
#else
	printf("daemonTask start\r\n");
#endif

	while (1)
	{
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
		vTaskDelay(5000);

		if (cnt++ > 10)
		{
			cnt = 0;
			LogPrint("----daemonTask----");
		}
	}
	printf("daemonTask exit\r\n");
	vTaskDelete(NULL);
}


#define TASK_NUM 5
typedef struct{
	const char *   task_name;
	TaskHandle_t   task_handler;
}TASK_HANDLER_TAP;

//   logUploadTaskHandle;       //日志任务句柄
//   daemonTaskHandle;          // led任务句柄
//   tcpServerTaskHandle;       //modbus tcp 任务句柄
//   rs485TaskHandle;           //rs485任务句柄
void taskHeapDetectTask(void *pvParameters)
{
#if 0
	unsigned char i = 0;
	TASK_HANDLER_TAP task_handler_tap [TASK_NUM] = {
//			{"setZeroTask",        setZeroTaskHandle},
			{"logUploadTask",      logUploadTaskHandle},
			{"daemonTask",         daemonTaskHandle},
			{"tcpServerTask",      tcpServerTaskHandle},
			{"rs485Task",          rs485TaskHandle},
			{"taskHeapDetectTask", taskHeapDetectTaskHandle},
	};



	UBaseType_t task_surplus_heap = 0;
	size_t task_free_heap = 0;
//	uint16_t os_usage = 0;

#if LOG_PRINTF_DEF
	LogPrint("taskHeapDetectTask start");
#else
	printf("taskHeapDetectTask start\r\n");
#endif

	while (1)
	{
		for (i = 0; i < TASK_NUM; i++)
		{
			task_surplus_heap = uxTaskGetStackHighWaterMark(task_handler_tap[i].task_handler);
			task_free_heap    = xPortGetFreeHeapSize();
#if LOG_PRINTF_DEF

//			os_usage = osGetCPUUsage();
//			printf("RTOS CPU usage: %d%%\r\n",os_usage);

			//rtos 空闲�???????
			snprintf(log_msg_global, sizeof(log_msg_global), "RTOS free heap: %d bytes", task_free_heap);
			LogPrint(log_msg_global);

			//各个任务空闲�???????
			snprintf(log_msg_global, sizeof(log_msg_global), "task[%s] surplus heap: %ld bytes", task_handler_tap[i].task_name, task_surplus_heap*4);
			LogPrint(log_msg_global);
#else
			printf("task[%s] surplus heap:%d bytes\r\n", task_handler_tap[i].task_name, task_surplus_heap*4);
#endif
			vTaskDelay(1000);
		}
	}
	printf("taskHeapDetectTask exit\r\n");
	vTaskDelete(NULL);
#else
	unsigned char i = 0;
	UBaseType_t task_surplus_heap = 0;
	size_t task_free_heap = 0;
#if LOG_PRINTF_DEF
	LogPrint("taskHeapDetectTask start");
#else
	printf("taskHeapDetectTask start\r\n");
#endif

	while (1)
	{
		printf("--------------------------------------------------------\r\n");
		task_free_heap    = xPortGetFreeHeapSize();
		snprintf(log_msg_global, sizeof(log_msg_global), "RTOS free heap: %d bytes", task_free_heap);
		LogPrint(log_msg_global);

		task_surplus_heap = uxTaskGetStackHighWaterMark(setZeroTaskHandle);
		snprintf(log_msg_global, sizeof(log_msg_global), "task[%s] surplus heap: %ld bytes", "setZeroTask", task_surplus_heap*4);
		LogPrint(log_msg_global);

		task_surplus_heap = uxTaskGetStackHighWaterMark(logUploadTaskHandle);
		snprintf(log_msg_global, sizeof(log_msg_global), "task[%s]: surplus heap: %ld bytes", "logUploadTask", task_surplus_heap*4);
		LogPrint(log_msg_global);

		task_surplus_heap = uxTaskGetStackHighWaterMark(daemonTaskHandle);
		snprintf(log_msg_global, sizeof(log_msg_global), "task[%s]: surplus heap: %ld bytes", "daemonTask", task_surplus_heap*4);
		LogPrint(log_msg_global);

		task_surplus_heap = uxTaskGetStackHighWaterMark(tcpServerTaskHandle);
		snprintf(log_msg_global, sizeof(log_msg_global), "task[%s]: surplus heap: %ld bytes", "tcpServerTask", task_surplus_heap*4);
		LogPrint(log_msg_global);

		task_surplus_heap = uxTaskGetStackHighWaterMark(wifiClientTaskHandle);
		snprintf(log_msg_global, sizeof(log_msg_global), "task[%s]: surplus heap: %ld bytes", "wifiClientTask", task_surplus_heap*4);
		LogPrint(log_msg_global);

		task_surplus_heap = uxTaskGetStackHighWaterMark(rs485TaskHandle);
		snprintf(log_msg_global, sizeof(log_msg_global), "task[%s]: surplus heap: %ld bytes", "rs485Task", task_surplus_heap*4);
		LogPrint(log_msg_global);

		task_surplus_heap = uxTaskGetStackHighWaterMark(taskHeapDetectTaskHandle);
		snprintf(log_msg_global, sizeof(log_msg_global), "task[%s]: surplus heap: %ld bytes", "taskHeapDetectTask", task_surplus_heap*4);
		LogPrint(log_msg_global);

		printf("Temperature:[%lf]\n\r",Tem.My_Temperature);

		printf("--------------------------------------------------------\r\n");

		for (i = 0; i < 5; i++)
		{
			vTaskDelay(10000);
		}
	}

	printf("taskHeapDetectTask exit\r\n");
	vTaskDelete(NULL);
#endif
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
