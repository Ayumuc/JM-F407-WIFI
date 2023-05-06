/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tftp_server.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart4_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define FLASH_PROGRAM_ADDR						(0x08020000)
#define FLASH_JUMP_ADDR							(FLASH_PROGRAM_ADDR)
#define FLASH_OTAFLAG_ADDR						(0x0805FFF0)
#define FLASH_FILESIZE_ADDR						(0x0805FF00)
typedef  void (*pFunction)(void);
pFunction jump_to_application;
uint32_t jump_address;
/*!
* @brief 跳转到应用程序段
*        执行条件：无
* @param[in1] : 用户代码起始地址.
*
* @retval: �?????
*/
void jump_to_app(uint32_t app_addr)
{
    //printf("app_addr=%X=%X=%X=%X\n",app_addr, *(__IO uint32_t*)app_addr, *(__IO uint32_t*)(app_addr+4), ((*(__IO uint32_t*)app_addr) & 0x2FFE0000 ));
    /* Check if valid stack address (RAM address) then jump to user application */
    //if (((*(__IO uint32_t*)app_addr) & 0x2FFE0000 ) == 0x20020000)
    {
      /* Jump to user application */
      jump_address = *((__IO uint32_t*) (app_addr + 4));
      jump_to_application = (pFunction) jump_address;

      /* Initialize user application's Stack Pointer */
      __set_MSP(*(__IO uint32_t*) jump_address);
      jump_to_application();
    }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  if(*(__IO uint8_t*)(FLASH_OTAFLAG_ADDR) == 0x33)
  {
	  if(((FLASH_JUMP_ADDR+4)&0xFF000000)==0x08000000) //Judge if start at 0X08XXXXXX.
	{

		jump_to_app(FLASH_JUMP_ADDR);//Load APP
	}
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  printf("start bootloader...\n");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
   HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer, UART4_BUF_SIZE);//打开DMA接收，数据存入rx_buffer数组中�??
  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD0 PD1 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


uint32_t writedata_to_flash(uint8_t *data, uint32_t len, uint32_t address)
{
	uint32_t i = 0, temp;

	HAL_FLASH_Unlock();
	for(i = 0; i < len; i++)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)address, *data) == HAL_OK)
		{
			address = address + 1;
			data = data + 1;
		}
		else
		{
			return -1;
		}

	}

	HAL_FLASH_Lock();

	return address;
 }

int8_t erase_flash(uint32_t sector_start, uint32_t sector_end)
{
	 FLASH_EraseInitTypeDef EraseInitStruct;
	 uint32_t SECTORError = 0;
     // sector 5  0x08020000 - 0x0803FFFF
	 // sector 6  0x08040000 - 0x0805FFFF
	 // sector 7  0x08060000 - 0x0807FFFF
	 // sector 8  0x08080000 - 0x0809FFFF
	 // sector 9  0x080A0000 - 0x080BFFFF
	 // sector 10 0x080C0000 - 0x080DFFFF
	 // sector 11 0x080E0000 - 0x080FFFFF
	 HAL_FLASH_Unlock();
	 EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	 EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	 EraseInitStruct.Sector        = sector_start;
	 EraseInitStruct.NbSectors     = sector_end;
	 if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	 {
		 return -1;
	 }
	 HAL_FLASH_Lock();
 }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "md5.h"

uint32_t readsize = 0;
uint32_t writesize = 0;
uint8_t cmdflag = 0;
static void* tftp_open(const char* fname, const char* mode, u8_t is_write, u32_t filesize)
{
  LWIP_UNUSED_ARG(mode);
  if(*(uint8_t*)fname == 'E')
	  cmdflag = 1;
  else if(*(uint8_t*)fname == 'R')
	  cmdflag = 2;
  else if(*(uint8_t*)fname == 'W')
	  cmdflag = 3;
  else if(*(uint8_t*)fname == 'M')
  	  cmdflag = 4;
  else if(*(uint8_t*)fname == 'D')
  	  cmdflag = 5;
  else if(*(uint8_t*)fname == 'F')
  	  cmdflag = 6;
  else if(*(uint8_t*)fname == 'A')
  	  cmdflag = 7;
  else if(*(uint8_t*)fname == 'S')
  	  cmdflag = 8;
  printf("send start:%s is_write = %d cmdflag = %d - %c\n", fname, is_write, cmdflag, *(uint8_t*)fname);
  return 1;
}

static void tftp_close(void* handle)
{
	printf("send over\n");
}

uint32_t readaddr = 0;
unsigned char decrypt[16];
static int tftp_read(void* handle, void* buf, int bytes)
{
	if(cmdflag == 0x02){
		if(readaddr < (readsize - 512) )
		{
			memcpy(buf, FLASH_PROGRAM_ADDR + readaddr, 512);
			readaddr += 512;
			bytes = 512;
		}
		else
		{
			memcpy(buf, FLASH_PROGRAM_ADDR + readaddr, readsize - readaddr);
			readaddr += readsize - readaddr;
			bytes = readsize - readaddr;
			printf("read over readaddr = %d , bytes = %d\n", readaddr, bytes);
			readaddr = 0;
			cmdflag = 0;
		}
	}
	else if(cmdflag == 0x07){
		if(readaddr < (0xE0000 - 512) )
		{
			memcpy(buf, FLASH_PROGRAM_ADDR + readaddr, 512);
			readaddr += 512;
			bytes = 512;
		}
		else
		{
			memcpy(buf, FLASH_PROGRAM_ADDR + readaddr, 0xE0000 - readaddr);
			readaddr += 0xE0000 - readaddr;
			bytes = 0xE0000 - readaddr;
			printf("read over readaddr = %d , bytes = %d\n", readaddr, bytes);
			readaddr = 0;
			cmdflag = 0;
		}
	}
	else if(cmdflag == 0x04){
		printf("start md5 checksum\n");
		memcpy(buf, "md5", 3);
		bytes = 3;

	}
	else if(cmdflag == 0x05){
		printf("return md5 checksum\n");
		for(int i=0; i<16; i++) {
			printf("%02x", decrypt[i]);
		}
		printf("\n");
		memcpy(buf, decrypt, 16);
		bytes = 16;

	}
	else if(cmdflag == 0x06){
		uint32_t filebytes = 0;
		uint8_t numbuf[12];
	    bzero(numbuf, sizeof(numbuf));
		filebytes = *(__IO uint32_t*)(FLASH_FILESIZE_ADDR);
		sprintf(numbuf, "%d", filebytes);
		printf("filebytes = %d\n", filebytes, filebytes);
		memcpy(buf, numbuf, strlen(numbuf));
		bytes = strlen(numbuf);
		cmdflag = 0;
	}
	else if(cmdflag == 0x01){
		printf("start erase\n");
		memcpy(buf, "erase", 6);
		bytes = 6;

	}
	return bytes;
}


uint8_t tftpbuf[516];
uint8_t k = 0;
uint8_t ota_flag = 0;
uint16_t tftpsn = 0;
uint32_t flashaddr = FLASH_PROGRAM_ADDR;
uint32_t count = 0;

static int tftp_write(void* handle, struct pbuf* p)
{
	if(cmdflag == 0x03){
		ota_flag = 0x33;
	}
	else if(cmdflag == 0x08){
		ota_flag = 0x77;
	}
	else
	{
		memcpy(tftpbuf, p->payload, p->len);
		flashaddr = writedata_to_flash(tftpbuf, p->len, flashaddr);
		count = count + p->len;
		if(p->len < 512)
		{
			printf("count = %d len = %d\n", count, p->len);
			ota_flag = 0x3F;
			writesize = count;
			readsize = count;
			count = 0;
			cmdflag = 0;
		}
	}

    return 0;
}

const struct tftp_context tftp = {
		tftp_open,
		tftp_close,
		tftp_read,
		tftp_write,
};



#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}



volatile  uint16_t  uart4_rx_len;
volatile  uint8_t   uart4_recv_end_flag;
          uint8_t   uart4_rx_buffer[UART4_BUF_SIZE];
TaskHandle_t   wifiClientTaskHandle;

char * at_okip_str = "OKIP\r\n";
char * at_busy_str = "busy p...\r\n";
char * at_ok_str = "OK\r\n";
char * at_error_str = "ERROR\r\n";
char * at_conn_str = "CONNECT\r\n";
char * at_aready_conn_str = "ALREADY CONNECTED\r\n";
char * at_closed_str = "CLOSED\r\n";
char * at_wifi_disconn_str = "WIFI DISCONNECT\r\n"; // 5 disconnect AP
char * at_wifi_conn_str = "WIFI CONNECT\r\n"; // 5 disconnect AP
char * at_wifi_getip_str = "WIFI GOT IP\r\n"; // 2 connect AP

char * at_cwmode_val1_str = "+CWMODE:1\r\n";
char * at_cwmode_val2_str = "+CWMODE:2\r\n";
char * at_cwmode_val3_str = "+CWMODE:3\r\n";

char * at_cipmode_val1_str = "+CIPMODE:1\r\n";
char * at_cipmode_val0_str = "+CIPMODE:0\r\n";

char * at_cifsr_ack_ip_str = "+CIFSR:STAIP,\"0.0.0.0\"\r\n";

char * at_cipstatus0_str = ",0\r\n";
char * at_wifi_status2_str = "STATUS:2\r\n"; // 2 connect AP
char * at_wifi_status3_str = "STATUS:3\r\n"; // 3 connect TCP
char * at_wifi_status4_str = "STATUS:4\r\n"; // 4 disconnect TCP
char * at_wifi_status5_str = "STATUS:5\r\n"; // 5 disconnect AP

char ssid_buf[20];
char pwd_buf[20];
char cwjap_buf[100];
char * at_cwjap_ack1_str = "+CWJAP:1\r\n"; // 1 timeout
char * at_cwjap_ack2_str = "+CWJAP:2\r\n"; // 2 passwd error,
char * at_cwjap_ack3_str = "+CWJAP:3\r\n"; // 3 no AP,
char * at_cwjap_ack4_str = "+CWJAP:4\r\n"; // 4 connect fail

char ip_buf[16];
char sip_buf[16];
char port_buf[6];
char cipstart_buf[100];

uint8_t at_rxbuf[UART4_BUF_SIZE];
uint8_t modbus_tx_buf[516];
uint16_t at_txsize = 0;
uint16_t at_rxcnt = 0;
typedef enum{
	STATE_AT_3PLUS_TX = 0,
	STATE_AT_RST_TX,
	STATE_AT_CWMODE_GET_TX,
	STATE_AT_CWMODE_SET_TX,
	STATE_AT_CIPMODE_GET_TX,
	STATE_AT_CIPMODE_SET_TX,
	STATE_AT_CIFSR_TX,
	STATE_AT_CIPSTATUS_TX,
	STATE_AT_CWJAP_TX,
	STATE_AT_CIPSTART_TX,
	STATE_AT_CIPSEND_TX,
	STATE_AT_CIPTRANS_TX,
	STATE_AT_PLUS3_TX,
	STATE_AT_NONE
}STATE_t;

STATE_t at_state_tx = STATE_AT_3PLUS_TX;
STATE_t at_state_rx = STATE_AT_3PLUS_TX;
void wifi_at_proc(uint8_t * rxbuf, uint16_t rxlen);
void wifi_at_send(void);
void wifi_uart_send(uint8_t * txbuf, uint16_t len);
void at_cifsr_send(void);
extern DEV_CONFIG  config;

void print_hex(uint8_t * buf, uint16_t len)
{
	uint16_t i = 0;

	for(i = 0; i < len; i++)
		printf("%02X", buf[i]);
	printf("\r\n");
}


uint8_t wifi_tftpbuf[516];
uint8_t wifi_cmdflag = 0;
uint32_t wifi_flashaddr = FLASH_PROGRAM_ADDR;
uint32_t  wifi_count = 0;
uint16_t  wifi_sn= 0;
uint16_t  wifi_wsn= 0;
uint32_t wifi_readaddr = 0;
uint32_t wifi_readsize = 0;
uint32_t wifi_writesize = 0;
uint8_t wifi_ota_flag = 0;
unsigned char wifi_decrypt[16];
void wifiClientTask(void *pvParameters)
{
#if LOG_PRINTF_DEF
	LogPrint("wifiClientTask start");
#else
	printf("wifiClientTask start\r\n");
#endif

	printf("wait wifi for connect ...\r\n");
	readConfig();//读出配置
	bzero(ssid_buf, 20);
	bzero(pwd_buf, 20);
	bzero(ip_buf, 16);
	bzero(sip_buf, 16);
	bzero(port_buf, 6);
	memcpy(ssid_buf, config.wifi_info.ssid, 20);
	memcpy(pwd_buf, config.wifi_info.pwd, 20);
	memcpy(ip_buf, config.wifi_ip.wip, 16);
	memcpy(sip_buf, config.wifi_ip.wsip, 16);
	memcpy(port_buf, config.wifi_ip.wport, 6);
	printf("ssid:%s pwd:%s ip:%s sip:%s port:%s\r\n", ssid_buf, pwd_buf, ip_buf, sip_buf, port_buf);
	bzero(cwjap_buf, 100);
	bzero(cipstart_buf, 100);
	snprintf(cwjap_buf, sizeof(cwjap_buf), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid_buf, pwd_buf);
	//snprintf(cipstart_buf, sizeof(cipstart_buf), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", ip_buf, port_buf);
	snprintf(cipstart_buf, sizeof(cipstart_buf), "AT+CIPSTART=\"UDP\",\"%s\",3355,69,0\r\n", sip_buf);
	uart4_recv_end_flag = 1;
	vTaskDelay(1000);

	wifi_ota_flag = *(__IO uint8_t*)(FLASH_OTAFLAG_ADDR);
	wifi_readsize = *(__IO uint32_t*)(FLASH_FILESIZE_ADDR);
	printf("reset wifi_ota_flag = %d\n", wifi_ota_flag);
	printf("reset wifi_readsize = %d\n", wifi_readsize);
	uint16_t ledcnt = 0;
    while (1)
    {
    	if(uart4_recv_end_flag == 1)   // 结束标志
		{
			bzero(at_rxbuf, UART4_BUF_SIZE);
			memcpy(at_rxbuf, uart4_rx_buffer, uart4_rx_len);
			bzero(uart4_rx_buffer, UART4_BUF_SIZE);
			if(STATE_AT_CIPTRANS_TX != at_state_rx)
				printf("%s\r\n=%d\r\n", at_rxbuf, at_state_rx);
			//else
				//print_hex(at_rxbuf, uart4_rx_len);
			wifi_at_proc(at_rxbuf, uart4_rx_len);
			uart4_recv_end_flag=0;
			uart4_rx_len=0;
			at_rxcnt = 0;
			wifi_at_send();
			HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer, UART4_BUF_SIZE);
		}


    	if(STATE_AT_CIPTRANS_TX != at_state_rx)
    	{

    		if(STATE_AT_RST_TX == at_state_rx)
			{
				if(at_rxcnt++ > 4)
				{
					at_state_tx = STATE_AT_CWMODE_GET_TX;
					at_rxcnt = 0;
					uart4_recv_end_flag = 1;
				}
			}
			else if((STATE_AT_CIPTRANS_TX > at_state_rx) && (STATE_AT_CWMODE_GET_TX < at_state_rx))
			{
				if(at_rxcnt++ > 40)
				{
					at_state_tx = STATE_AT_CIPMODE_GET_TX;
					at_rxcnt = 0;
					uart4_recv_end_flag = 1;
				}
			}
			else
				at_rxcnt = 0;
    		vTaskDelay(500);
    	}
    	else
    	{
    		vTaskDelay(10);
    	}

		if(wifi_cmdflag == 0x01){
			wifi_ota_flag = 0x30;
			wifi_cmdflag = 0;
		 }

		 if(wifi_cmdflag == 0x04){
			int i, k, len;
			unsigned char encrypt[1024];
			MD5_CTX md5;

			wifi_readsize = *(__IO uint32_t*)(FLASH_FILESIZE_ADDR);
			printf("wifi_readsize= %d\n", wifi_readsize);

			if(wifi_readsize <= 0x00040000){
				printf("wifi_md5 checking\n");
				MD5Init(&md5);

				for(i = 0; i < wifi_readsize - 1024; i = i + 1024){
					memcpy(encrypt, (uint8_t*)(FLASH_PROGRAM_ADDR + i), 1024);
					//for(k=0; k<1024; k++) {
					//	printf("%02x", encrypt[k]);
					//}
					//printf(" i = %d\n", i);
					MD5Update(&md5, encrypt, 1024);
				}
				printf("i = %d\n", i);
				printf("wifi_readsize - i = %d\n", wifi_readsize - i);

				memcpy(encrypt, (uint8_t*)(FLASH_PROGRAM_ADDR + i), wifi_readsize - i);
				for(k=0; k<(wifi_readsize - i); k++) {
					printf("%02x", encrypt[k]);
				}

				MD5Update(&md5, encrypt, wifi_readsize - i);
				MD5Final(&md5, wifi_decrypt);
			}

			printf("wifi_decrypt:");
			for(i=0; i<16; i++) {
				printf("%02x", wifi_decrypt[i]);
			}
			printf("\n");

			wifi_cmdflag = 0;
		 }

		 if(wifi_ota_flag == 0x11)
		 {
			 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);  	osDelay(250);
			 erase_flash(5, 6);
			 printf("wifi  erase ok block 5 and block 6 \n");
			 wifi_ota_flag = *(__IO uint8_t*)(FLASH_OTAFLAG_ADDR);
			 printf("wifi ota_flag = %X\n",wifi_ota_flag);
		 }
		 else if(wifi_ota_flag == 0xFF)
		 {
			 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);  osDelay(250);
			 wifi_ota_flag = 0x7F;
			 printf("wait wifi write or read ok \n");
		 }
		 else if(wifi_ota_flag == 0x7F)
		 {
			 if(ledcnt++ == 100){
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
			 }
			 else if(ledcnt++ == 200){
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
				 ledcnt = 0;
			 }
		 }
		 else if(wifi_ota_flag == 0x3F)
		 {
			 printf("wifi_writesize = %d\n", wifi_writesize);
			 writedata_to_flash(&wifi_writesize, 4, FLASH_FILESIZE_ADDR);
			 printf("wifi_readsize = %d\n", wifi_readsize);
			 wifi_readsize = *(__IO uint32_t*)(FLASH_FILESIZE_ADDR);
			 printf("wifi_readsize = %d\n", wifi_readsize);
			 wifi_ota_flag = 0x37;
		 }
		 else if(wifi_ota_flag == 0x37)
		 {
			 if(ledcnt++ == 100){
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
			 }
			 else if(ledcnt++ == 200){
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
				 ledcnt = 0;
			 }
		 }
		 else if(wifi_ota_flag == 0x33)
		 {
			printf("wifi_ota_flag = %X\n", wifi_ota_flag);
			writedata_to_flash(&wifi_ota_flag, 1, FLASH_OTAFLAG_ADDR);
			wifi_ota_flag = *(__IO uint8_t*)(FLASH_OTAFLAG_ADDR);
			printf("write wifi_ota_flag = %X\n", wifi_ota_flag);
			printf("reset wifi_device for bootloader to app\n");
			osDelay(1000);printf("3\n");
			osDelay(1000);printf("2\n");
			osDelay(1000);printf("1\n");
			osDelay(1000);printf("0\n");
			osDelay(1000);printf(".\n");
			HAL_NVIC_SystemReset();
		 }
		 else if(wifi_ota_flag == 0x77)
		 {
			printf("reset wifi_device\n");
			osDelay(1000);printf("3\n");
			osDelay(1000);printf("2\n");
			osDelay(1000);printf("1\n");
			osDelay(1000);printf("0\n");
			osDelay(1000);printf(".\n");
			HAL_NVIC_SystemReset();
		 }
		 else
		 {
			 wifi_ota_flag = 0x11;
		 }
    }



#if LOG_PRINTF_DEF
	LogPrint("wifiClientTask exit");
#else
	printf("wifiClientTask exit\r\n");
#endif

    vTaskDelete(NULL);

}



void wifi_uart_send(uint8_t * txbuf, uint16_t len)
{
	HAL_UART_Transmit(&huart4, txbuf, len, 50);
}

void at_reset_send(void)
{
	wifi_uart_send("AT+RST\r\n", sizeof("AT+RST\r\n"));
}

void at_cwmode_get_send(void)
{
	wifi_uart_send("AT+CWMODE?\r\n", sizeof("AT+CWMODE?\r\n"));
}

void at_cwmode_set_send(void)
{
	wifi_uart_send("AT+CWMODE=1\r\n", sizeof("AT+CWMODE=1\r\n"));
}

void at_cipmode_get_send(void)
{
	wifi_uart_send("AT+CIPMODE?\r\n", sizeof("AT+CIPMODE?\r\n"));
}

void at_cipmode_set_send(void)
{
	wifi_uart_send("AT+CIPMODE=1\r\n", sizeof("AT+CIPMODE=1\r\n"));
}

void at_cifsr_send(void)
{
	wifi_uart_send("AT+CIFSR\r\n", sizeof("AT+CIFSR\r\n"));
}


void at_cipstatus_send(void)
{
	wifi_uart_send("AT+CIPSTATUS\r\n", sizeof("AT+CIPSTATUS\r\n"));
}

void at_cwjap_send(void)
{
	// append ssid and password
	wifi_uart_send(cwjap_buf, strlen(cwjap_buf));
}

void at_cipstart_send(void)
{
	// append ip address
	wifi_uart_send(cipstart_buf, strlen(cipstart_buf));

}

void at_cipsend_send(void)
{
	wifi_uart_send("AT+CIPSEND\r\n", sizeof("AT+CIPSEND\r\n"));
}


void at_plus3_send(void)
{
	wifi_uart_send("+++", 3);
}

void at_modbus_send_data(void)
{
	wifi_uart_send(modbus_tx_buf, at_txsize);
	//osDelay(200);
	//wifi_uart_send(modbus_tx_buf, at_txsize);
	//osDelay(200);
}


void wifi_at_send(void)
{
	switch(at_state_tx)
	{
		case STATE_AT_3PLUS_TX:// 0 send +++ quit transfer mode
			at_plus3_send();
			at_state_rx = STATE_AT_3PLUS_TX;
		break;
		case STATE_AT_RST_TX:// 1 send reset
			at_reset_send();
			at_state_rx = STATE_AT_RST_TX;
		break;
		case STATE_AT_CWMODE_GET_TX:// 2 send cwmode? ap mode
			at_cwmode_get_send();
			at_state_rx = STATE_AT_CWMODE_GET_TX;
		break;
		case STATE_AT_CWMODE_SET_TX:// 3 set ap mode
			at_cwmode_set_send();
			at_state_rx = STATE_AT_CWMODE_SET_TX;
		break;
		case STATE_AT_CIPMODE_GET_TX:// 4 send cipmode? transfer mode
			at_cipmode_get_send();
			at_state_rx = STATE_AT_CIPMODE_GET_TX;
		break;
		case STATE_AT_CIPMODE_SET_TX:// 5 set transfer mode
			at_cipmode_set_send();
			at_state_rx = STATE_AT_CIPMODE_SET_TX;
		break;
		case STATE_AT_CIFSR_TX:// 6 send get ip address infomation
			at_cifsr_send();
			at_state_rx = STATE_AT_CIFSR_TX;
		break;
		case STATE_AT_CIPSTATUS_TX:// 7 send get udp connect infomation
			at_cipstatus_send();
			at_state_rx = STATE_AT_CIPSTATUS_TX;
		break;
		case STATE_AT_CWJAP_TX:// 8 send connect wifi ssid pwd
			at_cwjap_send();
			at_state_rx = STATE_AT_CWJAP_TX;
		break;
		case STATE_AT_CIPSTART_TX:// 9 send start udp connect
			at_cipstart_send();
			at_state_rx = STATE_AT_CIPSTART_TX;
		break;
		case STATE_AT_CIPSEND_TX:// 10 send start transfer data
			at_cipsend_send();
			at_state_rx = STATE_AT_CIPSEND_TX;
		break;
		case STATE_AT_CIPTRANS_TX:// 11 transfer data
			at_modbus_send_data();
			at_state_rx = STATE_AT_CIPTRANS_TX;
		break;
		case STATE_AT_PLUS3_TX:// 12 send +++ quit transfer mode
			at_plus3_send();
			at_state_rx = STATE_AT_PLUS3_TX;
		break;

		default:break;
	}

	at_state_tx = STATE_AT_NONE;

}

static uint8_t str_cmp(char* srcstr, char* dstchar)
{
	char* src1 = NULL;
	char* src2 = NULL;
	char* sub = NULL;

	src1 = srcstr;
	while (*src1 != '\0')
	{
		src2 = src1;
		sub = dstchar;
		do
		{
			if ((*sub == '\n') || (*sub == '\0'))
			{
				return 1;//找到子串
			}
		} while (*src2++ == *sub++);

		src1++;
	}

	return 0;
}

static uint8_t str_cmp2(char* srcstr, char* dstchar)
{
	char* src1 = NULL;
	char* src2 = NULL;
	char* sub = NULL;

	src1 = srcstr;
	while (*src1 != '\0')
	{
		src2 = src1;
		sub = dstchar;
		do
		{
			if (*sub == ',')
			{
				return 1;//找到子串
			}
		} while (*src2++ == *sub++);

		src1++;
	}

	return 0;
}

static uint8_t string_compare(char* srcstr, char* dstchar1, char* dstchar2, char* dstchar3, char* dstchar4 )
{
	uint8_t i = 0;
	uint8_t ret = 0;

	if(NULL != dstchar1)
	{
		if(str_cmp(srcstr, dstchar1))
			ret |= 0x01;
	}

	if(NULL != dstchar2)
	{
		if(str_cmp(srcstr, dstchar2))
			ret |= 0x02;
	}

	if(NULL != dstchar3)
	{
		if(str_cmp(srcstr, dstchar3))
			ret |= 0x04;
	}

	if(NULL != dstchar4)
	{
		if(str_cmp(srcstr, dstchar4))
			ret |= 0x08;
	}

	return ret;
}

STATE_t at_cwmode_get_proc(uint8_t * rxbuf)
{
	STATE_t state_tx = STATE_AT_NONE;

	if(string_compare(rxbuf, at_cwmode_val1_str, NULL, NULL, NULL))     // station
		state_tx = STATE_AT_CIPMODE_GET_TX;
	else if(string_compare(rxbuf, at_cwmode_val2_str, at_cwmode_val3_str, NULL, NULL)) // AP , station-AP
		state_tx = STATE_AT_CWMODE_SET_TX;
	else
	{
		vTaskDelay(3000);
		state_tx = STATE_AT_CWMODE_GET_TX;
	}

	return state_tx;
}

STATE_t at_cipmode_get_proc(uint8_t * rxbuf)
{
	STATE_t state_tx = STATE_AT_NONE;

	if(string_compare(rxbuf, at_cipmode_val1_str, NULL, NULL, NULL))     // transfer
		state_tx = STATE_AT_CIFSR_TX;
	else if(string_compare(rxbuf, at_cipmode_val0_str, NULL, NULL, NULL)) // AT
		state_tx = STATE_AT_CIPMODE_SET_TX;
	else
	{
		vTaskDelay(3000);
		state_tx = STATE_AT_CIPMODE_GET_TX;
	}

	return state_tx;
}
STATE_t at_cifsr_proc(uint8_t * rxbuf)
{
	STATE_t state_tx = STATE_AT_NONE;

	if(string_compare(rxbuf, at_cifsr_ack_ip_str, NULL, NULL, NULL))
	{
		vTaskDelay(3000);
		state_tx = STATE_AT_CWJAP_TX;

	}
	else if(string_compare(rxbuf, at_ok_str, "+CIFSR:STAIP", NULL, NULL))
	{
		//vTaskDelay(3000);
		state_tx = STATE_AT_CIPSTATUS_TX;
	}

	return state_tx;
}


STATE_t at_cipstatus_proc(uint8_t * rxbuf)
{
	STATE_t state_tx = STATE_AT_NONE;

	if(string_compare(rxbuf, at_wifi_status2_str,  NULL, NULL, NULL)) // 2 connect AP, 4 disconnect TCP
	{
		if(string_compare(rxbuf, NULL, "UDP",  NULL, NULL)) //UDP
				state_tx = STATE_AT_CIPSEND_TX;
		else
		{
			vTaskDelay(3000);
			state_tx = STATE_AT_CIPSTART_TX;
		}
	}
	else if(string_compare(rxbuf, at_wifi_status3_str, at_conn_str, NULL, NULL)) // 3 connect TCP
		state_tx = STATE_AT_CIPSEND_TX;
	else if(string_compare(rxbuf, at_wifi_status4_str, at_wifi_status5_str, NULL, NULL)) // 5 disconnect AP
		state_tx = STATE_AT_CWJAP_TX;


	return state_tx;
}


STATE_t at_cipsend_proc(uint8_t * rxbuf)
{
	STATE_t state_tx = STATE_AT_NONE;

	if(string_compare(rxbuf, at_ok_str, NULL, at_busy_str, NULL)) //
	{
		at_state_rx = STATE_AT_CIPTRANS_TX;
	}
	else
	{
		state_tx = STATE_AT_CIPMODE_GET_TX;
	}


	return state_tx;
}

//00 01 46 2E 62 69 6E 00 6F 63 74 65 74 00 F.bin.octet.
//00 02 63 6C 6F 75 64 2E 62 69 6E 00 6F 63 74 65 74 00  cloud.bin.octet.
//00 03 00 01
//00 04 00 01
//00 03 00 02
//00 04 00 02
//00 05 00 01 file not found

void at_modbus_recv_data(uint8_t * rxbuf, uint16_t rxlen)
{
	at_txsize = 0;
	bzero(modbus_tx_buf, sizeof(modbus_tx_buf));

	if((rxbuf[0] == 0x00) && (rxbuf[1] == 0x01))
	{
		if(rxbuf[2] == 'E')
				wifi_cmdflag = 1;
		else if(rxbuf[2] == 'R')
			wifi_cmdflag = 2;
		//else if(rxbuf[2] == 'W')
		//	wifi_cmdflag = 3;
		else if(rxbuf[2] == 'M')
			wifi_cmdflag = 4;
		else if(rxbuf[2] == 'D')
			wifi_cmdflag = 5;
		else if(rxbuf[2] == 'F')
			wifi_cmdflag = 6;
		else if(rxbuf[2] == 'A')
			wifi_cmdflag = 7;
		//else if(rxbuf[2] == 'S')
			//wifi_cmdflag = 8;

		modbus_tx_buf[0] = 0x00;
		modbus_tx_buf[1] = 0x03;
		modbus_tx_buf[2] = 0x00;
		modbus_tx_buf[3] = 0x01;
		if(wifi_cmdflag == 0x02)
		{
			wifi_sn = 0;
			modbus_tx_buf[3] = 0x00;
			memcpy(modbus_tx_buf + 4, FLASH_PROGRAM_ADDR, 512);
			at_txsize = 512 + 4;
			printf("wifi_cmdflag start read %s\n", rxbuf+2);
		}
		else if(wifi_cmdflag == 0x07)
		{
			wifi_sn = 0;
			modbus_tx_buf[3] = 0x00;
			memcpy(modbus_tx_buf + 4, FLASH_PROGRAM_ADDR, 512);
			at_txsize = 512 + 4;
			printf("wifi_cmdflag start read all %s\n", rxbuf+2);
		}
		else if(wifi_cmdflag == 0x04){
			printf("wifi_cmdflag start md5 checksum\n");
			memcpy(modbus_tx_buf + 4, "wifi_md5\n", 9);
			at_txsize = 9 + 4;
		}
		else if(wifi_cmdflag == 0x05){
			printf("wifi_cmdflag return md5 checksum\n");
			for(int i=0; i<16; i++) {
				printf("%02x", wifi_decrypt[i]);
			}
			printf("\n");
			memcpy(modbus_tx_buf + 4, wifi_decrypt, 16);
			at_txsize = 16 + 4;
		}
		else if(wifi_cmdflag == 0x06){
			uint32_t filebytes = 0;
			uint8_t numbuf[12];
			bzero(numbuf, sizeof(numbuf));
			filebytes = *(__IO uint32_t*)(FLASH_FILESIZE_ADDR);
			sprintf(numbuf, "%d\n", filebytes);
			printf("wifi filebytes = %d\n", filebytes, filebytes);
			memcpy(modbus_tx_buf + 4, numbuf, strlen(numbuf));
			at_txsize = strlen(numbuf);
			at_txsize = at_txsize + 4;
		}
		else if(wifi_cmdflag == 0x01){
			memcpy(modbus_tx_buf + 4, "wifi_erase\n", 11);
			at_txsize = 11 + 4;
			printf("wifi start erase\n");
		}
	}
	else if((rxbuf[0] == 0x00) && (rxbuf[1] == 0x02))
	{
		if(rxbuf[2] == 'W')
			wifi_cmdflag = 3;
		else if(rxbuf[2] == 'S')
			wifi_cmdflag = 8;

		printf("wifi file:%c wifi_cmdflag=%d\n", rxbuf[2], wifi_cmdflag);
		modbus_tx_buf[0] = 0x00;
		modbus_tx_buf[1] = 0x04;
		modbus_tx_buf[2] = 0x00;
		modbus_tx_buf[3] = 0x00;
		at_txsize = 4;
		wifi_wsn = 0;
	}
	else if((rxbuf[0] == 0x00) && (rxbuf[1] == 0x03))
	{
		modbus_tx_buf[0] = 0x00;
		modbus_tx_buf[1] = 0x04;
		modbus_tx_buf[2] = rxbuf[2];
		modbus_tx_buf[3] = rxbuf[3];
		if(wifi_cmdflag == 0x03){
			wifi_ota_flag = 0x33;
			at_txsize = 4;
		}
		else if(wifi_cmdflag == 0x08){
			wifi_ota_flag = 0x77;
			at_txsize = 4;
		}
		else
		{
			if((wifi_wsn + 1) == (rxbuf[2] * 256 + rxbuf[3]))
			{
				wifi_wsn = wifi_wsn + 1;
				uint32_t datalen = rxlen - 4;
				memcpy(wifi_tftpbuf, rxbuf + 4, datalen);
				//printf("wifi_count = %d datalen = %d\n", wifi_count, datalen);
				wifi_flashaddr = writedata_to_flash(wifi_tftpbuf, datalen, wifi_flashaddr);
				wifi_count = wifi_count + datalen;
				if(datalen < 512)
				{
					printf("wifi_wsn = %d wifi_count = %d datalen = %d\n", wifi_wsn, wifi_count, datalen);
					wifi_ota_flag = 0x3F;
					wifi_writesize = wifi_count;
					wifi_readsize = wifi_count;
					wifi_count = 0;
					wifi_cmdflag = 0;
				}

			}
			else
				printf("wifi_wsn = %d\n", wifi_wsn);
		}

		at_txsize = 4;

	}
	else if((rxbuf[0] == 0x00) && (rxbuf[1] == 0x04))
	{
		if((wifi_cmdflag == 0x02) || (wifi_cmdflag == 0x07))
		{
			//printf("ack rxbuf[2]=%d rxbuf[3]=%d wifi_cmdflag = %d \n", rxbuf[2], rxbuf[3], wifi_cmdflag);
			if(wifi_cmdflag == 0x02)
			{
				if(wifi_sn == (rxbuf[2] * 256 + rxbuf[3]))
				{
					wifi_sn++;
					wifi_readaddr += 512;
				}
				modbus_tx_buf[0] = 0x00;
				modbus_tx_buf[1] = 0x03;
				modbus_tx_buf[2] = wifi_sn / 256;
				modbus_tx_buf[3] = wifi_sn % 256;
				if(wifi_readaddr < (wifi_readsize - 512) )
				{
					memcpy(modbus_tx_buf + 4, FLASH_PROGRAM_ADDR + wifi_readaddr, 512);
					at_txsize = 512 + 4;
					//printf("2 wifi_readaddr = %d , wifi_readsize = %d\n", wifi_readaddr, wifi_readsize);
				}
				else if (wifi_readaddr < wifi_readsize)
				{
					memcpy(modbus_tx_buf + 4, FLASH_PROGRAM_ADDR + wifi_readaddr, wifi_readsize - wifi_readaddr);
					at_txsize = wifi_readsize - wifi_readaddr +4;
					printf("2 wifi_readaddr = %d , at_txsize = %d\n", wifi_readaddr, at_txsize);
					wifi_readaddr = 0;
					wifi_cmdflag = 0;
					wifi_sn  = 0;
				}
				else
				{
					printf("2 wifi_sn = %d\n", wifi_sn); // last ack no return data
					at_txsize = 0;
				}
			}

			else if(wifi_cmdflag == 0x07)
			{
				if(wifi_sn == (rxbuf[2] * 256 + rxbuf[3]))
				{
					wifi_sn++;
					wifi_readaddr += 512;
				}
				modbus_tx_buf[0] = 0x00;
				modbus_tx_buf[1] = 0x03;
				modbus_tx_buf[2] = wifi_sn / 256;
				modbus_tx_buf[3] = wifi_sn % 256;
				if(wifi_readaddr < (0xE0000 - 512) )
				{
					memcpy(modbus_tx_buf + 4, FLASH_PROGRAM_ADDR + wifi_readaddr, 512);
					at_txsize = 512 + 4;
				}
				else if (wifi_readaddr < wifi_readsize)
				{
					memcpy(modbus_tx_buf + 4, FLASH_PROGRAM_ADDR + wifi_readaddr, 0xE0000 - wifi_readaddr - 1);
					at_txsize = 0xE0000 - wifi_readaddr - 1 + 4;
					printf("7 wifi_readaddr = %d , at_txsize = %d\n", wifi_readaddr, at_txsize);
					wifi_readaddr = 0;
					wifi_cmdflag = 0;
					wifi_sn  = 0;
				}
				else
				{
					printf("7 wifi_sn = %d\n", wifi_sn); // last ack no return data
					at_txsize = 0;
				}
			}

		}
	}
	else if((rxbuf[0] == 0x00) && (rxbuf[1] == 0x05))
	{
		printf("wifi tftp error\n");
	}
	//memcpy(modbus_tx_buf, rxbuf, at_txsize);
	at_state_tx = STATE_AT_CIPTRANS_TX;


}

void wifi_at_proc(uint8_t * rxbuf, uint16_t rxlen)
{
	switch(at_state_rx)
	{
		case STATE_AT_3PLUS_TX:// 0
			at_state_tx = STATE_AT_RST_TX;
		break;
		case STATE_AT_CWMODE_GET_TX:// 2
			at_state_tx = at_cwmode_get_proc(rxbuf);
		break;
		case STATE_AT_CWMODE_SET_TX:// 3
			vTaskDelay(3000);
			at_state_tx = STATE_AT_CWMODE_GET_TX;
		break;
		case STATE_AT_CIPMODE_GET_TX:// 4
			at_state_tx = at_cipmode_get_proc(rxbuf);
		break;
		case STATE_AT_CIPMODE_SET_TX:// 5
			vTaskDelay(3000);
			at_state_tx = STATE_AT_CIPMODE_GET_TX;
		break;
		case STATE_AT_CIFSR_TX:// 6
			at_state_tx = at_cifsr_proc(rxbuf);
		break;
		case STATE_AT_CIPSTATUS_TX:// 7
			at_state_tx = at_cipstatus_proc(rxbuf);
		break;
		case STATE_AT_CWJAP_TX:// 8
			at_state_tx = STATE_AT_CIPSTATUS_TX;
		break;
		case STATE_AT_CIPSTART_TX:// 9
			at_state_tx = STATE_AT_CIPSTATUS_TX;
		break;
		case STATE_AT_CIPSEND_TX:// 10
			at_state_tx = at_cipsend_proc(rxbuf);
		break;
		case STATE_AT_CIPTRANS_TX:// 11
			at_modbus_recv_data(rxbuf, rxlen);
		break;
		case STATE_AT_PLUS3_TX:// 12
			at_state_rx = STATE_AT_CIPSTATUS_TX;
		break;
		default:break;
	}

	if(STATE_AT_CIPTRANS_TX > at_state_rx)
	{
		if(string_compare(rxbuf, at_closed_str, at_error_str, at_wifi_getip_str, at_conn_str)) // tcp close
			at_state_tx = STATE_AT_CIPMODE_GET_TX;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  BaseType_t  ret;
  ret = xTaskCreate(wifiClientTask, "wifiClientTask", 512, NULL, 6, &wifiClientTaskHandle);
  if (ret != pdTRUE)
  {
    printf("wifiClientTask create err\r\n");
  }

    ota_flag = *(__IO uint8_t*)(FLASH_OTAFLAG_ADDR);
    readsize = *(__IO uint32_t*)(FLASH_FILESIZE_ADDR);
    printf("reset ota_flag = %d\n", ota_flag);
    printf("reset readsize = %d\n", readsize);

  tftp_init(&tftp);


  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);  	osDelay(250);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);  	osDelay(250);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);  	osDelay(250);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);  	osDelay(250);
#if 1
  /* Infinite loop */
  uint8_t cnt = 0;
  for(;;)
  {
	 osDelay(250);

     if(cmdflag == 0x01){
		 ota_flag = 0x30;
		 cmdflag = 0;
	 }

     if(cmdflag == 0x04){
		int i, k, len;
		unsigned char encrypt[1024];
		MD5_CTX md5;

		readsize = *(__IO uint32_t*)(FLASH_FILESIZE_ADDR);
		printf("readsize= %d\n", readsize);

		if(readsize <= 0x00040000){
			printf("md5 checking\n");
			MD5Init(&md5);

			for(i = 0; i < readsize - 1024; i = i + 1024){
				memcpy(encrypt, (uint8_t*)(FLASH_PROGRAM_ADDR + i), 1024);
				//for(k=0; k<1024; k++) {
				//	printf("%02x", encrypt[k]);
				//}
				//printf(" i = %d\n", i);
				MD5Update(&md5, encrypt, 1024);
			}
			printf("i = %d\n", i);
			printf("readsize - i = %d\n", readsize - i);

			memcpy(encrypt, (uint8_t*)(FLASH_PROGRAM_ADDR + i), readsize - i);
			for(k=0; k<(readsize - i); k++) {
				printf("%02x", encrypt[k]);
			}
			printf(" i = %d\n", i);
			MD5Update(&md5, encrypt, readsize - i);

			MD5Final(&md5, decrypt);
		}

		for(i=0; i<16; i++) {
			printf("%02x", decrypt[i]);
		}
		printf("\n");

		cmdflag = 0;
     }

	 if(ota_flag == 0x11)
	 {
		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);  	osDelay(250);
		 erase_flash(5, 6);
		 printf("erase ok block 5 and block 6 \n");
		 ota_flag = *(__IO uint8_t*)(FLASH_OTAFLAG_ADDR);
		 printf("ota_flag = %X\n", ota_flag);
	 }
	 else if(ota_flag == 0xFF)
	 {
		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);  osDelay(250);
		 ota_flag = 0x7F;
		 printf("wait write or read ok \n");
	 }
	 else if(ota_flag == 0x7F)
	 {
		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);  osDelay(250);
		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);  osDelay(250);
	 }
	 else if(ota_flag == 0x3F)
	 {
		 printf("writesize = %d\n", writesize);
		 writedata_to_flash(&writesize, 4, FLASH_FILESIZE_ADDR);
		 printf("readsize = %d\n", readsize);
		 readsize = *(__IO uint32_t*)(FLASH_FILESIZE_ADDR);
		 printf("readsize = %d\n", readsize);
		 ota_flag = 0x37;
	 }
	 else if(ota_flag == 0x37)
	 {
		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);  osDelay(250);
		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);  osDelay(250);
	 }
	 else if(ota_flag == 0x33)
	 {
		ota_flag = *(__IO uint8_t*)(FLASH_OTAFLAG_ADDR);
		printf("read ota_flag = %X\n", ota_flag);
		ota_flag = 0x33;
		writedata_to_flash(&ota_flag, 1, FLASH_OTAFLAG_ADDR);
		ota_flag = *(__IO uint8_t*)(FLASH_OTAFLAG_ADDR);
		printf("write ota_flag = %X\n", ota_flag);
		printf("reset device for bootloader to app\n");
		osDelay(1000);printf("3\n");
		osDelay(1000);printf("2\n");
		osDelay(1000);printf("1\n");
		osDelay(1000);printf("0\n");
		osDelay(1000);printf(".\n");
		HAL_NVIC_SystemReset();
	 }
	 else if(ota_flag == 0x77)
	 {
		printf("reset device\n");
		osDelay(1000);printf("3\n");
		osDelay(1000);printf("2\n");
		osDelay(1000);printf("1\n");
		osDelay(1000);printf("0\n");
		osDelay(1000);printf(".\n");
		HAL_NVIC_SystemReset();
	 }
	 else
	 {
		 ota_flag = 0x11;
	 }


  }
#else
  while(1){osDelay(200);}
#endif
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

