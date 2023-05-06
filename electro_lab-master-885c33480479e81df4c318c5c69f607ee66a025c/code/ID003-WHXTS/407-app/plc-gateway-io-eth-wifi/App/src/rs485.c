/*
 * rs485.c
 *
 *  Created on: Oct 8, 2021
 *      Author: Administrator
 */
#include "rs485.h"
#include "gpio.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "sys/types.h"
#include "sys/socket.h"
#include <lwip/netdb.h>
#include "lwip/netif.h"
#include "lwip/netif.h"
#include "boardConfig.h"
#include "cJSON.h"



#define  SLAVE_01     0x01
#define  SLAVE_02     0x02
#define  SLAVE_03     0x03
#define  SLAVE_04     0x04
#define  SLAVE_05     0x05

#define TYPE_SET_OPT_REQ       3
#define TYPE_SET_OPT_REQ_ACK   4
#define TYPE_SET_SUCCESS       5
#define TYPE_SET_SUCCESS_ACK   6

//#define PRINTF

double    ADS_K1 = 0.0009314;
double	  ADS_B1 = 374.7;
double    ADS_K2 = 0.0009314;
double    ADS_B2 = 374.7;

float  x_value, y_value;
double slave2_value_ch1;
double slave2_value_ch2;
double slave3_value_ch1;
double slave3_value_ch2;
double slave4_value_ch1;
double slave4_value_ch2;
double slave5_value_ch1;
double slave5_value_ch2;

int set_zero_sock;
//set_zero_sta = 0: 发送请求是否设置零点
//set_zero_sta = 1: 发送设置零点成功
static char set_zero_sta;
static uint8_t  set_zero_tcpbuf[1500];
extern TaskHandle_t   rs485TaskHandle;

//字节数据转float
static float ByteToFloat(uint8_t *data)
{
	float f;
	unsigned char *p = (unsigned char*)&f;
 	p[3] = *data;
	p[2] = *(data+1);
	p[1] = *(data+2);
    p[0] = *(data+3);
	return f;
}

static uint16_t ModBusCRC (uint8_t *ptr,uint8_t size)
{
	unsigned short a,b,tmp,CRC16,V;
	CRC16=0xffff;
	for (a=0;a<size;a++)
	{
		CRC16=*ptr^CRC16;
		for (b=0;b<8;b++)
		{
			tmp=CRC16 & 0x0001;
			CRC16 =CRC16 >>1;
			if (tmp)
			CRC16=CRC16 ^ 0xa001;
		}
		*ptr++;
	}
	V = ((CRC16 & 0x00FF) << 8) | ((CRC16 & 0xFF00) >> 8) ;//éŤä˝ĺ­čč˝Źć˘
	return V;
}




static void recvHandler(uint8_t *data)
{
//	int i;
	uint8_t   crc_h, crc_l;
	uint16_t  crc;

	int32_t   slave2_ch1 = 0;
	int32_t   slave2_ch2 = 0;
	int32_t   slave3_ch1 = 0;
	int32_t   slave3_ch2 = 0;
	int32_t   slave4_ch1 = 0;
 	int32_t   slave4_ch2 = 0;
	int32_t   slave5_ch1 = 0;
	int32_t   slave5_ch2 = 0;



	switch (data[0])
	{
		case SLAVE_01://角度传感器地址为1
		{
			crc = ModBusCRC(data, 11);   //printf("LINE:%d  crc = %x\r\n",__LINE__, crc);
			crc_h = (unsigned char)(crc>>8);        //printf("LINE:%d  crc_h = %x\r\n",__LINE__, crc_h);
			crc_l = (unsigned char)(crc&0xff);      //printf("LINE:%d  crc_l = %x\r\n",__LINE__, crc_l);
			if( crc_h == data[11] && crc_l == data[12] )
			{
				x_value = ByteToFloat(&data[3]);
				y_value = ByteToFloat(&data[7]);
			}
#if LOG_PRINTF_DEF
			snprintf(log_msg_global, sizeof(log_msg_global),"x_value : %lf    y_value : %lf",x_value, y_value);
			LogPrint(log_msg_global);
#else
			printf("x_value : %lf    y_value : %lf\r\n",x_value, y_value);
#endif

			break;
		}

		case SLAVE_02://地址为2  //slave2_ch1  slave2_ch2
		{
			crc = ModBusCRC(data, 9);    //printf("LINE:%d  crc = %x\r\n",__LINE__, crc);
			crc_h = (unsigned char)(crc>>8);        //printf("LINE:%d  crc_h = %x\r\n",__LINE__, crc_h);
			crc_l = (unsigned char)(crc&0xff);      //printf("LINE:%d  crc_l = %x\r\n",__LINE__, crc_l);
			if( crc_h == data[9] && crc_l == data[10] )
			{
				slave2_ch1 = data[3] << 16 | data[4] << 8 | data[5];
				slave2_ch2 = data[6] << 16 | data[7] << 8 | data[8];
			}


			if( slave2_ch1 > 0x800000 ) //如果xxxx大于2^23, 则实际值=xxxx-2^24, 否则的话实际值=xxxx
			{
				slave2_ch1 -= 0x1000000;
			}
			if( slave2_ch2 > 0x800000)
			{
				slave2_ch2 -= 0x1000000;
			}

			slave2_value_ch1 = ADS_K1*slave2_ch1 + ADS_B1;
			slave2_value_ch2 = ADS_K2*slave2_ch2 + ADS_B2;

#if LOG_PRINTF_DEF
			snprintf(log_msg_global, sizeof(log_msg_global),"slave2_value_ch1 : %lf    slave2_value_ch2 : %lf",slave2_value_ch1, slave2_value_ch2);
			LogPrint(log_msg_global);
#else
			printf("slave2_value_ch1 : %lf    slave2_value_ch2 : %lf\r\n",slave2_value_ch1, slave2_value_ch2);
#endif


			if(slave2_value_ch1 > 2000)
			{
				printf("slave2_value_ch1 : %lf    slave2_value_ch2 : %lf\r\n",slave2_value_ch1, slave2_value_ch2);
				printf("slave2_ch1 : %d    slave2_ch2 : %d\r\n", (int)slave2_ch1, (int)slave2_ch2);
				printf("err hex:");
				printf("%02x %02x %02x\r\n", data[3],  data[4],  data[5]);
			}
			break;
		}

		case SLAVE_03://地址为3 //slave3_ch1 slave3_ch2
		{
			crc = ModBusCRC(data, 9);    //printf("LINE:%d  crc = %x\r\n",__LINE__, crc);
			crc_h = (unsigned char)(crc>>8);        //printf("LINE:%d  crc_h = %x\r\n",__LINE__, crc_h);
			crc_l = (unsigned char)(crc&0xff);      //printf("LINE:%d  crc_l = %x\r\n",__LINE__, crc_l);
			if( crc_h == data[9] && crc_l == data[10] )
			{
				slave3_ch1 = data[3] << 16 | data[4] << 8 | data[5];
				slave3_ch2 = data[6] << 16 | data[7] << 8 | data[8];
			}


			if( slave3_ch1 > 0x800000 ) //如果xxxx大于2^23, 则实际值=xxxx-2^24, 否则的话实际值=xxxx
			{
				slave3_ch1 -= 0x1000000;
			}
			if( slave3_ch2 > 0x800000)
			{
				slave3_ch2 -= 0x1000000;
			}

			slave3_value_ch1 = ADS_K1*slave3_ch1 + ADS_B1;
			slave3_value_ch2 = ADS_K2*slave3_ch2 + ADS_B2;



#if LOG_PRINTF_DEF
			snprintf(log_msg_global, sizeof(log_msg_global),"slave3_value_ch1 : %lf    slave3_value_ch2 : %lf",slave3_value_ch1, slave3_value_ch2);
			LogPrint(log_msg_global);
#else
			printf("slave3_value_ch1 : %lf    slave3_value_ch2 : %lf\r\n",slave3_value_ch1, slave3_value_ch2);
#endif


			if(slave3_value_ch1 > 2000)
			{
				printf("slave3_value_ch1 : %lf    slave3_value_ch2 : %lf\r\n",slave3_value_ch1, slave3_value_ch2);
				printf("slave3_ch1 : %d    slave3_ch2 : %d\r\n", (int)slave3_ch1, (int)slave3_ch2);
				printf("err hex:");
				printf("%02x %02x %02x\r\n", data[3],  data[4],  data[5]);
			}
			break;
		}

		case SLAVE_04://地址为4 slave4_value_ch1 slave4_ch2
		{
			crc = ModBusCRC(data, 9);    //printf("LINE:%d  crc = %x\r\n",__LINE__, crc);
			crc_h = (unsigned char)(crc>>8);        //printf("LINE:%d  crc_h = %x\r\n",__LINE__, crc_h);
			crc_l = (unsigned char)(crc&0xff);      //printf("LINE:%d  crc_l = %x\r\n",__LINE__, crc_l);

			if( crc_h == data[9] && crc_l == data[10] )
			{
				slave4_ch1 = data[3] << 16 | data[4] << 8 | data[5];
				slave4_ch2 = data[6] << 16 | data[7] << 8 | data[8];
			}
			if( slave4_ch1 > 0x800000 ) //如果xxxx大于2^23, 则实际值=xxxx-2^24, 否则的话实际值=xxxx
			{
				slave4_ch1 -= 0x1000000;
			}
			if( slave4_ch2 > 0x800000)
			{
				slave4_ch2 -= 0x1000000;
			}

			slave4_value_ch1 = ADS_K1*slave4_ch1 + ADS_B1;
			slave4_value_ch2 = ADS_K2*slave4_ch2 + ADS_B2;

#if LOG_PRINTF_DEF
			snprintf(log_msg_global, sizeof(log_msg_global),"slave4_value_ch1 : %lf    slave4_value_ch2 : %lf",slave4_value_ch1, slave4_value_ch2);
			LogPrint(log_msg_global);
#else
			printf("slave4_value_ch1 : %lf    slave4_value_ch2 : %lf\r\n",slave4_value_ch1, slave4_value_ch2);
#endif


			break;
		}

		case SLAVE_05://地址为5 slave5_value_ch1 slave5_ch2
		{
			crc = ModBusCRC(data, 9);    //printf("LINE:%d  crc = %x\r\n",__LINE__, crc);
			crc_h = (unsigned char)(crc>>8);        //printf("LINE:%d  crc_h = %x\r\n",__LINE__, crc_h);
			crc_l = (unsigned char)(crc&0xff);      //printf("LINE:%d  crc_l = %x\r\n",__LINE__, crc_l);

			if( crc_h == data[9] && crc_l == data[10] )
			{
				slave5_ch1 = data[3] << 16 | data[4] << 8 | data[5];
				slave5_ch2 = data[6] << 16 | data[7] << 8 | data[8];
			}
			if( slave5_ch1 > 0x800000 ) //如果xxxx大于2^23, 则实际值=xxxx-2^24, 否则的话实际值=xxxx
			{
				slave5_ch1 -= 0x1000000;
			}
			if( slave5_ch2 > 0x800000)
			{
				slave5_ch2 -= 0x1000000;
			}

			slave5_value_ch1 = ADS_K1*slave5_ch1 + ADS_B1;
			slave5_value_ch2 = ADS_K2*slave5_ch2 + ADS_B2;

#if LOG_PRINTF_DEF
			snprintf(log_msg_global, sizeof(log_msg_global),"slave5_value_ch1 : %lf    slave5_value_ch2 : %lf\r\n",slave5_value_ch1, slave5_value_ch2);
			LogPrint(log_msg_global);
#else
			printf("slave5_value_ch1 : %lf    slave5_value_ch2 : %lf\r\n",slave5_value_ch1, slave5_value_ch2);
#endif


			break;
		}
	}
}

#define TOF 0
//修改ads采集板子地址 ： FF 06 01 01 00 xx crc16h crc16l

extern uint8_t tcp_send_flag;
extern uint8_t tcp_recv_flag;
extern int16_t acc_txbuf[3 * 100];
extern int16_t acc_rxbuf[3 * 100];
extern volatile  uint8_t   uart4_send_start_flag;
extern uint8_t modbus_tx_buf[1206];
extern uint16_t at_txsize;
uint32_t sncnt = 0;
__IO ITStatus UartReady = RESET;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == USART2)
	{
		UartReady = SET;
		//rxcount++;
	}
}
extern uint32_t rx_sn;
void rs485DataHandler(void *pvParameters)
{
	int16_t accx = 0;
	int16_t accy = 0;
	int16_t accz = 0;
	uint16_t cnt = 0;
	uint32_t rxcnt = 0;
	uint16_t k = 0;
	vTaskDelay(1000);

	while(1)
	{
//		if(tcp_send_flag == 1)
//		  {
//			   bzero(uart2_rx_buffer, sizeof(uart2_rx_buffer));
//			   if(HAL_UART_Receive_DMA(&huart2, uart2_rx_buffer, UART2_BUF_SIZE) != HAL_OK)
//			   {
//				 Error_Handler();
//			   }
//
//			   while (UartReady != SET);
//			   UartReady = RESET;
//
//			   for(k = 0; k < sizeof(uart2_rx_buffer);)
//			   {
//					if((uart2_rx_buffer[k + 0] == 0x68) && (uart2_rx_buffer[k + 1] == 0x0D))
//					{
//						if((uart2_rx_buffer[k + 4] % 0x10) > 0x03)
//							accx = 30000;
//						else
//							accx = (uart2_rx_buffer[k + 4] % 0x10) * 10000 + (uart2_rx_buffer[k + 5] / 0x10) * 1000 + (uart2_rx_buffer[k + 5] % 0x10) * 100 + (uart2_rx_buffer[k + 6] / 0x10) * 10 + uart2_rx_buffer[k + 6] % 0x10;
//
//						if((uart2_rx_buffer[k + 7] % 0x10) > 0x03)
//							accx = 30000;
//						else
//							accy = (uart2_rx_buffer[k + 7] % 0x10) * 10000 + (uart2_rx_buffer[k + 8] / 0x10) * 1000 + (uart2_rx_buffer[k + 8] % 0x10) * 100 + (uart2_rx_buffer[k + 9] / 0x10) * 10 + uart2_rx_buffer[k + 9] % 0x10;
//
//						if((uart2_rx_buffer[k + 10] % 0x10) > 0x03)
//							accx = 30000;
//						else
//							accz = (uart2_rx_buffer[k + 10] % 0x10) * 10000 + (uart2_rx_buffer[k + 11] / 0x10) * 1000 + (uart2_rx_buffer[k + 11] % 0x10) * 100 + (uart2_rx_buffer[k + 12] / 0x10) * 10 + uart2_rx_buffer[k + 12] % 0x10;
//
//						if(uart2_rx_buffer[k + 4] > 0x09)
//						{
//							accx = -accx;
//						}
//
//						if(uart2_rx_buffer[k + 7] > 0x09)
//						{
//							accy = -accy;
//						}
//
//						if(uart2_rx_buffer[k + 10] > 0x09)
//						{
//							accy = -accy;
//						}
//
//						acc_txbuf[rxcnt + 0] = accx;
//						acc_txbuf[rxcnt + 1] = accy;
//						acc_txbuf[rxcnt + 2] = accz;
//						rxcnt += 3;
//						if(rxcnt == 300)
//						{
//							modbus_tx_buf[0] = 0x33;
//							modbus_tx_buf[1] = 0x55;
//							at_txsize = 606;
//							*(uint32_t*)(modbus_tx_buf + 2) = sncnt++;
//							memcpy(modbus_tx_buf+6, acc_txbuf, sizeof(acc_txbuf));
//							//wifi_uart_send(modbus_tx_buf, at_txsize);
//							//HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer, sizeof(uart4_rx_buffer) );
//							if(sncnt > (rx_sn + 10))
//							{
//								printf("tcp timeout reset device\n");
//								vTaskDelay(100);
//								printf("tcp timeout reset device\n");
//								vTaskDelay(100);
//								printf("tcp timeout reset device\n");
//								vTaskDelay(100);
//								HAL_NVIC_SystemReset();
//						   }
//							else
//								printf("sncnt = %d rx_sn = %d \n", sncnt, rx_sn);
//							rxcnt = 0;
//						}
//						//printf("accx = %d accy = %d accz = %d cnt = %d\n", accx, accy, accz, rxcnt);
//						k = k + 14;
//					}
//					else
//						k++;
//
//			   }
//
//		  }
//		  else
//		  {
//			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);  	osDelay(100);
//			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);  	osDelay(100);
//		  }
		osDelay(1000);
	}

}




#define SYSTEM_TRUE 0x00
#define SIGNAL_LOW 0x01
#define SIGNAL_HIGH 0x02
#define TOOLONG 0x03
#define SYSTEM_ERROR 0x04

void LP20_Data_Handle(){
	//先校验
	uint8_t crc_handle[5] = {uart2_rx_buffer[1],uart2_rx_buffer[2],uart2_rx_buffer[3],uart2_rx_buffer[4],uart2_rx_buffer[5]};
	uint8_t ret = crc_high_first(crc_handle,sizeof(crc_handle));
//	if(ret != uart2_rx_buffer[6]){
//		LP20_TOF.Data_Value_Sum = 0;
//		return 0;
//	}
	LP20_TOF.Data_Head = uart2_rx_buffer[0];
	LP20_TOF.Data_Key = uart2_rx_buffer[1];
	LP20_TOF.Data_Value[0] = uart2_rx_buffer[2];
	LP20_TOF.Data_Value[1] = uart2_rx_buffer[3];
	LP20_TOF.Data_Value[2] = uart2_rx_buffer[4];
	LP20_TOF.Data_Value[3] = uart2_rx_buffer[5];
	LP20_TOF.Data_Crc = uart2_rx_buffer[6];
	LP20_TOF.Data_End = uart2_rx_buffer[7];
	LP20_TOF.Data_Value_Sum = ((LP20_TOF.Data_Value[1] << 8 | LP20_TOF.Data_Value[2]) << 8) | LP20_TOF.Data_Value[3];
	printf("SUM:%d\r\n",LP20_TOF.Data_Value_Sum);
//	if(LP20_TOF.Data_Head != 0x55){printf("HEAD ERROR\r\n");}
#if TOF
	switch(LP20_TOF.Data_Value[0]){
//	case SYSTEM_TRUE:printf("SYSTEM TRUE\r\n");break;
	case SIGNAL_LOW:printf("SIGNAL_LOW\r\n");break;
	case SIGNAL_HIGH:printf("SIGNAL_HIGH\r\n");break;
	case TOOLONG:printf("TOOLONG\r\n");break;
	case SYSTEM_ERROR:printf("SYSTEM_ERROR\r\n");break;
	default:printf("DATA_ERROR\r\n");break;
	}
#endif
}

uint8_t crc_high_first(uint8_t *ptr, uint8_t len){
    uint8_t i;
    uint8_t crc = 0x00;/*计算初始crc*/
    while(len--){
        crc ^= *ptr++;
        for(i = 8 ; i > 0 ; --i){
            if(crc & 0x80){
                crc = (crc << 1)^0x31;
            }
            else{
                crc = (crc << 1);
            }
        }
    }
    return crc;
}




//zero_type = 0 : 设置绝对零点abs_zero
//zero_type = 1 : 设置相对零点rel_zero
//return 0: 设置失败
//return 1: 设置并保存成功
int setZero(int zero_type)
{
	int ret = 0;
	int zero_sta = zero_type;
	uint8_t abs_zero_buf[] = {0x01, 0x06, 0x00, 0x0B, 0x00, 0x00, 0xf8, 0x08};//01 06 00 0b 00 00 f8 08 hui:01 06 00 0B 00 00 F8 08
	uint8_t rel_zero_buf[] = {0x01, 0x06, 0x00, 0x0B, 0x00, 0x01, 0x39, 0xc8};//01 06 00 0b 00 01 39 C8 hui:01 06 00 0B 00 01 39 C8
	uint8_t save_buf[]     = {0x01, 0x06, 0x00, 0x0F, 0x00, 0x00, 0xb9, 0xc9};//01 06 00 0F 00 00 B9 C9 hui:01 06 00 0F 00 04 B8 0A


	while(1)
	{
		//read act926m-t 4 registers 8 bytes data ,data format xx xx xx xx (4 bytes ieeee754 x angle data) yy yy yy yy (4 bytes ieeee754 y angle data)
		switch (zero_sta)
		{
			case 0://发送设置绝对零点
				HAL_UART_Transmit(&huart2,abs_zero_buf,8,200);
				printf("send set abs_zero_buf\r\n");
				break;
			case 1://发送设置相对零点
				HAL_UART_Transmit(&huart2,rel_zero_buf,8,200);
				printf("send set rel_zero_buf\r\n");
				break;
			case 2://发送保存
				HAL_UART_Transmit(&huart2,save_buf,8,200);
				printf("send set save_buf\r\n");
				break;
			default://
				break;
		}

		vTaskDelay(250); /////delay time must large than 25ms or will lost frame data

		if (uart2_recv_end_flag ==1)                                           //recv_end_flag 结束标志
		{
			if (uart2_rx_buffer[0] != 0x01)
				continue;
			if (uart2_rx_buffer[1] != 0x06)
				continue;
			if (uart2_rx_buffer[2] != 0x00)
				continue;
			if (uart2_rx_buffer[3] == 0x0B)   //设置零点返回结果
			{
				zero_sta = 2;
			}else if (uart2_rx_buffer[3] == 0x0F)
			{
				printf("set zero success and saved\r\n");
				ret = 1;//设置成功
				break;
			}

			uart2_rx_len=0;
			uart2_recv_end_flag=0;
			HAL_UART_Receive_DMA(&huart2, uart2_rx_buffer, UART2_BUF_SIZE);
		}
	}
	return ret;
}

static int sendSetZeroRequestJson()
{
	int ret = 0, length = 0;
	cJSON *root = NULL;
	char* out = NULL;

	root = cJSON_CreateObject(); //
	cJSON_AddStringToObject(root, "optRequest",  "what?");
	out = cJSON_Print(root);
	length = strlen(out);
	free(out);
	ret = sendToTcpServer(set_zero_sock, (unsigned char *)out, length, TYPE_SET_OPT_REQ);
	cJSON_Delete(root);
	return ret;
}
static int sendSetZeroResultJson()
{
	int ret = 0, length = 0;
	cJSON *root = NULL;
	char* out = NULL;

	root = cJSON_CreateObject(); //
	cJSON_AddStringToObject(root, "optRequest",  "success!");
	out = cJSON_Print(root);
	length = strlen(out);
	free(out);
	ret = sendToTcpServer(set_zero_sock, (unsigned char *)out, length, TYPE_SET_SUCCESS);
	cJSON_Delete(root);
	return ret;
}

static int setZeroClientSendPoll()
{
	int ret = 0;
	switch (set_zero_sta)
	{
	case 0: //请求设置
		ret = sendSetZeroRequestJson();
		return ret;
	case 1: //设置成功
		ret = sendSetZeroResultJson();
		if (ret > 0) set_zero_sta = 0;
		return ret;
		break;
	default:
		break;
	}
	return ret;
}

//static int setZeroClientDataHandler(char* buf)
//{
//	int type = 0;
//	int length = 0;
//	int ret = 0;
//
//	//正常接受得到
//	//解析type
//	type   = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
//	length = buf[4] << 24 | buf[5] << 16 | buf[6] << 8 | buf[7];
//	switch (type)
//	{
//		case TYPE_SET_ABS_ACK:
//			printf("type is right,type = %d    length = %d    set_zero_sta = %d\r\n", type, length, set_zero_sta);
//			//设置绝对零点
//			vTaskSuspend(rs485TaskHandle);
//			ret = setZero(0);
//			if (ret == 1)
//			{
//				set_zero_sta = 1;//设置成功
//			}
//			vTaskResume(rs485TaskHandle);
//			break;
//		case TYPE_SET_REL_ACK:
//			printf("type is right,type = %d    length = %d    set_zero_sta = %d\r\n", type, length, set_zero_sta);
//			//设置相对零点
//			vTaskSuspend(rs485TaskHandle);
//			ret = setZero(1);
//			if (ret == 1)
//			{
//				set_zero_sta = 1;//设置成功
//			}
//			vTaskResume(rs485TaskHandle);
//			break;
//		case TYPE_SET_SUCCESS_ACK:
//			printf("type is right,type = %d    length = %d    set_zero_sta = %d\r\n", type, length, set_zero_sta);
//			//设置成功
//			set_zero_sta = 0;
//			break;
//		default:
//			printf("unkown type\r\n"); //正常情况不会执行到这里
//			ret = -2;
//			break;
//	}
//
//	return ret;
//}

static int parseJson(uint8_t *payload)
{
	int ret = 0;
	cJSON *parseRoot, *item;
	parseRoot=cJSON_Parse(payload);

	if(!parseRoot){
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());
	}


	item = cJSON_GetObjectItem(parseRoot, "operation");
	if (item != NULL)
	{
		printf("operation:%s\r\n", item->valuestring);
		if (!strncmp(item->valuestring, "set abs zero", 12))
		{
			vTaskSuspend(rs485TaskHandle);
			ret = setZero(0);
			if (ret == 1)
			{
//				set_zero_sta = 1;//设置成功
			}
			vTaskResume(rs485TaskHandle);
		}
		else if (!strncmp(item->valuestring, "set rel zero", 12))
		{
			vTaskSuspend(rs485TaskHandle);
			ret = setZero(1);
			if (ret == 1)
			{
//				set_zero_sta = 1;//设置成功
			}
			vTaskResume(rs485TaskHandle);
		}
	}

	item = cJSON_GetObjectItem(parseRoot, "ADS_K1");
	if (item != NULL)
	{
		printf("ADS_K1:%f\r\n", item->valuedouble);
		ADS_K1 = item->valuedouble;
		ret = 1;
	}

	item = cJSON_GetObjectItem(parseRoot, "ADS_B1");
	if (item != NULL)
	{
		printf("ADS_B1:%f\r\n", item->valuedouble);
		ADS_B1 = item->valuedouble;
		ret = 1;
	}

	item = cJSON_GetObjectItem(parseRoot, "ADS_K2");
	if (item != NULL)
	{
		printf("ADS_K2:%f\r\n", item->valuedouble);
		ADS_K2 =  item->valuedouble;
		ret = 1;
	}

	item = cJSON_GetObjectItem(parseRoot, "ADS_B2");
	if (item != NULL)
	{
		printf("ADS_B2:%f\r\n", item->valuedouble);
		ADS_B2 =  item->valuedouble;
		ret = 1;
	}

	cJSON_Delete(parseRoot);
	return ret;
}

static int setZeroClientPayloadHandler(int type, uint8_t *payload)
{
	int ret = -1;
	switch (type)
	{
		case TYPE_SET_OPT_REQ_ACK:
			ret = parseJson(payload);
			if (ret == 1) set_zero_sta = 1;//设置成功
			break;
		case TYPE_SET_SUCCESS_ACK:
			set_zero_sta = 0;//回到初始状态
			break;
		default:
			break;

	}
	return ret;
}

static int setZeroClientRecvHandler()
{
	int len = 0 , ret = -1, type = 0, length = 0;//len:实际读到的数据长度  length：数据长度
//	char t_l[8];//包头
//	char payload[256];//数据

	len = recv(set_zero_sock, set_zero_tcpbuf, 8, 0); //协议规定，前八字节为type-length
	if (len == 0)
	{
		printf("tcp setZero client: connect close\r\n");
		return -1;
	}
	else if (len < 0)
	{
		printf("tcp setZero client: recv fail, timeout\r\n");
		return -1;
	}
	else if (len > 1500)
	{
		printf("tcp setZero client: read len err\r\n");
		bzero(set_zero_tcpbuf, sizeof(set_zero_tcpbuf));
		return -1;
	}

	//正常接受得到
	//解析type
	type   = set_zero_tcpbuf[0] << 24 | set_zero_tcpbuf[1] << 16 | set_zero_tcpbuf[2] << 8 | set_zero_tcpbuf[3];
	length = set_zero_tcpbuf[4] << 24 | set_zero_tcpbuf[5] << 16 | set_zero_tcpbuf[6] << 8 | set_zero_tcpbuf[7];
	len = recv(set_zero_sock, set_zero_tcpbuf, length, 0);
	if (len == 0)
	{
		printf("tcp setZero client: connect close\r\n");
		return -1;
	}
	else if (len < 0)
	{
		printf("tcp setZero client: recv fail, timeout\r\n");
		return -1;
	}
	else if (len > 1500)
	{
		printf("tcp setZero client: read len err\r\n");
		bzero(set_zero_tcpbuf, sizeof(set_zero_tcpbuf));
		return -1;
	}

	ret = setZeroClientPayloadHandler(type, set_zero_tcpbuf);

	return ret;
}


static void setZeroClientInit()
{
	int ret = 0;
	struct sockaddr_in addr;
	struct sockaddr_in peerAddr;
	socklen_t addrLen;
	struct timeval tv_out;
	char ipbuf[20] = {0};

	do{
		set_zero_sock = socket(AF_INET, SOCK_STREAM, 0);
		printf("tcp setZero client: set_zero_sock = %d\r\n", set_zero_sock);

		tv_out.tv_sec = 10;
		tv_out.tv_usec = 0;
		ret = setsockopt(set_zero_sock, SOL_SOCKET, SO_RCVTIMEO, &tv_out, sizeof(tv_out)); //设置发送超时时间
		if(ret == -1)
		{
			printf("tcp setZero client: setsockopt fail! set_zero_sock = %d\r\n", set_zero_sock);
		}

		// 第二步：绑定端口号
		// 初始化
		memset(&addr, 0, sizeof(addr));
		addr.sin_len = sizeof(addr);

		// 指定协议簇为AF_INET，比如TCP/UDP等
		addr.sin_family = AF_INET;

		// 监听任何ip地址
		addr.sin_addr.s_addr = INADDR_ANY;
		ret = bind(set_zero_sock, (const struct sockaddr *)&addr, sizeof(addr));
		if (ret < 0)
		{
			printf("tcp setZero client: bind fail! set_zero_sock = %d\r\n",  set_zero_sock);
		}

		// p2p
		memset(&peerAddr, 0, sizeof(peerAddr));
		peerAddr.sin_len = sizeof(peerAddr);
		peerAddr.sin_family = AF_INET;
		peerAddr.sin_port = htons(config.server_ip.server_port_cfg);

		// 指定服务端的ip地址，测试时，修改成对应自己服务器的ip
		snprintf(ipbuf, sizeof(ipbuf), "%d.%d.%d.%d", config.server_ip.ip_cfg[0],
										config.server_ip.ip_cfg[1],
										config.server_ip.ip_cfg[2],
										config.server_ip.ip_cfg[3]);

		peerAddr.sin_addr.s_addr = inet_addr(ipbuf);

		addrLen = sizeof(peerAddr);
		ret = connect(set_zero_sock, (struct sockaddr *)&peerAddr, addrLen);
		if (ret < 0)
		{
			close(set_zero_sock);
			set_zero_sock = -1;
			printf("tcp setZero client: connect fail! ret = %d\r\n", ret);
			vTaskDelay(10000);
			continue;
		}
		else
		{
			#if LOG_PRINTF_DEF
			LogPrint("tcp setZero client: connect success");
			#else
			printf("tcp setZero client: connect success");
			#endif
		}

		while (1)
		{
			//sendhandler
			ret = setZeroClientSendPoll();
			if (ret < 0)
			{
				printf("setZeroClientSendPoll err\r\n");
				break;
			}

			//recvhandler
			ret = setZeroClientRecvHandler();
			if (ret < 0)
			{
				printf("setZeroClientRecvHandler err\r\n");
				break;
			}
		}

		close(set_zero_sock);
		set_zero_sock = -1;

	}while(1);

	printf("setZeroTask exit");
	vTaskDelete(NULL);

}

//请求是否设置角度传感器零点任务
void setZeroTask(void *pvParameters)
{
#if LOG_PRINTF_DEF
	LogPrint("setZeroTask start");
#else
	printf("setZeroTask start\r\n");
#endif
	setZeroClientInit();
}
