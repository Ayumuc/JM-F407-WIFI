/*
 * logQueue.c
 *
 *  Created on: Dec 1, 2021
 *      Author: Administrator
 */
#include "main.h"
#include "logQueue.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "sys/types.h"
#include "sys/socket.h"
#include <lwip/netdb.h>
#include "lwip/netif.h"
#include "lwip/netif.h"
#include "cJSON.h"
#include "boardConfig.h"
#include "rtc.h"
#include "tim.h"
#include "myfunction.h"
#include "sd2078.h"
#include "delay.h"

//#define SERVER_IP      "192.168.1.104"
#define SERVER_IP      "192.168.0.101"
#define SERVER_PORT     8001

#define TYPE_SEND_LOG      1
#define TYPE_RETN_LOG      2


#define  REQUEST_OPT    0
#define  HANDLER_OPT    1

//typedef struct {
//	char buffer[1024];
//	int  len;
////	int  offset; //起始地址
//}LOG_BUFFER;
//LOG_BUFFER log_buffer;


int                  log_client_disconnect_flg; //0表示连接上；1表示没有连接上
int                  log_client_sock  = -1 ;
xQueueHandle         xQueueLoginfo;
xQueueHandle         xQueueErrLoginfo;
SemaphoreHandle_t    xUploadLogSemaphore;



void initLogQueue()
{
	HAL_TIM_Base_Start_IT(&htim7);

	xQueueLoginfo = xQueueCreate(30, sizeof(LOGINFO));
	if (xQueueLoginfo == 0)
	{
		printf("xQueueLoginfo creat err\r\n");
	}

	xQueueErrLoginfo = xQueueCreate(30, sizeof(LOGINFO));
	if (xQueueErrLoginfo == 0)
	{
		printf("xQueueErrLoginfo creat err\r\n");
	}

	xUploadLogSemaphore = xSemaphoreCreateBinary();
	if( xUploadLogSemaphore == NULL )
	{
		printf("create xUploadLogSemaphore error");
	}
}


//ip+时间戳作为key
static void getLogKey(char *key)
{
//	char key_tmp[50];
	RTC_TimeTypeDef gTime = {0};
	RTC_DateTypeDef gDate = {0};
	float sub_sec = 0, sec = 0;

	DEVICE_CONFIG* config_tmp = readConfig();//读出配置

	//获取时间戳 和 ip 地址作为 key
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BCD);

	gTime.Seconds = BCD_Decimal(gTime.Seconds);
	sub_sec = (255.0 - gTime.SubSeconds)/256;
	sec = sub_sec + gTime.Seconds;

	if (log_client_disconnect_flg == 0)
		sprintf(key, "%d.%d.%d.%d-20%02x-%02x-%02x %02x:%02x:%.3f", config_tmp->eth_info.ip_cfg[0],   //192
			                                                         config_tmp->eth_info.ip_cfg[1],   //168
								                                     config_tmp->eth_info.ip_cfg[2],   //3
								                                     config_tmp->eth_info.ip_cfg[3],   //46
									                                  gDate.Year,                      //年
																	  gDate.Month,                     //月
																	  gDate.Date,                      //日
																	  gTime.Hours,                     //时
																	  gTime.Minutes,                   //分
																	  sec);               //亚秒    //时间戳 秒，改成毫秒还需要修改接口
	else
		sprintf(key, "lost:%d.%d.%d.%d-20%02x-%02x-%02x %02x:%02x:%.3f", config_tmp->eth_info.ip_cfg[0],   //192
					                                                         config_tmp->eth_info.ip_cfg[1],   //168
										                                     config_tmp->eth_info.ip_cfg[2],   //3
										                                     config_tmp->eth_info.ip_cfg[3],   //46
											                                  gDate.Year,                      //年
																			  gDate.Month,                     //月
																			  gDate.Date,                      //日
																			  gTime.Hours,                     //时
																			  gTime.Minutes,                   //分
																			  sec);
}


void intoLogQueue(char *msg, uint32_t msgLen, uint8_t isISR)
{
	LOGINFO loginfo;

	bzero( &loginfo, sizeof(loginfo));

	//获取ke
	getLogKey(loginfo.log_key);

	memcpy(&loginfo.log_value, msg, msgLen);

	if (xQueueLoginfo == 0)
	{
		printf("uncreated xQueueLoginfo handler\r\n");
		return;
	}
	if (isISR)
	{
		if (log_client_disconnect_flg == 0)
			xQueueSendFromISR( xQueueLoginfo, ( void * ) &loginfo, (TickType_t)0);
		else


			xQueueSendFromISR( xQueueErrLoginfo, ( void * ) &loginfo, (TickType_t)0);
	}
	else
	{
		if (log_client_disconnect_flg == 0)
			xQueueSend( xQueueLoginfo, ( void * ) &loginfo, 0);
		else
			xQueueSend( xQueueErrLoginfo, ( void * ) &loginfo, 0);
	}
}
//void intoErrLogQueue(char *msg, uint32_t msgLen, uint8_t isISR)
//{
//	LOGINFO errloginfo;
//	bzero( &errloginfo, sizeof(loginfo));
//	getLogKey(errloginfo.log_key);	//获取key
//	memcpy(&errloginfo.log_value, msg, msgLen);
//
//	if (xQueueLoginfo == 0)
//	{
//		printf("uncreated xQueueLoginfo handler\r\n");
//		return;
//	}
//
//	if (isISR)
//		xQueueSendFromISR( xQueueErrLoginfo, ( void * ) &errloginfo, (TickType_t)0);
//	else
//		xQueueSend( xQueueErrLoginfo, ( void * ) &errloginfo, 0);
//}


void LogPrint(char *log_msg)
{
	char buf[256] = {0};
	char i = 0;

	for(i = 0; i < 20; i++) //这里需要延时一下才能成功打印，至于为什么，我也不知道，反正不延时就有一条打印不了,以后有时间再优化
		for_delay_us(100);

	int length = strlen(log_msg)+1;

    memcpy(buf, log_msg, length);  //printf("strlen(log_msg)+1 = %d\r\n", strlen(log_msg)+1);

//    if (log_client_disconnect_flg == 0)
	intoLogQueue(buf, length, 0);
//    else
//    	intoErrLogQueue(buf, length, 0); //err log

    printf("%s\r\n", buf);

    bzero( log_msg_global, sizeof(log_msg_global));
}

//void outLogQueue()
//{
//
//}

static int sendToTcp(int sock, unsigned char *data,unsigned int length)
{
	int ret = 0;

	ret = send(sock, data, length, 0);
	if (ret < 0)
	{
		close(sock);
		sock = -1;
		printf("sendToTcp error\r\n");
	}
	return ret;
}





// type = 1 ,发日志
//               大端
// 返回值：0 成功
//      小于0 失败
int sendToTcpServer(int sock, unsigned char* data, int length, int type)
{
//	int i = 0;
	int ret = 0;

	if (data == NULL || length == 0)
	{
		printf("line:%d ----------- sendToTcpServer error\r\n", __LINE__);
		return -1;
	}

	unsigned char* buf = (unsigned char *)malloc(length+8);
	buf[0] = (type & 0xFF000000) >> 24;
	buf[1] = (type & 0x00FF0000) >> 16;
	buf[2] = (type & 0x0000FF00) >> 8;
	buf[3] = (type & 0x000000FF) >> 0;

	buf[4] = (length & 0xFF000000) >> 24;
	buf[5] = (length & 0x00FF0000) >> 16;
	buf[6] = (length & 0x0000FF00) >> 8;
	buf[7] = (length & 0x000000FF) >> 0;

	memcpy(buf+8, data, length); //数据内容

	ret = sendToTcp(sock, buf,length+8);

//	//print hex
//	printf("sendToTcpServer hex:");
//	for (i = 0; i < length+8; i++)
//	{
//		printf("%02x ", buf[i]);
//	}
//	printf("\r\n");

	//print string
//	printf("sendToTcpServer str:%s\r\n", buf);

	free(buf);

	return ret;
}




int outLogQueue(void)
{
	LOGINFO loginfo;
	cJSON *root = NULL;
	char* out = NULL;
	int length = 0;
	int ret = 0;
	static char cjson_cnt;

//	bzero(log_buffer.buffer,sizeof(log_buffer.buffer));

	root = cJSON_CreateObject(); //
	if (root == NULL)
	{
		printf("tcp log client: cJSON_CreateObject error\r\n");
		return -2;
	}

	while (1)
	{
		if (xQueueReceive(xQueueLoginfo, &loginfo, 0))
		{
			if (cjson_cnt++ > 30)
			{
				cjson_cnt = 0;
				printf("tcp log client: too many log!!\r\n");
				ret = -3;
				break;
			}
			else
			{
				//组合json格式
				cJSON_AddStringToObject(root, loginfo.log_key,  loginfo.log_value);
			}
		}
		else //读取完毕
		{
			out = cJSON_Print(root);
			if (out==NULL)
			{
				printf("tcp log client: cJSON_Print error!\r\n");
				ret = -2;
			}
			else
			{
				length = strlen(out);
				ret = sendToTcpServer(log_client_sock, (unsigned char *)out, length, TYPE_SEND_LOG);
				cjson_cnt = 0;
				free(out);
			}
			break;
		}
	}
	cJSON_Delete(root);
	return ret;
}

int outErrLogQueue(void)
{
	LOGINFO errloginfo;
	cJSON *root = NULL;
	char* out = NULL;
	int length = 0;
	int ret = 0;
	static char cjson_cnt;

	if (log_client_disconnect_flg == 1)
	{
		root = cJSON_CreateObject(); //
		if (root == NULL)
		{
			printf("tcp log client: cJSON_CreateObject error\r\n");
			return -2;
		}

		while (1)
		{
			if (xQueueReceive(xQueueErrLoginfo, &errloginfo, 0))
			{
				if (cjson_cnt++ > 30)
				{
					cjson_cnt = 0;
					printf("tcp log client: too many log!!\r\n");
					ret = -3;
					break;
				}
				else
				{
					//组合json格式
					cJSON_AddStringToObject(root, errloginfo.log_key,  errloginfo.log_value);
				}
			}
			else //读取完毕
			{
				out = cJSON_Print(root);
				if (out==NULL)
				{
					printf("tcp log client: cJSON_Print error!\r\n");
					ret = -2;
				}
				else
				{
					length = strlen(out);
					ret = sendToTcpServer( log_client_sock,(unsigned char *)out, length, TYPE_SEND_LOG);
					if (ret >= 0) log_client_disconnect_flg = 0;//要等把丢失的log发出去再置回0
					cjson_cnt = 0;
					free(out);
				}
				break;
			}
		}

		cJSON_Delete(root);
		return ret;
	}
	return ret;
}

//直接使用type来决定还是用json来决定
static int logClientDataHandler(char *buf)
{
	int type = 0;
	int length = 0;

	//正常接受得到
	//解析type
	type   = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
	length = buf[4] << 24 | buf[5] << 16 | buf[6] << 8 | buf[7];
	switch (type)
	{
		case TYPE_RETN_LOG:
			printf("type is right,type = %d    length = %d\r\n", type, length);
			if (length == 0)
			{
				//开启定时器，等待定时时间到
				 HAL_TIM_Base_Start_IT(&htim7);
				 if (xSemaphoreTake(xUploadLogSemaphore, portMAX_DELAY)) //
				 {
					 HAL_TIM_Base_Stop_IT(&htim7);
				 }
			}
			break;
		default:
			printf("unkown type\r\n"); //正常情况不会执行到这里
			return -2;
			break;
	}
	return 0;
}

static int logClientRecvHandler(void)
{
	int len = 0 , ret = 0;
    char t_l[8] = {0};      //type,length
	len = read(log_client_sock, t_l, sizeof(t_l));
	if (len == 0)
	{
		printf("tcp log client: connect close\r\n");
		return -1;
	}
	else if (len < 0)
	{
		printf("tcp log client: recv fail, timeout\r\n");
		return -1;
	}
	else if (len != sizeof(t_l)) //理论上不会执行这里，除非丢失数据了
	{
		printf("tcp log client: read len err\r\n");
		bzero(t_l, sizeof(t_l));
		return -1;
	}
	else   //成功接收到数据
	{
		ret = logClientDataHandler(t_l);
		if (ret < 0)
		{
			printf("tcp log client: recv fail, close sock, reconnect\r\n");
			return -1;
		}
	}
	return ret;
}

static void logClientInit(void) //只管发送
{
	// 第一步：创建soket
	// TCP是基于数据流的，因此参数二使用SOCK_STREAM
	int ret = 0;
	struct sockaddr_in addr;
	struct sockaddr_in peerAddr;
	socklen_t addrLen;
	struct timeval tv_out;
	char ipbuf[20] = {0};

	do{
		log_client_sock = socket(AF_INET, SOCK_STREAM, 0);
		printf("tcp log client: log_client_sock = %d\r\n", log_client_sock);

		tv_out.tv_sec = 60;
		tv_out.tv_usec = 0;
		ret = setsockopt(log_client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv_out, sizeof(tv_out)); //设置发送超时时间
		if(ret == -1)
		{
			printf("tcp log client: setsockopt fail! log_client_sock = %d\r\n", log_client_sock);
		}

		// 第二步：绑定端口号
		// 初始化
		memset(&addr, 0, sizeof(addr));
		addr.sin_len = sizeof(addr);

		// 指定协议簇为AF_INET，比如TCP/UDP等
		addr.sin_family = AF_INET;

		// 监听任何ip地址
		addr.sin_addr.s_addr = INADDR_ANY;
		ret = bind(log_client_sock, (const struct sockaddr *)&addr, sizeof(addr));
		if (ret < 0)
		{
			printf("tcp log client: bind fail! log_client_sock = %d\r\n", log_client_sock);
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
		ret = connect(log_client_sock, (struct sockaddr *)&peerAddr, addrLen);
		if (ret < 0)
		{
			close(log_client_sock);
			log_client_sock = -1;
			log_client_disconnect_flg = 1;
#if LOG_PRINTF_DEF
			LogPrint("tcp log client: connect fail!");
#else
			printf("tcp log client: connect fail! ret = %d\r\n", ret);
#endif
			vTaskDelay(10000);
			continue;
		}
		else
		{
#if LOG_PRINTF_DEF
			LogPrint("tcp log client: connect success");
#else
			printf("tcp log client: connect success\r\n");
#endif
		}

//		log_flg = 1;//第一次发送
		while (1)
		{
			ret = outErrLogQueue(); //读空队列 //发送日志
			if (ret < 0)
			{
				printf("tcp log client: outErrLogQueue err\r\n");
				break; //发送出错，可能是套接字关闭了
			}

			ret = outLogQueue(); //读空队列 //发送日志
			if (ret < 0)
			{
				printf("tcp log client: outLogQueue err\r\n");
				break; //发送出错，可能是套接字关闭了
			}

			ret = logClientRecvHandler();
			if (ret < 0)
			{
				printf("tcp log client: recv_handler err\r\n");
				break; //发送出错，可能是套接字关闭了
			}
		}

		close(log_client_sock);
		log_client_sock = -1;

	}while(1);

	vTaskDelete(NULL);
}

void logUploadTask(void *pvParameters)
{
#if LOG_PRINTF_DEF
	LogPrint("logUploadTask start");
#else
	printf("logUploadTask start\r\n");
#endif

	logClientInit();

}
