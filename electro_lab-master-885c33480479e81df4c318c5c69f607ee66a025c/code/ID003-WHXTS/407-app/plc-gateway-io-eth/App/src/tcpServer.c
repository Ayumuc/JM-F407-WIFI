/*
 * tcpServer.c
 *
 *  Created on: Sep 27, 2021
 *      Author: Administrator
 */







#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <strings.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ctype.h>
#include "main.h"
#include "modbusTcp.h"
#include "boardConfig.h"
#include "myfunction.h"

#include "usart.h"

char * at_okip_str = "OKIP\r\n";
char * at_busy_str = "busy p...\r\n";
char * at_ok_str = "OK\r\n";
char * at_error_str = "ERROR\r\n";
char * at_conn_str = "CONNECT\r\n";
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
char port_buf[6];
char cipstart_buf[100];

uint8_t at_rxbuf[UART4_BUF_SIZE];
uint8_t modbus_tx_buf[100];
uint8_t at_txsize = 0;
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
void wifi_at_proc(uint8_t * rxbuf, uint8_t rxlen);
void wifi_at_send(void);
void wifi_uart_send(uint8_t * txbuf, uint8_t len);
void at_cifsr_send(void);

#define SERV_PORT     502
#define READ_CMD_LEN  12

uint8_t buf[2048];


//void sendToTcp(uint8_t* buf, int32_t len)
//{
//    int32_t err = send(sock, buf, len, 0);
//    if (err < 0)
//    {
//        printf("Error occured during sending\r\r\n");
//        return;
//    }
//}

//功能码0x03, 板子最多只能返回18个浮点数数据（即36个保持寄存器的值）。
void tcpServerTask(void *pvParameters)
{
	uint8_t *send_buf = NULL;
//	int i = 0;
    int sfd = -1, cfd = -1;
    int ret = -1, len = 0;
    uint8_t cmd_len = 0, size = 0;
    char client_ip[128] = {0};
    static int cnt;

    struct sockaddr_in serv_addr, client_addr;
    socklen_t addr_len;
    struct timeval tv_out;

#if LOG_PRINTF_DEF
	LogPrint("tcpServerTask start");
#else
	printf("tcpServerTask start\r\n");
#endif

    //AF_INET:ipv4   SOCK_STREAM:流协议   0:默认协议(tcp,udp)
    sfd = socket(AF_INET, SOCK_STREAM, 0);

    //绑定前先构造出服务器地址
    bzero(&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    //网络字节序
    serv_addr.sin_port = htons(SERV_PORT);
    //INADDR_ANY主机所有ip
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(sfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));

    //服务器能接收并发链接的能力
    listen(sfd, 128);

    printf("wait for connect ...\r\n");
    addr_len = sizeof(client_addr);

    while (1)
    {
    	 //阻塞，等待客户端链接，成功则返回新的文件描述符，用于和客户端通信
    	    cfd = accept(sfd, (struct sockaddr *)&client_addr, &addr_len);
    	    printf("cfd = %d\r\n", cfd);

#if LOG_PRINTF_DEF
    	    snprintf(log_msg_global, sizeof(log_msg_global),"client IP:%s  %d",
    	    													inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, client_ip, sizeof(client_ip)),
																ntohs(client_addr.sin_port));
    	    LogPrint(log_msg_global);
#else
    	    printf("client IP:%s\t%d\r\n",
    	      	            inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, client_ip, sizeof(client_ip)),
    	      	            ntohs(client_addr.sin_port));
#endif
    	    tv_out.tv_sec = 10;
			tv_out.tv_usec = 0;
			ret = setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv_out, sizeof(tv_out));
			printf("tv_sec = %d tv_usec = %d\r\n", (int)tv_out.tv_sec, (int)tv_out.tv_usec);
			if (ret == -1)
			{
				printf("setsockopt errno = %d\r\n", errno);
			}

			while (1)
    	    {
				if (cnt++ > 10000)
				{
					cnt = 0;
					LogPrint("----tcpServerTask----");
				}

    	        //阻塞接收客户端数据
				bzero(buf, sizeof(buf));
    	        len = read(cfd, buf, sizeof(buf));
//    	        printf("buf:\r\n");
//    	        for(int i = 0 ; i < sizeof(buf) ; i++){
//    	        	printf("%02X ",buf[i]);
//    	        }
//    	        printf("\r\n");
//    	        printf("len = %d\r\n", len);

    	        if (len == 0)
    	        {
#if LOG_PRINTF_DEF
    	    	    LogPrint("connect close");
#else
    	        	printf("connect close\r\n");
#endif

    	        	break;
    	        }

    	        else if (len < 0)
    	        {
#if LOG_PRINTF_DEF
    	    	    LogPrint("recv fail, timeout");
#else
    	    	    printf("recv fail, timeout\r\n");
#endif
    	        	break;
    	        }

    	        else if (len > 2000)
    	        {
#if LOG_PRINTF_DEF
    	    	    LogPrint("tcpbuf overflow");
#else
    	    	    printf("tcpbuf overflow\r\n");
#endif

    	        	bzero(buf, sizeof(buf));
    	        	break;
    	        }

    	        else   //成功接收到数据
    	        {
    	        	if (buf[2] != 0 && buf[3] != 0 ) //协议标识
    	        	{
#if LOG_PRINTF_DEF
    	    	    LogPrint("Protocol identifiers");
#else
    	    	    printf("Protocol identifiers\r\n");
#endif
    	        		break;
    	        	}

    	        	cmd_len = (buf[4] << 8 | buf[5]) + 6;

    	        	if ((cmd_len != 15) && ( cmd_len != READ_CMD_LEN) )//命令长度
    	        	{
						printf("cmd len = %d\n",cmd_len);
#if LOG_PRINTF_DEF
						LogPrint("cmd length error");
#else
						printf("cmd length error\r\n");
#endif
    	        		break;
    	        	}

    	        	send_buf = parseModbusCommand(buf, cmd_len, &size);
//    	        	printf("size = %d\r\n",  size);
    	            ret = send(cfd, send_buf, size, 0);
    	            if (ret < 0)
    	            {
#if LOG_PRINTF_DEF
    	    	    LogPrint("Error occured during sending");
#else
    	    	    printf("Error occured during sending\r\n");
#endif
    	                break;
    	            }
    	        }

    	    }
    	    close(cfd);
    	    cfd = -1;
    }

    close(sfd); //
#if LOG_PRINTF_DEF
	LogPrint("tcpServerTask exit");
#else
	printf("tcpServerTask exit\r\n");
#endif

    vTaskDelete(NULL);

}

void print_hex(uint8_t * buf, uint8_t len)
{
	uint8_t i = 0;

	for(i = 0; i < len; i++)
		printf("%02X ", buf[i]);
	printf("\r\n");
}

uint16_t speed_kilometer = 0;
void wifiClientTask(void *pvParameters)
{
#if LOG_PRINTF_DEF
	LogPrint("wifiClientTask start");
#else
	printf("wifiClientTask start\r\n");
#endif

	printf("wait wifi for connect ...\r\n");
	DEVICE_CONFIG* gconfig = readConfig();//读出配置
	bzero(ssid_buf, 20);
	bzero(pwd_buf, 20);
	bzero(ip_buf, 16);
	bzero(port_buf, 6);
	memcpy(ssid_buf, gconfig->wifi_info.ssid, strlen(gconfig->wifi_info.ssid));
	memcpy(pwd_buf, gconfig->wifi_info.pwd, strlen(gconfig->wifi_info.pwd));
	memcpy(ip_buf, gconfig->wifi_ip.wip, strlen(gconfig->wifi_ip.wip));
	memcpy(port_buf, gconfig->wifi_ip.wport, strlen(gconfig->wifi_ip.wport));
	printf("ssid:%s pwd:%s ip:%s port:%s\r\n", ssid_buf, pwd_buf, ip_buf, port_buf);
	bzero(cwjap_buf, 100);
	bzero(cipstart_buf, 100);
	snprintf(cwjap_buf, sizeof(cwjap_buf), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid_buf, pwd_buf);
	snprintf(cipstart_buf, sizeof(cipstart_buf), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", ip_buf, port_buf);
	uart4_recv_end_flag = 1;
	vTaskDelay(1000);
	volatile uint32_t ms_cnt = 0;
	volatile uint32_t now_cnt = 0;
	uint16_t delaycnt = 0;
    while (1)
    {
    	ms_cnt = xTaskGetTickCount();
    	while(GPIO_PIN_SET == HAL_GPIO_ReadPin(PB6_DIN4_GPIO_Port, PB6_DIN4_Pin));

    	now_cnt = xTaskGetTickCount();
    	if(((now_cnt - ms_cnt) > 5) && ((now_cnt - ms_cnt) < 800))
    	{
    		speed_kilometer = (36 * 1335 * 8) / (now_cnt - ms_cnt + 10);
    		printf("=== speed_kilometer = %d (now_cnt - ms_cnt): %d ===\n", speed_kilometer, (now_cnt - ms_cnt));
    		delaycnt = 0;
    		vTaskDelay(5000);
    		speed_kilometer = 0;
    		vTaskDelay(10000);
    	}
    	vTaskDelay(1);
#if 0
    	if(uart4_recv_end_flag == 1)   // 结束标志
		{
			bzero(at_rxbuf, UART4_BUF_SIZE);
			memcpy(at_rxbuf, uart4_rx_buffer, uart4_rx_len);
			//bzero(uart4_rx_buffer, UART4_BUF_SIZE);
			if(STATE_AT_CIPTRANS_TX != at_state_rx)
				printf("=\r\n%s\r\n=%d\r\n=", at_rxbuf, at_state_rx);
			else
				print_hex(at_rxbuf, uart4_rx_len);
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
    		vTaskDelay(20);
    	}
#endif
    }



#if LOG_PRINTF_DEF
	LogPrint("wifiClientTask exit");
#else
	printf("wifiClientTask exit\r\n");
#endif

    vTaskDelete(NULL);

}



void wifi_uart_send(uint8_t * txbuf, uint8_t len)
{
	HAL_UART_Transmit(&huart4, txbuf, len, 100);
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

void at_cipsendip_send(void)
{
	wifi_uart_send("192.168.20.154:502\r\n", strlen("192.168.20.154:502\r\n"));
}

void at_plus3_send(void)
{
	wifi_uart_send("+++", 3);
}

void at_modbus_send_data(void)
{
	wifi_uart_send(modbus_tx_buf, at_txsize);
}


void wifi_at_send(void)
{
	switch(at_state_tx)
	{
		case STATE_AT_3PLUS_TX:// 0
			at_plus3_send();
			at_state_rx = STATE_AT_3PLUS_TX;
		break;
		case STATE_AT_RST_TX:// 1
			at_reset_send();
			at_state_rx = STATE_AT_RST_TX;
		break;
		case STATE_AT_CWMODE_GET_TX:// 2
			at_cwmode_get_send();
			at_state_rx = STATE_AT_CWMODE_GET_TX;
		break;
		case STATE_AT_CWMODE_SET_TX:// 3
			at_cwmode_set_send();
			at_state_rx = STATE_AT_CWMODE_SET_TX;
		break;
		case STATE_AT_CIPMODE_GET_TX:// 4
			at_cipmode_get_send();
			at_state_rx = STATE_AT_CIPMODE_GET_TX;
		break;
		case STATE_AT_CIPMODE_SET_TX:// 5
			at_cipmode_set_send();
			at_state_rx = STATE_AT_CIPMODE_SET_TX;
		break;
		case STATE_AT_CIFSR_TX:// 6
			at_cifsr_send();
			at_state_rx = STATE_AT_CIFSR_TX;
		break;
		case STATE_AT_CIPSTATUS_TX:// 7
			at_cipstatus_send();
			at_state_rx = STATE_AT_CIPSTATUS_TX;
		break;
		case STATE_AT_CWJAP_TX:// 8
			at_cwjap_send();
			at_state_rx = STATE_AT_CWJAP_TX;
		break;
		case STATE_AT_CIPSTART_TX:// 9
			at_cipstart_send();
			at_state_rx = STATE_AT_CIPSTART_TX;
		break;
		case STATE_AT_CIPSEND_TX:// 10
			at_cipsend_send();
			at_state_rx = STATE_AT_CIPSEND_TX;
		break;
		case STATE_AT_CIPTRANS_TX:// 11
			at_modbus_send_data();
			at_state_rx = STATE_AT_CIPTRANS_TX;
		break;
		case STATE_AT_PLUS3_TX:// 12
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
			if (*sub == '\n')
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

	if(string_compare(rxbuf, at_wifi_status2_str, at_wifi_status4_str, at_closed_str, NULL)) // 2 connect AP, 4 disconnect TCP
	{
		vTaskDelay(3000);
		state_tx = STATE_AT_CIPSTART_TX;
	}
	else if(string_compare(rxbuf, at_wifi_status3_str, at_cipstatus0_str, NULL, NULL)) // 3 connect TCP
		state_tx = STATE_AT_CIPSEND_TX;
	else if(string_compare(rxbuf, at_wifi_status5_str, NULL, NULL, NULL)) // 5 disconnect AP
		state_tx = STATE_AT_CWJAP_TX;

	return state_tx;
}


STATE_t at_cipsend_proc(uint8_t * rxbuf)
{
	STATE_t state_tx = STATE_AT_NONE;

	if(string_compare(rxbuf, at_ok_str, "AT+CIPSEND\r\n", at_busy_str, NULL)) //
	{
		at_state_rx = STATE_AT_CIPTRANS_TX;
	}
	else
	{
		state_tx = STATE_AT_CIPMODE_GET_TX;
	}


	return state_tx;
}

void at_modbus_recv_data(uint8_t * rxbuf, uint8_t rxlen)
{
	uint8_t * txbuf = NULL;


	txbuf = parseModbusCommand(rxbuf, rxlen, &at_txsize);
	bzero(modbus_tx_buf, 100);
	memcpy(modbus_tx_buf, txbuf, at_txsize);
	at_state_tx = STATE_AT_CIPTRANS_TX;


}

void wifi_at_proc(uint8_t * rxbuf, uint8_t rxlen)
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

