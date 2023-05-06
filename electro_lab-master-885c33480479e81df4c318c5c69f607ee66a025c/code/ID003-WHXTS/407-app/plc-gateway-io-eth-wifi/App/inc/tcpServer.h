/*
 * tcpServer.h
 *
 *  Created on: Sep 28, 2021
 *      Author: Administrator
 */

#ifndef INC_TCPSERVER_H_
#define INC_TCPSERVER_H_

void tcp_server_init(void);

void tcpServerTask(void *pvParameters);
void wifiClientTask(void *pvParameters);

#endif /* INC_TCPSERVER_H_ */
