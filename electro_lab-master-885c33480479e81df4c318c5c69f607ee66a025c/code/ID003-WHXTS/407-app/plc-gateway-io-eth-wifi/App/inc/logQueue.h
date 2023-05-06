/*
 * logQueue.h
 *
 *  Created on: Dec 1, 2021
 *      Author: Administrator
 */

#ifndef INC_LOGQUEUE_H_
#define INC_LOGQUEUE_H_

typedef struct
{
	char     log_value[256];
	char     log_key[100];
}LOGINFO;





void initLogQueue();

void intoLogQueue(char *msg, uint32_t msgLen, uint8_t isISR);

int sendToTcpServer(int sock, unsigned char* data, int length, int type);

void LogPrint(char *log_msg);

void logUploadTask(void *pvParameters);

#endif /* INC_LOGQUEUE_H_ */
