#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#ifdef _WIN32
# include <winsock2.h>
#else
# include <sys/socket.h>
#endif

/* For MinGW */
#ifndef MSG_NOSIGNAL
# define MSG_NOSIGNAL 0
#endif


#include <sys/types.h> 
#include <stdarg.h>
#include <time.h>
#include <pthread.h>
#include "sys/time.h"
#include <sys/ipc.h>
#include <sys/msg.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <arpa/inet.h> 
#include <netinet/in.h> 

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>




int init_tcp()
{
     // 1.创建套接字 - 设置协议
     int sfd = socket(AF_INET,SOCK_STREAM, 0);
     if( 0 > sfd )
     {
         perror("socket");
         exit(-1);
     }
     //2. 解决在close之后会有一个WAIT_TIME，导致bind失败的问题
     int val = 1;
     int ret = setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, (void *)&val, sizeof(int));
     if(ret < 0)
     {
         printf("setsockopt wait_time");
         exit(1);
     }

     //3. 绑定IP和PORT
     struct sockaddr_in saddr;
     saddr.sin_family = AF_INET;
     saddr.sin_addr.s_addr = INADDR_ANY;
     saddr.sin_port = htons(4188);
     ret = bind(sfd, (struct sockaddr *)&saddr, sizeof(saddr));
     if(ret < 0)
     {
         printf("bind");
         exit(1);
     }
     //4. 监听
     ret = listen(sfd, 5);
     if(ret == -1)
     {
         printf("listen");
         exit(1);
     }
     printf("Server is ready ... \n");

     return sfd;
 }

int main(int argc, char*argv[])
{
    unsigned char buf[1206];

    int rc;

    //1. 初始化(创建套接字socket/地址复用+bind/listen)
    int sfd = init_tcp();
    //2. 并发处理客户端（accept / fork） 2-2通信
	while(1)
     	{
         	int cfd = accept(sfd, NULL, NULL);
         	if(cfd == -1)
         	{
             		perror("accept");
             		exit(1);
         	}
                
		char client_addr_ip[20];
                struct sockaddr_in newAddr;
                socklen_t client_addr_len = sizeof(newAddr);
                getpeername(cfd, (struct sockaddr*)&newAddr, &client_addr_len);
                // 把ip转换为点分十进制
                inet_ntop(AF_INET, &newAddr.sin_addr, client_addr_ip, sizeof(client_addr_ip));
                printf("设备ip和端口 %s:%d\n", client_addr_ip, ntohs(newAddr.sin_port));
         	
    		struct tm *ptm = NULL;
    		struct timeval tv;
    		struct timezone tz;
    		short nMilSec = 0;

    		gettimeofday(&tv, &tz);
		struct timeval timeout;
                timeout.tv_sec = 10;
                int ret = setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, (void *)&timeout, sizeof(timeout));
                if(ret < 0)
                {
                     printf("setsockopt recv timeout 3 seconds");
                     exit(1);
                }

		while(1)
     		{
         		memset(buf, 0, sizeof(buf));
         		int len = recv(cfd, buf, sizeof(buf), 0);
         		if(len > 0)
         		{
				// raw acc data 
                		//for(int k = 0; k < len; k++)
                		//{
                		//        printf("%02X", buf[k]);
                		//}
                		printf(" len = %d ", len);
				gettimeofday(&tv, &tz);
            			nMilSec = (long long)tv.tv_usec/1000;
            			ptm = localtime(&tv.tv_sec);
            			printf("[%04d.%02d.%02d-%02d:%02d:%02d.%03d]\n",
                			ptm->tm_year + 1900,
                			ptm->tm_mon + 1,
                			ptm->tm_mday,
                			ptm->tm_hour,
                			ptm->tm_min,
                			ptm->tm_sec,
                			nMilSec);

                		if(len == 610)
                		{
                        		if((buf[0] == 0x33) && (buf[1] == 0x55))
                        		{
						buf[0] = 0x44;
						buf[1] = 0x66;
                                		send(cfd, buf, 6, 0);

                        		}

                		}
         		}
         		else
         		{
                 		printf("exit recv data\n");
                 		break;
         		}
         	}
         	
		close(cfd);
     	}
     
	close(sfd);
	printf("exit thread modbus407");
}


