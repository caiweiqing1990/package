#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>   /* signal() */
#include <fcntl.h> /* file control definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <unistd.h> /* 注意此处：signal.h要先include进来 */
#include <ctype.h>
#include <sys/select.h> /* select() */
#include <netinet/tcp.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/stat.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/prctl.h>
#include "msg.h"
#include "server.h"
#define USERID_LLEN 21  //完整的用户ID长度
#define USERID_LEN  12  //上传TSC服务器时，只需要传送用户ID的后12位，以节约流量
#define IMSI_LEN    15  //IMEI和IMSI的长度

static int client_socket = -1;

int ConnectTSC(char* routename, char* ip, int port, int *err, int timeout)
{
	int sockfd;
    struct sockaddr_in server_addr;   
	struct timeval timeo = {timeout, 0};
	int on = 1;
	int size_send = 1024*1024;
	int optlen = sizeof(int);
    struct ifreq struIR;
	
    if(inet_aton(ip, &server_addr.sin_addr) == 0) 
	{
        printf("the hostip is not right!");   
        return -1;  
    }

    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
	{
        printf("Socket Error:%s", strerror(errno));   
        return -1; 
    }

	setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, (void *)&size_send, optlen);
	setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY,&on,sizeof(on));        
	if(timeout > 0) setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeo, sizeof(timeo));

	if(routename)
	{
		strncpy(struIR.ifr_name, routename, IFNAMSIZ);
		if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, &struIR, sizeof(struIR)) < 0)
		{
			printf("setsockopt Error:%s\n", strerror(errno));  
			close(sockfd);
			return -1; 
		}
	}

    server_addr.sin_family = AF_INET;   
    server_addr.sin_port = htons(port);   

    if(connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) 
	{
		if(err != NULL) *err = errno;
        printf("Connect Error:%s errno=%d %s %d\n", strerror(errno), errno, ip, port);   
		close(sockfd);
        return -1;
    }
	else
	{
		printf("ConnectTSC success sockfd=%d %s %d\n", sockfd, ip, port);
	}

	return sockfd;
}

static void *HeartBeatThead(void *p)
{
	char buf[1024] = {0};
	MsgAppConnectReq *req = (MsgAppConnectReq *)buf;
	req->header.length = sizeof(MsgAppConnectReq);
	req->header.mclass = CONNECT_CMD;
	memcpy(req->userid, "860311000013112121509", 21);
	write(client_socket, buf, req->header.length);

	MsgAppHeartbeatReq *Heartbeat = (MsgAppHeartbeatReq *)buf;
	Heartbeat->header.length = sizeof(MsgAppHeartbeatReq);
	Heartbeat->header.mclass = HEART_BEAT_CMD;
	Heartbeat->Type = 0;
	memcpy(Heartbeat->userid, "860311000013112121509", 21);

	while(1)
	{
		write(client_socket, buf, Heartbeat->header.length);
		sleep(5);
	}
	
	return NULL;
}

static void *select_server(void *p)
{
	struct timeval timeout={3,0};
	fd_set fds;
	char buf[1024];
	char tmp[1024];
	int nread;
	MsgHeader *pHeader;
	unsigned short offset = 0;

	while(1)
	{
		if(client_socket < 0)
		{
			client_socket = ConnectTSC(NULL, "192.168.1.1", 12080, NULL, 10);
			sleep(5);
			continue;
		}
	
		FD_ZERO(&fds);
		FD_SET(client_socket, &fds);
		timeout.tv_sec = 3;
		switch(select(client_socket + 1, &fds, NULL, NULL, &timeout))
		{
			case -1: break;
			case  0: break;
			default:
				{
					if(FD_ISSET(client_socket, &fds))
					{
						if(offset < sizeof(buf))
						{
							bzero(buf+offset,sizeof(buf)-offset);
						}
						bzero(tmp,sizeof(tmp));
						
						nread = read(client_socket, buf+offset, 512) + offset;

						if(offset > 0 && nread <= offset)
						{
							printf("server error %d %s\n",errno, strerror(errno));
							close(client_socket);
							client_socket = -1;
							break;
						}

						offset	= 0;
						while(1)
						{
							if (nread > 0)
							{
								pHeader = (MsgHeader *)(&buf[offset]);
								printf("read from server nread=%d,length=%d,mclass=0X%04X offset=%d\n",nread, pHeader->length, pHeader->mclass, offset);

								if((offset + pHeader->length) > nread)
								{
									int leftsize = nread - offset;
									memcpy(buf,pHeader,leftsize);
									offset = leftsize;
									printf("leftsize=%d\n",leftsize);
									break;
								}

								switch(pHeader->mclass)
								{
									case 0x8000:
									{
										break;	
									}									
									case 0x8001:
									{
										break;	
									}									

									default:
									{
										printf("received unrecognized message from server: %04x\n", pHeader->mclass);
										offset = 0;
										nread = 0;
										pHeader->length = 0;
										break;	
									}								
								}
							}
							else
							{
								printf("server read return error: errno=%d (%s) nread=%d\n", errno, strerror(errno), nread);
								close(client_socket);
								client_socket = -1;
								break;
							}
							
							offset += pHeader->length;
							if(offset == nread)
							{
								offset = 0;
								break;
							}
						}	
					}
			}
			break;
		}
		
	}
	
	return NULL;
}

int main(int argc, char *argv[])
{
	if(argc != 3) return -1;

	pthread_t id_1;
	client_socket = ConnectTSC(NULL, "192.168.1.1", 12080, NULL, 0);

	if(client_socket > 0)
	{
		if(pthread_create(&id_1, NULL, HeartBeatThead, NULL) == -1) exit(1);
		if(pthread_create(&id_1, NULL, select_server, NULL) == -1) exit(1);
	}

	while(1)
	{
		printf("getchar=%x\n", getchar());
		char tmp[512] = {0};
		MsgPtt *req = (MsgPtt *)tmp;
		req->header.length = sizeof(MsgPtt);
		req->header.mclass = PTT;

		memcpy(req->MsID, argv[1], 21);
		memcpy(req->GrpID, argv[2], 21);
		
		//printf("PTT	%.21s %.21s\n", req->MsID, req->GrpID);		
		//write(client_socket, tmp, req->header.length);

		//sleep(3);
		
		MsgHup *req2 = (MsgHup *)tmp;
		req2->header.length = sizeof(MsgHup);
		req2->header.mclass = HUP;

		memcpy(req2->MsID, argv[1], 21);
		memcpy(req2->GrpID, argv[2], 21);
		
		//printf("HUP	%.21s %.21s\n", req2->MsID, req2->GrpID);		
		//write(client_socket, tmp, req2->header.length);

		//sleep(3);

		MsgMsHup *req1 = (MsgMsHup *)tmp;
		req1->header.length = sizeof(MsgMsHup);
		req1->header.mclass = MS_HUP;
		//860311100013112121509  860311100115815855317
		memcpy(req1->MsID, argv[1], 21);
		memcpy(req1->Src_Name, argv[2], 21);
		req1->Operation = 0;
		printf("MS_HUP	%.21s %.21s\n", req1->MsID, req1->Src_Name);		
		write(client_socket, tmp, req1->header.length);
		
		sleep(3);

	}

	return 0;
}


