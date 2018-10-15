#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <netinet/in.h>
#include "server.h"

int ConnectTSC(char* ip, int port)
{
	int sockfd;
    struct sockaddr_in server_addr;   
	
    if (inet_aton(ip, &server_addr.sin_addr) == 0) {   
        fprintf(stderr, "the hostip is not right!");   
        return -1;  
    }
    // 创建套接字   
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {   
        fprintf(stderr, "Socket Error:%s\n\a", strerror(errno));   
        return -1; 
    }   
    // 填充服务器的地址信息   
    server_addr.sin_family = AF_INET;   
    server_addr.sin_port = htons(port);   
    // 向服务器发起连接   
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) {   
        fprintf(stderr, "Connect Error:%s\n\a", strerror(errno));   
        return -1;
    }

	return sockfd;
}

void SendCmdToTsc(int socket, int cmd, char *userid, char *groupid)
{
	char buf[2048];
	int n;

	if((socket < 0) || (userid == NULL) || (groupid == NULL))
	{
		printf("%s error!\n",__FUNCTION__);
		return;
	}

	printf("cmd=0X%04X\n",cmd);
	switch(cmd)
	{
		case APP_QUERY_GROUP_CMD:
			{
			memset(buf,0,2048);
			Msg_Query_Group_Req* req = (Msg_Query_Group_Req*)buf;
			req->header.length = sizeof(Msg_Query_Group_Req);
			req->header.mclass = APP_QUERY_GROUP_CMD;
			
			StrToBcd(req->MsID, userid);
			printf("length = %d,mclass = 0x%x\n",req->header.length,req->header.mclass);
			n= write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			break;
		}
		case APP_QUERY_MS_CMD:
			{
			memset(buf,0,2048);
			Msg_Query_Ms_Req* req = (Msg_Query_Ms_Req*)buf;
			req->header.length = sizeof(Msg_Query_Ms_Req);
			req->header.mclass = APP_QUERY_MS_CMD;
			
			StrToBcd(req->MsID, userid);
			StrToBcd(req->GrpID, groupid);
			printf("length = %d,mclass = 0x%x\n",req->header.length,req->header.mclass);
			n= write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			break;
		}
		case APP_SET_BLOCK_CMD:
			{
			memset(buf,0,2048);
			Msg_SetBlock_Req* req = (Msg_SetBlock_Req*)buf;
			req->header.length = sizeof(Msg_SetBlock_Req);
			req->header.mclass = APP_SET_BLOCK_CMD;
			
			StrToBcd(req->MsID, userid);
			StrToBcd(req->GrpID, groupid);
			printf("length = %d,mclass = 0x%x\n",req->header.length,req->header.mclass);
			n= write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			break;
		}
		case APP_UPLOAD_MESSAGE_CMD:
			{
			char Message[] = "测试-message!";
			memset(buf,0,2048);
			Msg_Upload_Message_Req* req = (Msg_Upload_Message_Req*)buf;
			req->header.length = sizeof(Msg_Upload_Message_Req) + strlen(Message);
			req->header.mclass = APP_UPLOAD_MESSAGE_CMD;
			
			StrToBcd(req->MsID, userid);
			StrToBcd(req->TargetGrpID, "010200020102");
			strcpy(req->Message,Message);
			printf("length = %d,mclass = 0x%x\n",req->header.length,req->header.mclass);
			n= write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			break;
		}
		case APP_UPLOAD_VOICE_CMD:
			{
			char Message[] = "测试 Voice";
			memset(buf,0,2048);
			Msg_Upload_Voice_Req* req = (Msg_Upload_Voice_Req*)buf;
			req->header.length = sizeof(Msg_Upload_Voice_Req) + strlen(Message);
			req->header.mclass = APP_UPLOAD_VOICE_CMD;
			
			StrToBcd(req->MsID, userid);
			StrToBcd(req->TargetGrpID, "010200020102");
			req->name = 1;
			req->lengthtotal= sizeof(req->data);
			req->packseq = 1;
			req->packtotal = 1;
			strcpy(req->data,Message);
			printf("length = %d,mclass = 0x%x\n",req->header.length,req->header.mclass);
			n= write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			break;
		}
		case APP_UPLOAD_PICTURE_CMD:
			{
			char Message[] = "测试 Picture";
			memset(buf,0,2048);
			Msg_Upload_Picture_Req* req = (Msg_Upload_Picture_Req*)buf;
			req->header.length = sizeof(Msg_Upload_Picture_Req) + strlen(Message);
			req->header.mclass = APP_UPLOAD_PICTURE_CMD;
			
			StrToBcd(req->MsID, userid);
			StrToBcd(req->TargetGrpID, "010200020102");
			req->name = 1;
			req->lengthtotal= sizeof(req->data);
			req->packseq = 1;
			req->packtotal = 1;
			//req->data = malloc(512);
			strcpy(req->data,Message);
			printf("length = %d,mclass = 0x%x\n",req->header.length,req->header.mclass);
			n= write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			//free(req->data);
			break;
		}
		case APP_READ_OK_CMD:
			{
			memset(buf,0,2048);
			Msg_Read_OK_Req* req = (Msg_Read_OK_Req*)buf;
			req->header.length = sizeof(Msg_Read_OK_Req);
			req->header.mclass = APP_READ_OK_CMD;
			
			StrToBcd(req->MsID, userid);
			req->ID = 1;
			req->Type= 1;
			printf("length = %d,mclass = 0x%x\n",req->header.length,req->header.mclass);
			n= write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			break;
		}
		case APP_GET_MESSAGE_CMD:
			{
			memset(buf,0,2048);
			Msg_Get_Message_Req* req = (Msg_Get_Message_Req*)buf;
			req->header.length = sizeof(Msg_Get_Message_Req);
			req->header.mclass = APP_GET_MESSAGE_CMD;
			
			StrToBcd(req->MsID, "010200020102");
			printf("length = %d,mclass = 0x%x\n",req->header.length,req->header.mclass);
			n= write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			break;
		}
		case APP_ZF_MESSAGE_CMD:
			{
			char path[] = "/tmp/";
			memset(buf,0,2048);
			Msg_Zf_Message1_Req* req = (Msg_Zf_Message1_Req*)buf;
			req->header.length = sizeof(Msg_Zf_Message1_Req);
			req->header.mclass = APP_ZF_MESSAGE_CMD;
			StrToBcd(req->MsID, userid);
			req->Type 		 = 0;//0或者1
			StrToBcd(req->TargetID, "010200020102");
			req->MessageType = 1;
			strcpy(req->Path,path);
			printf("length = %d,mclass = 0x%x\n",req->header.length,req->header.mclass);
			n= write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			break;
		}
		case APP_EMERGENCY_ALARM_CMD:
			{
			memset(buf,0,2048);
			Msg_Emergency_Alarm_Req* req = (Msg_Emergency_Alarm_Req*)buf;
			req->header.length = sizeof(Msg_Emergency_Alarm_Req);
			req->header.mclass = APP_EMERGENCY_ALARM_CMD;
			StrToBcd(req->MsID, userid);
			req->Lg 		= 123456789;
			req->Lt 		= 987654321;
			req->AlarmType	= 1;
			printf("length = %d,mclass = 0x%x\n",req->header.length,req->header.mclass);
			n= write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			break;
		}
		case APP_CANCEL_EMERGENCY_ALARM_CMD:
			{
			char Remark[] = "Octet String";
			memset(buf,0,2048);
			Msg_Cancel_Emergency_Alarm_Req* req = (Msg_Cancel_Emergency_Alarm_Req*)buf;
			req->header.length = sizeof(Msg_Cancel_Emergency_Alarm_Req);
			req->header.mclass = APP_CANCEL_EMERGENCY_ALARM_CMD;
			StrToBcd(req->MsID, userid);
			req->Type = 1;
			strcpy(req->Remark,Remark);
			printf("length = %d,mclass = 0x%x\n",req->header.length,req->header.mclass);
			n= write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			break;
		}
#if 0
		case SATFI_CONNECT_CMD:
			{
			memset(buf,0,2048);
			Msg_Connect_Req *req = (Msg_Connect_Req*)buf;
			req->header.length = sizeof(Msg_Connect_Req);
			req->header.mclass = SATFI_CONNECT_CMD;
			strncpy(req->sat_imsi, "901059898013791", IMSI_LEN);
			n = write(socket,buf,req->header.length);
			if(n<0) printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			break;
		}
		case SATFI_HEART_BEAT_CMD:
			{
			char users[512]={0};
			time_t now = time(0);
			memset(buf,0,2048);
			Header *p = (Header *)buf;
			int offset = 4;
			p->mclass = SATFI_HEART_BEAT_CMD;
			strncpy(&buf[offset], "901059898013791", IMSI_LEN);
			offset += IMSI_LEN;

			*(unsigned short *)&buf[offset] = strlen(GpsData);
			offset += 2;

			strcpy(&buf[offset], GpsData);
			offset += strlen(GpsData);

			int iCntUser = 0;
			USER *pUser = gp_users;
			USER *q = pUser;
			while(pUser)
			{
				if(now - pUser->update < base.tsc.tsc_timeout)
				{
					strncpy(&users[iCntUser * USERID_LEN], &pUser->userid[USERID_LLEN - USERID_LEN], USERID_LEN); 
					iCntUser ++;
					q = pUser;
					pUser = pUser->next;
				}
				else
				{
					//remove user
					if(pUser == gp_users)
					{
						gp_users = pUser->next;
						free(pUser);
						pUser = gp_users;
						q = pUser;
					}
					else
					{
						q->next = pUser->next;
						free(pUser);
						pUser = q->next;
					}
				}
			}
			*(unsigned short *)&buf[offset] = iCntUser;
			offset += 2;

			strncpy(&buf[offset], users, iCntUser * USERID_LEN);
			offset += iCntUser * USERID_LEN;

			p->length = offset;

			n = write(socket,buf,p->length);
			if (n<=0)
			{
				printf("write return error: errno=%d (%s)\n", errno, strerror(errno));
			}
			else
			{
				base.tsc.tsc_hb_req_ltime = now;
			}

			break;
		}
#endif
		default:
			printf("unknow cmd\n");
			break;
	}
	//return 0;
}

