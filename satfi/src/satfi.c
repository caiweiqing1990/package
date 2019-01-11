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
#include <alsa/asoundlib.h>
#include <linux/soundcard.h>
#include <sys/prctl.h>
#include <linux/sockios.h>
#include <iconv.h>
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <speex/speex.h>
#include "speex/speex_preprocess.h"

#include"log.h"
#include "timer.h"
#include "msg.h"
#include "led_control.h"
#include "server.h"

#define GPS_DATA_FILE		"/etc/GpsData.txt"
#define CALL_RECORDS_FILE	"/etc/CallRecords.txt"
#define SAT_IMEI_FILE		"/etc/configSat.ini"

#define USERID_LLEN 21 	 			//完整的用户ID长度
#define USERID_LEN  12  			//上传TSC服务器时，只需要传送用户ID的后12位，以节约流量
#define IMSI_LEN    15  			//IMEI和IMSI的长度
#define UDP_VOICE_SRCPORT	12056	//对讲语音源端口
#define UDP_VOICE_DSTPORT	12056	//对讲语音目的端口

pthread_mutex_t sat_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t n3g_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t net_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t pack_mutex = PTHREAD_MUTEX_INITIALIZER;

//3G模块状态
enum N3G_STATE {
  N3G_STATE_IDLE = -1,
  N3G_STATE_AT = 0,
  N3G_STATE_AT_W,
  N3G_STATE_IMEI,
  N3G_STATE_IMEI_W,
  N3G_STATE_IMSI,
  N3G_STATE_IMSI_W,
  N3G_STATE_CSQ,
  N3G_STATE_CSQ_W,
  N3G_STATE_DIALING,
  N3G_SIM_NOT_INSERTED,
};

//卫星模块状态
enum SAT_STATE {
  SAT_STATE_IDLE=-1,
  SAT_STATE_AT=0,
  SAT_STATE_AT_W,  //01
  SAT_STATE_IMEI,
  SAT_STATE_IMEI_W,//03
  SAT_STATE_IMSI,
  SAT_STATE_IMSI_W, //5
  SAT_STATE_GPSTRACK_START,
  SAT_STATE_GPSTRACK_START_W,//7
  SAT_STATE_GPSTRACK_STOP,
  SAT_STATE_GPSTRACK_STOP_W, //9
  SAT_STATE_CSQ,
  SAT_STATE_CSQ_W, //11
  SAT_STATE_DIALING,
  SAT_STATE_CFUN,
  SAT_STATE_CFUN_W,//14
  SAT_STATE_SIM_ACTIVE,//15
  SAT_STATE_SIM_ACTIVE_W,
  SAT_STATE_FULL_FUN,
  SAT_STATE_FULL_FUN_W, //18
  SAT_STATE_CREG,
  SAT_STATE_CREG_W,
  SAT_STATE_RESTART, //21
  SAT_STATE_RESTART_W,
  SAT_SIM_NOT_INSERTED,
  SAT_SIM_ARREARAGE,//欠费
  SAT_STATE_REGFAIL,
};

enum SAT_STATE_PHONE {
  SAT_STATE_PHONE_IDLE = 0,		//空闲
  SAT_STATE_PHONE_CLIP,			//设置来电显示
  SAT_STATE_PHONE_CLIP_OK,		//设置来电显示成功	2
  SAT_STATE_PHONE_CLCC,   		//查询是否可进行拨号
  SAT_STATE_PHONE_CLCC_OK,		//可进行拨号
  SAT_STATE_PHONE_ATD_W,		//拨号命令已发送	5
  SAT_STATE_PHONE_ATA_W,		//应答命令已发送
  SAT_STATE_PHONE_ATH_W,		//挂断命令已发送	7
  SAT_STATE_PHONE_RING_COMING,	//电话来电
  SAT_STATE_PHONE_DIALING,		//电话拨号中 9
  SAT_STATE_PHONE_DIALING_RING,	//电话振铃中 10
  SAT_STATE_PHONE_NOANSWER,		//电话无人接听
  SAT_STATE_PHONE_ONLINE,		//电话通话中 12
  SAT_STATE_PHONE_HANGUP,		//电话已挂断
  SAT_STATE_PHONE_COMING_HANGUP,//来电电话已挂断	14
  SAT_STATE_PHONE_DIALINGFAILE,	//拨号失败
  SAT_STATE_PHONE_DIALING_SUCCESS,//接听成功	16
  SAT_STATE_PHONE_DIALING_ERROR,//拨号错误
  SAT_STATE_PHONE_DIALING_CLCC,//查询拨号情况 18
  SAT_STATE_PHONE_DIALING_ATH_W,//拨号失败挂断命令已发送
  SAT_STATE_PHONE_DIALING_FAILE_AND_ERROR,//20
};

typedef  struct _n3g
{
  int n3g_fd;                //3G模块串口文件
  int n3g_available;
  int n3g_status;            //0：command 1：online data
  enum N3G_STATE n3g_state;
  int n3g_hb_seconds;
  int n3g_baud_rate;         //波特率
  int n3g_csq_value;         //信号强度
  int n3g_csq_limit_value;   //信号阈值
  int n3g_csq_ltime;         //得到信号强度的时间
  int n3g_dialing;           //是否正在尝试拨号
  char n3g_ifname[16];
  char n3g_ifname_a[16];
  char n3g_imsi[16];
  char n3g_imei[16];
  char n3g_dev_name[32];     //3G模块串口设备
}N3G;

typedef struct _sat
{
  int sat_fd;                //SAT模块文件
  int sat_fd_message;
  int sat_available;
  int sat_status;            //0：command 1：online data
  enum SAT_STATE sat_state;
  enum SAT_STATE_PHONE sat_state_phone;
  int sat_hb_seconds;
  int sat_baud_rate;         //波特率
  int sat_csq_limit_value;   //信号阈值
  int sat_csq_value;         //信号强度
  int sat_csq_ltime;         //得到信号强的时间
  int sat_calling;           //是否正在进行呼叫
  int sat_dialing;           //是否正在尝试拨号
  int sat_msg_sending;
  char sat_ifname[16];
  char sat_ifname_a[16];
  char sat_imsi[16];
  char sat_imei[16];
  char sat_dev_name[32];     //SAT模块串口设备
  char sat_gps[256];         //SAT模块GPS数据
  int socket;				//主叫用户socket
  char calling_number[32];	//主叫号码
  char called_number[32];	//被叫号码
  int captain_socket;		//船长socket
  time_t start_time;		//开始通话时的时间
  time_t end_time;			//通话挂断时的时间

  char MsID[USERID_LLEN];				//终端ID
  char MsPhoneNum[11];			//船长的电话
  char DesPhoneNum[11];			//对方的电话
  unsigned long long StartTime;	//通话开始时间
  unsigned long long EndTime;	//通话结束时间
  unsigned short CallTime;		//通话时长
  unsigned int Money;			//通话费用/分
  int charge;
  
  int voice_socket_udp;
  struct sockaddr_in clientAddr1;
  struct sockaddr_in clientAddr2;
}SAT;

typedef struct _gps
{
  int gps_fd;               //北斗GPS模块文件
  int gps_baud_rate;        //波特率
  char gps_dev_name[32];    //北斗GPS模块串口设备
  char gps_bd[256];         //北斗GPS数据
  int Lg;					//经度
  char Lg_D;				//东西经
  int Lt;					//纬度
  char Lt_D;				//南北纬
  unsigned long long Date;	//时间戳
  int Speed;				//速度
}GPS;

typedef struct _tsc
{
  int tsc_port;              //TSC服务器端口
  int tsc_timeout;
  int keepalive_interval;
  int tsc_hb_req_ltime;
  int tsc_hb_rsp_ltime;
  char tsc_domain[256];      //TSC服务器：域名
  char tsc_addr[32];         //TSC服务器：IP
  int update_interval;		 //升级间隔
  int SizeTmp;
  int SendBufSize;
  int mss;
  int route[16];
}TSC;

typedef struct _app
{
  int app_port;              //APP监听端口
  int app_timeout;
  char app_addr[32];         //APP服务器：IP
}APP;

typedef struct _omc
{
  int omc_port;
  int omc_timeout;
  char omc_domain[256];
  char omc_addr[32];
}OMC;

typedef struct _dial
{
	int dialing;				//拨打电话中
	char calling_number[15];	//主叫号码
	char called_number[15];		//被叫号码
	int socket;					//主叫用户IP和PORT
}DIAL;

typedef struct _base
{
  N3G n3g;
  SAT sat;
  GPS gps;
  TSC tsc;
  APP app;
  OMC omc;
}BASE;

BASE base = { 0 };
char satfi_version[32] = {0}; //当前satfi版本
int version_num = 0; 			//当前satfi版本
char config_url[512] = {0};	//update.ini在服务器中的路径

void printfhex(unsigned char *buf, int len);
char *make_csq(char *buf, time_t *timep, int csqval);
void CreateSOSMessage(char *Msid, int Lg, int Lt, int AlarmType);
void CreateMessage(char *Msid, char *toPhone, int ID, char *messagedata);
int AppCallUpRsp(int socket, short sat_state_phone);

typedef struct _grpinfo
{
	char GrpID[USERID_LLEN];		//群组ID
	unsigned short Count;			//终端个数
	char msid[500][USERID_LLEN];	//一个群组最多500个终端,可修改
}GRPINFO;

typedef struct _user {
  int socketfd;						//套接字
  char userid[USERID_LLEN];        	//用户ID
  char defgrp[USERID_LLEN];			//默认群组ID
  unsigned short grpCount; 			//群组个数
  GRPINFO grpinfo[30];				//群组个数最多保存30个，可修改
  int udp_voice_packnum;			//用于语音对讲计费，统计收到的语音包数
  int famNumConut;
  char FamiliarityNumber[100][USERID_LLEN];//亲情号码最多100个
  struct _user *next;
}USER;

typedef struct _log {
  char MsID[USERID_LLEN]; 
  void *data;
  struct _log *next;
}LOG;

typedef  struct Pack{
	unsigned int Name;			//消息 语音 图片 唯一识别码
	unsigned short PackSeq; 	//包流水号
	unsigned short Packtotal; 	//语音 图片 总包数
	int offset;					//数据偏移
	char* Data;					//消息 语音 图片数据
	int isSended;
	unsigned short type;		// 0 1 2 消息 语音 图片
	struct Pack *next;
}PACK;

typedef  struct Messge{
	char MsID[USERID_LLEN]; 
	char toPhoneNum[USERID_LLEN];
	int ID;
	char Data[1024];
	struct Messge *next;
}MESSAGE;

typedef  struct GPSData{
	char* Data;
	struct GPSData *next;
}GPSDATA;

#define BUF_SIZE	1024*32
typedef struct _appsocket {
  int AppSocketFd;
  time_t Update;
  int DataSaveOffset;
  char Data[BUF_SIZE];
  char ip[16];
  struct _appsocket *next;
}APPSOCKET;

static APPSOCKET *gp_appsocket = NULL;
static PACK *PackHead = NULL;				//消息 语音 图片 记录
static GPSDATA *GpsDataHead = NULL;			//定位数据
static MESSAGE *MessagesHead = NULL;	//短信记录
static LOG *gp_log = NULL;
static USER *gp_users = NULL;				//用户列表

//发送给TSC的GPS数据，非标准格式
char GpsData[256] = "$GPRMC,0.0,A,0.0,N,0.0,E,0.0,2000,A*60,00,00,00000000,";
//发送给TSC的心跳间隔时间
static int tsc_hb_timeout = 600;

char FixMsID[10] = {0};
int bSatNetWorkActive = 1;//默认激活
int bTscConnected = 0;
int bGetGpsData = 0;
int sock_tsc = -1;
int sock_udp = -1;

int VoltageVal = 0;
#define VOLTAGE_WARNING_ADC_VAL	3500	
int udpvoicetimeout = 60;
int AppCnt = 0;
/* 同步锁
 *
 */
void n3g_lock() { pthread_mutex_lock(&n3g_mutex); }
void n3g_unlock() { pthread_mutex_unlock(&n3g_mutex); }
void sat_lock() { pthread_mutex_lock(&sat_mutex); }
void sat_unlock() { pthread_mutex_unlock(&sat_mutex); }
void net_lock() { pthread_mutex_lock(&net_mutex); }
void net_unlock() { pthread_mutex_unlock(&net_mutex); }

void AnsweringPhone(void);
void StartCallUp(char calling_number[15]);
void Data_To_MsID(char *MsID, void *data);
void Data_To_ExceptMsID(char *MsID, void *data);
void Data_Transmit(char *MsID, void *data);

#define SAT_LINK_DISCONNECT	1
#define SAT_LINK_NORMAL		0

void NotifyAllUserSatState(unsigned short state)
{
	char buf[24] = {0};
	int n;
	MsgHeader *p = (MsgHeader *)buf;

	if(base.sat.sat_calling)
	{
		return;
	}
	
	p->length = 6;
	p->mclass = NOTIFY_ALL_USER_SAT_STATE;

	*((unsigned short*)&buf[4]) = state;

	USER *pUser = gp_users;
	while(pUser)
	{
		if(pUser->socketfd > 0)
		{
			//satfi_log("NOTIFY_ALL_USER_SAT_STATE %d %.21s\n", state, pUser->userid);
			n = write(pUser->socketfd, buf, p->length);
			if(n < 0)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,p->length,__LINE__);	
			}
		}

		pUser = pUser->next;
	}
}

int SaveDataToFile(char *FileName, char *Data, int Length)
{
	int ret;
	int fd;
	
	fd = open(FileName, O_RDWR|O_CREAT, 0644);
	if(fd < 0)
	{
		satfi_log("open %s faile\n", FileName);
		return -1;
	}
	
	ret = lseek(fd,0,SEEK_END);
	if(ret < 0)
	{
		satfi_log("lseek %s faile\n", FileName);
		close(fd);
		return -1;
	}
	
	satfi_log("write %s Size %d OldFileSize %d\n", FileName, Length, ret);
	ret = write(fd, Data, Length);
	if(ret < 0)
	{
		satfi_log("write %s faile\n", FileName);
		close(fd);
		return -1;
	}
	
	close(fd);
	
	return 0;
}

int SaveGpsToFile(char *FileName)
{
	//satfi_log("SaveGpsToFile");
	char buf[1024] = {0};
	Msg_Blind_Area_Gps * req = (Msg_Blind_Area_Gps *)buf;

	req->header.length = sizeof(Msg_Blind_Area_Gps);
	req->header.mclass = BLIND_AREA_GPS;

	if(strlen(base.sat.sat_imei)==0)
	{
		satfi_log("sat_imei is null");
		return -1;
	}
	
	strncpy(req->Sat_IMSI, base.sat.sat_imei, IMSI_LEN);//357975060101153

	//req->Lg = 0;
	req->Lg = base.gps.Lg;
	req->Lg_D = base.gps.Lg_D;
	req->Lt = base.gps.Lt;
	req->Lt_D = base.gps.Lt_D;
	req->Date = base.gps.Date;//1491429600000
	req->Speed = base.gps.Speed;

	satfi_log("SaveGpsToFile imsi:%.15s,Lg:%d,Lg_D:%c,Lt:%d,Lt_D:%c,Date:%llu,Speed:%d,%d\n",
		req->Sat_IMSI, req->Lg, req->Lg_D, 
		req->Lt, req->Lt_D, req->Date, 
		req->Speed,req->header.length);

	if(base.gps.Lg == 0)
	{
		satfi_log("NOT GPS DATA %d %d %c %c", base.gps.Lg, base.gps.Lt, base.gps.Lg_D, base.gps.Lt_D);
	}
	else
	{
		SaveDataToFile(FileName , buf, req->header.length);
	}

	return 0;
}


int DelCallRecordsData(char *FileName)
{
	Header *pHeader = NULL;
	int fd;
	int filesize;
	int leftsize;
	fd = open(FileName, O_RDONLY);
	if(fd < 0)
	{
		satfi_log("No CallRecords\n", FileName);
		return -1;
	}

	filesize = lseek(fd, 0, SEEK_END);
	lseek(fd, 0, SEEK_SET);
	satfi_log("CallRecords FileSize %d\n", filesize);


	unsigned char *data = mmap(NULL, filesize, PROT_READ, MAP_PRIVATE , fd, 0);  
	if(data == (void *)-1)
	{
		satfi_log("mmap %s faile\n", FileName);
		close(fd);
		return -1;
	}

	pHeader = (Header *)data;

	leftsize = filesize - pHeader->length;
	
	satfi_log("DelCallRecords remove leftsize=%d\n", leftsize);

	if(leftsize == 0)
	{
		satfi_log("remove %s\n", FileName);
		remove(FileName);
	}
	else
	{
		remove(FileName);
		SaveDataToFile("/CallRecordsbak.txt" , &(data[pHeader->length]), leftsize);
		rename("/CallRecordsbak.txt", FileName);
	}
	
	munmap(data, filesize);
	close(fd);
	
	return leftsize;	
}


int ReadCallRecordsAndSendToTSC(char *FileName)
{
	Header *pHeader = NULL;
	int ret;
	int fd;
	int filesize;
	unsigned char buf[1024] = {0};
	
	fd = open(FileName, O_RDONLY);
	if(fd < 0)
	{
		//satfi_log("No CallRecords\n", FileName);
		return -1;
	}
	
	filesize = lseek(fd, 0, SEEK_END);
	lseek(fd, 0, SEEK_SET);
	satfi_log("FileSize %d %s\n", filesize, FileName);

	unsigned char *data = mmap(NULL, filesize, PROT_READ, MAP_PRIVATE , fd, 0);  
	if(data == (void *)-1)
	{
		satfi_log("mmap %s faile\n", FileName);
		close(fd);
		return -1;
	}
	
	pHeader = (Header *)data;
	satfi_log("ReadCallRecordsAndSendToTSC 0x%04x %d\n",pHeader->mclass, pHeader->length);
	Data_To_Tsc(pHeader, pHeader->length);
	
	munmap(data, filesize);
	close(fd);
	return 0;
}

int DelGpsData(char *FileName)
{
	Header *pHeader = NULL;
	int fd;
	int filesize;
	int leftsize;
	fd = open(FileName, O_RDONLY);
	if(fd < 0)
	{
		satfi_log("No GpsData\n", FileName);
		return -1;
	}

	filesize = lseek(fd, 0, SEEK_END);
	lseek(fd, 0, SEEK_SET);
	satfi_log("FileSize %d\n", filesize);


	unsigned char *data = mmap(NULL, filesize, PROT_READ, MAP_PRIVATE , fd, 0);  
	if(data == (void *)-1)
	{
		satfi_log("mmap %s faile\n", FileName);
		close(fd);
		return -1;
	}

	pHeader = (Header *)data;

	leftsize = filesize - pHeader->length;
	
	satfi_log("DelGpsData remove leftsize=%d\n", leftsize);

	if(leftsize == 0)
	{
		satfi_log("remove %s\n", FileName);
		remove(FileName);
	}
	else
	{
		remove(FileName);
		SaveDataToFile("/GpsDatabak.txt" , &(data[pHeader->length]), leftsize);
		rename("/GpsDatabak.txt", FileName);
	}
	
	munmap(data, filesize);
	close(fd);
	
	return leftsize;	
}

int ReadGpsDataAndSendToTSC(char *FileName)
{
	Header *pHeader = NULL;
	int ret;
	int fd;
	int filesize;
	unsigned char buf[1024] = {0};
	
	fd = open(FileName, O_RDONLY);
	if(fd < 0)
	{
		satfi_log("No GpsData\n", FileName);
		return -1;
	}
	
	filesize = lseek(fd, 0, SEEK_END);
	lseek(fd, 0, SEEK_SET);
	//satfi_log("ReadGpsDataAndSendToTSC FileSize %d\n", filesize);

	unsigned char *data = mmap(NULL, filesize, PROT_READ, MAP_PRIVATE , fd, 0);  
	if(data == (void *)-1)
	{
		satfi_log("mmap %s faile\n", FileName);
		close(fd);
		return -1;
	}
	
	pHeader = (Header *)data;
	Data_To_Tsc(pHeader, pHeader->length);
	satfi_log("ReadGpsDataAndSendToTSC 0x%04x %d filesize=%d\n",pHeader->mclass, pHeader->length, filesize);
	
	munmap(data, filesize);
	close(fd);

	return 0;
}

int GpsDataADD(char *packdata, unsigned short packsize)
{
	GPSDATA *tmp = NULL;
	GPSDATA *p = GpsDataHead;
	GPSDATA *q = NULL;

	while(p)
	{
		q = p;
		p = p->next;
	}

	if(p == NULL)
	{
		tmp = calloc(1, sizeof(GPSDATA));
		if(tmp == NULL)
		{
			satfi_log("GpsDataADD error %d\n",__LINE__);
			return -1;
		}
		
		tmp->Data = calloc(1, packsize);
		if(tmp->Data == NULL)
		{
			satfi_log("GpsDataADD error %d\n",__LINE__);
			return -1;
		}
				
		memcpy(tmp->Data, packdata, packsize);
		tmp->next = NULL;

		if(GpsDataHead == NULL)
		{
			satfi_log("GpsDataADD packsize1=%d\n",packsize);
			GpsDataHead = tmp;
		}
		else
		{
			satfi_log("GpsDataADD packsize2=%d\n",packsize);
			q->next = tmp;
		}
	}

	return 0;
}


int code_convert (char *tocode, char *fromcode, char *inbuf, size_t *inlen, char *outbuf, size_t *outlen)
{
	iconv_t cd 	= iconv_open (tocode, fromcode);
	if (cd == (iconv_t)-1)
	{
		perror ("iconv_open");
	}

	/* 由于iconv()函数会修改指针，所以要保存源指针 */
	char *tmpin = inbuf;
	char *tmpout = outbuf;
	size_t inbytesleft = *inlen;
	size_t outbytesleft = *outlen;

	size_t ret = iconv (cd, &tmpin, &inbytesleft, &tmpout, &outbytesleft);
	if (ret == -1)
	{
		iconv_close(cd);
		perror("iconv");
	}
	*outlen = *outlen-outbytesleft;
	iconv_close(cd);
	return 0;
}

int CreateMsgData(char *inPhoneNum, char *indata, char *outdata)
{
	unsigned char tmp[1024] = {0};
	unsigned char userdata[1024] = {0};
	
	int i;
	int plen = strlen(inPhoneNum);
	int dlen = strlen(indata);
	int outlen = sizeof(tmp);
	int msglen;
	
	strcat(outdata, "00");//SCA 服务中心号码
	strcat(outdata, "01");//PDU-Type 00表示收，01表示发
	strcat(outdata, "00");//MR
	
	bzero(tmp, sizeof(tmp));
	sprintf(tmp,"%02x", plen);
	//printf("%s\n",tmp);
	strcat(outdata, tmp);//0B 电话号码长度
	strcat(outdata, "81");//国内是A1或81 国际是91

	bzero(tmp, sizeof(tmp));
	if(plen%2)
	{
		//号码长度奇数
		for(i=0; i<plen-1;i+=2)
		{
			tmp[i] = inPhoneNum[i+1];
			tmp[i+1] = inPhoneNum[i];
		}
		tmp[i] = 'F';
		tmp[i+1] = inPhoneNum[i];
		//printf("%s\n",tmp);
	}
	else
	{
		//号码长度偶数
		for(i=0; i<plen;i+=2)
		{
			tmp[i] = inPhoneNum[i+1];
			tmp[i+1] = inPhoneNum[i];
		}
		//printf("%s\n",tmp);
	}
	strcat(outdata, tmp);//电话号码
	strcat(outdata, "00");//PID
	strcat(outdata, "08");//DCS 	00h 7bit数据编码 默认字符集 
						 //		F6h 8bit数据编码 Class1
						 //		08h USC2（16bit）双字节字符集
	
	//bzero(userdata, sizeof(userdata));	
	code_convert("UTF-16BE", "UTF-8", indata, &dlen, userdata, &outlen);

	bzero(tmp, sizeof(tmp));
	sprintf(tmp,"%02x", outlen);
	//printf("%s\n",tmp);
	strcat(outdata, tmp);//UDL 用户数据长度

	bzero(tmp, sizeof(tmp));	
	for (i=0; i<outlen; i++)
	{
		//printf("[%d].%02x ",i,userdata[i]);
		sprintf(&tmp[i*2],"%02x", userdata[i]);
	}
	strcat(outdata, tmp);//UD 用户数据
	//printf("%s\n",msgbuf);	
	msglen = strlen(outdata)/2 - 1;//for AT+CMGS=msglen
	return msglen;
}

void CheckisHaveMessageAndTriggerSend(int Serialfd)
{
	if(MessagesHead != NULL)
	{
		satfi_log("CheckisHaveMessageAndTriggerSend %d", base.sat.sat_status);
		if(Serialfd > 0)
		{
			char buf[128];
			sprintf(buf, "AT+CMGS=%d\r", strlen(MessagesHead->Data)/2 - 1);
			uart_send(Serialfd, buf, strlen(buf));
			satfi_log("CheckisHaveMessageAndTriggerSend %s", buf);
			base.sat.sat_msg_sending = 1;
		}
	}
}

void MessageSend(int Serialfd)
{
	pthread_mutex_lock(&pack_mutex);
	if(MessagesHead != NULL)
	{
		if(Serialfd > 0)
		{
			int mL=strlen(MessagesHead->Data);
			//satfi_log("%s %d\n",MessagesHead->Data, strlen(MessagesHead->Data));
			MessagesHead->Data[mL] = 0x1a;
			uart_send(Serialfd, MessagesHead->Data, mL+1);
		}
	}
	pthread_mutex_unlock(&pack_mutex);
}

// 7-bit编码
// pSrc: 源字符串指针
// pDst: 目标编码串指针
// nSrcLength: 源字符串长度
// 返回: 目标编码串长度
int gsmEncode7bit(const char* pSrc, unsigned char* pDst, int nSrcLength)
{
	int nSrc;        // 源字符串的计数值
	int nDst;        // 目标编码串的计数值
	int nChar;       // 当前正在处理的组内字符字节的序号，范围是0-7
	unsigned char nLeft;    // 上一字节残余的数据
	// 计数值初始化
	nSrc = 0;
	nDst = 0;
	// 将源串每8个字节分为一组，压缩成7个字节
	// 循环该处理过程，直至源串被处理完
	// 如果分组不到8字节，也能正确处理
	while(nSrc<=nSrcLength)
	{
		// 取源字符串的计数值的最低3位
		nChar = nSrc & 7;
		// 处理源串的每个字节
		if(nChar == 0)
		{
			// 组内第一个字节，只是保存起来，待处理下一个字节时使用
			nLeft = *pSrc;
		}
		else
		{
			// 组内其它字节，将其右边部分与残余数据相加，得到一个目标编码字节
			*pDst = (*pSrc << (8-nChar)) | nLeft;
			// 将该字节剩下的左边部分，作为残余数据保存起来
			nLeft = *pSrc >> nChar;
			// 修改目标串的指针和计数值
			pDst++;
			nDst++;
		}
		// 修改源串的指针和计数值
		pSrc++;
		nSrc++;
	}
	// 返回目标串长度
	return nDst;
}


// 7-bit解码
// pSrc: 源编码串指针
// pDst: 目标字符串指针
// nSrcLength: 源编码串长度
// 返回: 目标字符串长度
int gsmDecode7bit(const unsigned char* pSrc, char* pDst, int nSrcLength)
{
	int nSrc;        // 源字符串的计数值
	int nDst;        // 目标解码串的计数值
	int nByte;       // 当前正在处理的组内字节的序号，范围是0-6
	unsigned char nLeft;    // 上一字节残余的数据
	// 计数值初始化
	nSrc = 0;
	nDst = 0;
	// 组内字节序号和残余数据初始化
	nByte = 0;
	nLeft = 0;
	// 将源数据每7个字节分为一组，解压缩成8个字节
	// 循环该处理过程，直至源数据被处理完
	// 如果分组不到7字节，也能正确处理
	while(nSrc<nSrcLength)
	{
		// 将源字节右边部分与残余数据相加，去掉最高位，得到一个目标解码字节
		*pDst = ((*pSrc << nByte) | nLeft) & 0x7f;
		// 将该字节剩下的左边部分，作为残余数据保存起来
		nLeft = *pSrc >> (7-nByte);
		// 修改目标串的指针和计数值
		pDst++;
		nDst++;
		// 修改字节计数值
		nByte++;
		// 到了一组的最后一个字节
		if(nByte == 7)
		{
			// 额外得到一个目标解码字节
			*pDst = nLeft;
			// 修改目标串的指针和计数值
			pDst++;
			nDst++;
			// 组内字节序号和残余数据初始化
			nByte = 0;
			nLeft = 0;
		}
		// 修改源串的指针和计数值
		pSrc++;
		nSrc++;
	}
	*pDst = 0;
	// 返回目标串长度
	return nDst;
}


int MessageParse(char *indata, char *OAphonenum, char *Decode)
{
	unsigned char tmp[4] = {0};
	unsigned char Encode[1024] = {0};
	//unsigned char Decode[1024] = {0};
	
	char SCAphonenum[24] = {0}; //短消息服务中心号码
	//char OAphonenum[24] = {0};  //发送方地址（手机号码）
	char SCTS[24] = {0};		//消息中心收到消息时的时间戳
	int i;
	
	int pos=0;
	tmp[0] = indata[pos];pos +=1;
	tmp[1] = indata[pos];pos +=1;
	tmp[2] = 0;
	int SCALen = strtol(tmp, NULL, 16);
	//printf("SCALen=%d\n",SCALen);
	
	strncpy(SCAphonenum, &indata[pos], SCALen*2);pos +=SCALen*2;
	//printf("SCAphonenum=%s\n",SCAphonenum);
	
	tmp[0] = indata[pos];pos +=1;
	tmp[1] = indata[pos];pos +=1;
	tmp[2] = 0;
	int PDUType = atoi(tmp);
	//printf("PDUType=%d\n",PDUType);

	tmp[0] = indata[pos];pos +=1;
	tmp[1] = indata[pos];pos +=1;
	tmp[2] = 0;
	int OALen = strtol(tmp, NULL, 16);
	//printf("OALen=%d\n",OALen);
	
	strncpy(OAphonenum, &indata[pos+2], OALen+1);pos +=OALen+1+2;
	for(i=0;i<OALen;i+=2)
	{
		char c = OAphonenum[i];
		OAphonenum[i] = OAphonenum[i+1];
		if(i+1 < OALen)
		{
			OAphonenum[i+1] = c;
		}
		else
		{
			OAphonenum[i+1] = 0;			
		}
	}
	//satfi_log("OAphonenum=%s\n",OAphonenum);							 //发送方地址（手机号码）
	
	tmp[0] = indata[pos];pos +=1;
	tmp[1] = indata[pos];pos +=1;
	tmp[2] = 0;
	int PID = strtol(tmp, NULL, 16);
	//printf("PID=%d\n",PID);
	
	tmp[0] = indata[pos];pos +=1;
	tmp[1] = indata[pos];pos +=1;
	tmp[2] = 0;
	int DCS = strtol(tmp, NULL, 16);
	//printf("DCS=%d\n",DCS);						//用户数据编码方案
	
	strncpy(SCTS, &indata[pos], 14);pos +=14;
	//printf("SCTS=%s\n",SCTS);					//消息中心收到消息时的时间戳
	
	tmp[0] = indata[pos];pos +=1;
	tmp[1] = indata[pos];pos +=1;
	tmp[2] = 0;
	int UDL = strtol(tmp, NULL, 16);			//用户数据长度
	//printf("UDL=%d\n",UDL);
	//printf("UD=%s\n",&indata[pos]);
	
	for (i=0; i<UDL; i++)
	{
		tmp[0] = indata[pos];pos +=1;
		tmp[1] = indata[pos];pos +=1;
		tmp[2] = 0;
		Encode[i] = strtol(tmp, NULL, 16);
		//printf("%02x ",data[i]);
	}
	
	if(DCS == 0x08)
	{
		int outlen=1024;
		code_convert("UTF-8", "UTF-16BE", Encode, &UDL, Decode, &outlen);
		//satfi_log("%s\n", Decode);
		return outlen;
	}
	else if(DCS == 0x00)
	{
		return gsmDecode7bit(Encode, Decode, UDL);
		//satfi_log("%s\n", Decode);
	}
}


int MessageADD(char *MsID, char *toPhoneNum, int ID, unsigned char *data, unsigned int len)
{
	pthread_mutex_lock(&pack_mutex);
	MESSAGE *tmp = NULL;
	MESSAGE *p = MessagesHead;
	MESSAGE *q = NULL;

	while(p)
	{
		if(p->ID > 0 && p->ID == ID)
		{
			satfi_log("Message exist ID=%d", ID);
			break;
		}
		q = p;
		p = p->next;
	}

	if(p == NULL)
	{
		tmp = calloc(1, sizeof(MESSAGE));
		if(tmp == NULL)
		{
			satfi_log("MessageADD error %d\n",__LINE__);
			pthread_mutex_unlock(&pack_mutex);
			return -1;
		}

		bzero(tmp->MsID, USERID_LLEN);		
		bzero(tmp->toPhoneNum, USERID_LLEN);
		
		if(MsID != NULL)
		{
			memcpy(tmp->MsID, MsID, USERID_LLEN);
		}

		if(toPhoneNum != NULL)
		{
			memcpy(tmp->toPhoneNum, toPhoneNum, USERID_LLEN);
		}

		tmp->ID = ID;
		bzero(tmp->Data, sizeof(tmp->Data));
		memcpy(tmp->Data, data, len);
		tmp->next = NULL;
		
		if(MessagesHead == NULL)
		{
			satfi_log("MessageADD len1=%d\n",len);
			MessagesHead = tmp;
		}
		else
		{
			satfi_log("MessageADD len2=%d\n",len);
			q->next = tmp;
		}
	}
	pthread_mutex_unlock(&pack_mutex);

	return 0;
}


void MessageDel(void)
{
	pthread_mutex_lock(&pack_mutex);
	MESSAGE *p = MessagesHead;
	MESSAGE *n = NULL;
	
	if(p)
	{
		MessagesHead = p->next;
		satfi_log("MessageDel MsID=%.21s %.21s %s\n",p->MsID, p->toPhoneNum, p->Data);
		free(p);
	}
	pthread_mutex_unlock(&pack_mutex);
}


int Message_Pack_Add(char *packdata, unsigned short packsize, unsigned int Name)
{
	pthread_mutex_lock(&pack_mutex);
	PACK *tmp = NULL;
	PACK *p = PackHead;
	PACK *q = NULL;
	//int cnt=0;

	while(p)
	{
		if(p->Name == Name)
		{
			satfi_log("Message_Pack Exist Name=%d\n", Name);
			break;
		}
		//cnt++;
		q = p;
		p = q->next;
	}

	if(p == NULL)
	{
		tmp = calloc(1, sizeof(PACK));
		if(tmp == NULL)
		{
			satfi_log("Message_Pack_Add error %d\n",__LINE__);
			pthread_mutex_unlock(&pack_mutex);
			return -1;
		}
		
		tmp->Data = calloc(1, packsize);
		if(tmp->Data == NULL)
		{
			satfi_log("Message_Pack_Add error %d\n",__LINE__);
			pthread_mutex_unlock(&pack_mutex);
			return -1;
		}
				
		tmp->Name = Name;
		tmp->PackSeq = 0;
		tmp->Packtotal = 0;
		tmp->offset = 0;
		memcpy(tmp->Data, packdata, packsize);
		tmp->isSended = 0;
		tmp->type = 0;
		tmp->next = NULL;

		if(PackHead == NULL)
		{
			//satfi_log("Message_Pack_Add1 packsize=%d Name=%d %d\n",packsize, Name, cnt);
			PackHead = tmp;
		}
		else
		{
			//satfi_log("Message_Pack_Add2 packsize=%d Name=%d %d\n",packsize, Name, cnt);
			q->next = tmp;
		}
	}
	pthread_mutex_unlock(&pack_mutex);
	return 0;
}

int Voice_Pic_Pack_Add(unsigned short type, char *packdata, int packsize, unsigned short PackSeq, unsigned short Packtotal, unsigned int Name)
{
	PACK *tmp = NULL;
	PACK *p = NULL;
	PACK *q = NULL;
	//int cnt=0;
	//satfi_log("Voice_Pic_Pack_Add pack_mutex lock %d",__LINE__);
	pthread_mutex_lock(&pack_mutex);
	if(PackHead == NULL)
	{
		tmp = calloc(1, sizeof(PACK));
		if(tmp == NULL)
		{
			satfi_log("Voice_Pic_Pack_Add error %d\n",__LINE__);
			pthread_mutex_unlock(&pack_mutex);
			return -1;
		}
		
		tmp->Data = calloc(Packtotal, packsize);
		if(tmp->Data == NULL)
		{
			satfi_log("Voice_Pic_Pack_Add error %d\n",__LINE__);
			pthread_mutex_unlock(&pack_mutex);
			return -1;
		}

		//satfi_log("Voice_Pic_Pack_Add1 PackSeq=%d Packtotal=%d Name=%d\n",PackSeq,Packtotal,Name);
		
		tmp->Name = Name;
		tmp->PackSeq = PackSeq;
		tmp->Packtotal = Packtotal;
		memcpy(tmp->Data, packdata, packsize);
		tmp->offset = packsize;
		tmp->isSended = 0;
		tmp->type = type;
		tmp->next = NULL;

		PackHead = tmp;
	}
	else
	{
		p = PackHead;
		while(p)
		{
			if(p->Name == Name)
			{
				break;
			}
			q = p;
			p = q->next;
			//cnt++;
		}
		
		if(p == NULL)
		{
			tmp = calloc(1, sizeof(PACK));
			if(tmp == NULL)
			{
				satfi_log("Voice_Pic_Pack_Add error %d\n",__LINE__);
				pthread_mutex_unlock(&pack_mutex);
				return -1;
			}
			
			tmp->Data = calloc(Packtotal, packsize);
			if(tmp->Data == NULL)
			{
				satfi_log("Voice_Pic_Pack_Add error %d\n",__LINE__);
				pthread_mutex_unlock(&pack_mutex);
				return -1;
			}

			//satfi_log("Voice_Pic_Pack_Add2 PackSeq=%d Packtotal=%d Name=%d %d\n",PackSeq,Packtotal,Name,cnt);
			
			tmp->Name = Name;
			tmp->PackSeq = PackSeq;
			tmp->Packtotal = Packtotal;
			memcpy(tmp->Data, packdata, packsize);
			tmp->offset = packsize;
			tmp->isSended = 0;
			tmp->type = type;
			tmp->next = NULL;
			
			q->next = tmp;
		}
		else if(PackSeq > p->PackSeq)
		{
			//if(PackSeq == p->Packtotal)satfi_log("PackSeq=%d Packtotal=%d Name=%d\n",PackSeq, p->Packtotal, Name);
			//satfi_log("PackSeq=%d Name=%d Packtotal=%d packsize=%d\n",PackSeq, Name, p->Packtotal, packsize);
			int dataoffset = p->offset;
			memcpy(&(p->Data[dataoffset]), packdata, packsize);
			p->PackSeq = PackSeq;
			p->offset += packsize;
		}
		else
		{
			//satfi_log("Voice_Pic_Exist %d\n", Name);
		}
	}

	pthread_mutex_unlock(&pack_mutex);
	//satfi_log("Voice_Pic_Pack_Add pack_mutex unlock %d",__LINE__);
	return 0;
}

int Pack_Del(unsigned int Name)
{
	pthread_mutex_lock(&pack_mutex);
	PACK *p = PackHead;
	PACK *q = NULL;
	PACK *n = NULL;

	while(p)
	{
		//satfi_log("Pack_Name %d %d\n",p->Name,Name);
		if(p->Name == Name)
		{
			//satfi_log("Pack_Find %d\n",p->Name);
			break;
		}
		q = p;		
		p = q->next;
	}
	
	if(p)
	{
		n = p->next;
		
		if(p == PackHead)
		{
			PackHead = n;
		}
		else
		{
			q->next = n;
		}
		satfi_log("Pack_Del %d\n",p->Name);
		free(p->Data);
		free(p);
	}

	pthread_mutex_unlock(&pack_mutex);
	return 0;
}

int Data_To_Tsc(void *data, int len)
{
	int n;
	if(sock_tsc<0)
	{
		return 0;
	}
	
	if(PackHead == NULL)
	{
		//satfi_log("PackHead NULL");
		base.tsc.tsc_hb_req_ltime = time(0);
	}
	
	if(base.tsc.SizeTmp > 0)
	{
		base.tsc.SizeTmp += len;
		base.tsc.SendBufSize += len;
	}
	
	n = write(sock_tsc, data, len);
	return n;
}

int Data_To_Tsc_No_Rsp(void *data, int len)
{
	int n;
	if(sock_tsc<0)
	{
		return 0;
	}
	
	if(base.tsc.SizeTmp > 0)
	{
		base.tsc.SizeTmp += len;
		base.tsc.SendBufSize += len;
	}
	
	n = write(sock_tsc, data, len);
	return n;
}

void Close_Tsc_Socket(void)
{
	int size = 0;
	ioctl(sock_tsc, SIOCOUTQ, &size);
	satfi_log("Close_Tsc leftsize=%d", size);
	close(sock_tsc);
	sock_tsc = -1;
	base.tsc.tsc_hb_req_ltime = 0;
	base.tsc.tsc_hb_rsp_ltime = 0;
	bTscConnected = 0;
}

int SendPackToTSC(void)
{
	Header *pHeader = NULL;
	static int DataOffsetPic = 0;
	static int DataOffsetVic = 0;
	PACK *p;

	static int diffcnt1 = 0;
	static int cnt1 = 0;
	
	static int diffcnt2 = 0;
	static int cnt2 = 0;

	static int cnt3 = 0;
	
	if(bTscConnected == 0)
	{
		DataOffsetPic = 0;
		DataOffsetVic = 0;
		p = PackHead;
		while(p)
		{
			p->isSended = 0;
			p = p->next;
		}

		cnt3 = 0;
		return -1;
	}

	if(PackHead != NULL)
	{
		p = PackHead;
		while(p)
		{
			ioctl(sock_tsc, SIOCOUTQ, &(base.tsc.SendBufSize));
			if(base.tsc.SendBufSize >= base.tsc.mss)
			{
				//satfi_log("No Send");
				return 0;
			}
			
			if(p->type == 0 && p->isSended == 0 && p->PackSeq == 0 && p->Packtotal == 0)
			{
				pHeader = (Header *)p->Data;
				Data_To_Tsc(pHeader, pHeader->length);
				p->isSended = 1;
				satfi_log("Send Msg Finish ID=%d", p->Name);				
			}
			p = p->next;
		}

		p = PackHead;
		while(p)
		{			
			ioctl(sock_tsc, SIOCOUTQ, &(base.tsc.SendBufSize));
			if(base.tsc.SendBufSize >= base.tsc.mss)
			{
				//satfi_log("No Send");
				return 0;
			}
			
			if(p->type == 1 && p->isSended == 0 && p->Packtotal != 0 && p->PackSeq < p->Packtotal)
			{
				if(diffcnt1 == p->Packtotal - p->PackSeq)
				{
					cnt1++;
					satfi_log("viccnt1=%d", cnt1);
					if(cnt1 >= 30)
					{
						cnt1 = 0;
						diffcnt1 = 0;
						Pack_Del(p->Name);
					}
				}
				else
				{
					cnt1 = 0;
					diffcnt1 = p->Packtotal - p->PackSeq;
				}
				break;
			}
			
			if(p->type == 1 && p->isSended == 0 && p->Packtotal != 0 && p->PackSeq == p->Packtotal)
			{
				satfi_log("VIC %d %d %d",p->offset, DataOffsetVic, base.tsc.SendBufSize);
				int hadsendlen = base.tsc.SendBufSize;
				while(1)
				{
					pHeader = (Header *)&(p->Data[DataOffsetVic]);
					Data_To_Tsc(pHeader, pHeader->length);
					DataOffsetVic += pHeader->length;
					
					if(DataOffsetVic == p->offset)
					{
						satfi_log("VIC Send Finish ID=%d", p->Name);					
						DataOffsetVic = 0;
						p->isSended = 1;
						break;
					}
					
					hadsendlen += pHeader->length;
					if(hadsendlen >= base.tsc.mss)
					{
						break;
					}
				}
				break;
			}
			p = p->next;
		}
		
		p = PackHead;
		while(p)
		{			
			ioctl(sock_tsc, SIOCOUTQ, &(base.tsc.SendBufSize));
			if(base.tsc.SendBufSize >= base.tsc.mss)
			{
				//satfi_log("No Send");
				return 0;
			}
			
			if(p->type == 2 && p->isSended == 0 && p->Packtotal != 0 && p->PackSeq < p->Packtotal)
			{
				if(diffcnt2 == p->Packtotal - p->PackSeq)
				{
					cnt2++;
					satfi_log("piccnt2=%d", cnt2);
					if(cnt2 >= 30)
					{
						cnt2 = 0;
						diffcnt2 = 0;
						Pack_Del(p->Name);
					}
				}
				else
				{
					cnt2 = 0;
					diffcnt2 = p->Packtotal - p->PackSeq;
				}
				break;
			}
			
			if(p->type == 2 && p->isSended == 0 && p->Packtotal != 0 && p->PackSeq == p->Packtotal)
			{
				satfi_log("PIC %d %d %d",p->offset, DataOffsetPic, base.tsc.SendBufSize);
				int hadsendlen = base.tsc.SendBufSize;
				while(1)
				{
					pHeader = (Header *)&(p->Data[DataOffsetPic]);
					Data_To_Tsc(pHeader, pHeader->length);
					DataOffsetPic += pHeader->length;
					
					if(DataOffsetPic == p->offset)
					{
						satfi_log("PIC Send Finish ID=%d", p->Name);					
						DataOffsetPic = 0;
						p->isSended = 1;
						break;
					}
					
					hadsendlen += pHeader->length;
					if(hadsendlen >= base.tsc.mss)
					{
						break;
					}
				}
				break;
			}
			p = p->next;
		}

		if(base.tsc.SendBufSize == 0)
		{
			p = PackHead;
			while(p)
			{
				if(p->isSended == 1)
				{
					cnt3++;
					satfi_log("cnt3=%d", cnt3);
					if(cnt3 >= 10)
					{
						satfi_log("Pack_Delcnt3=%d", cnt3);
						cnt3 = 0;
						char buf[256];
						if(p->type == 0)
						{
							Msg_Upload_Message_Req *req = (Msg_Upload_Message_Req *)p->Data;
						
							MsgUploadMessageRsp *rsp = (MsgUploadMessageRsp *)buf;
							rsp->header.length = sizeof(MsgUploadMessageRsp);
							if(req->header.mclass == APP_UPLOAD_MESSAGE_CMD)
							{
								rsp->header.mclass = UPLOAD_MESSAGE_RSP;	
							}
							else if(req->header.mclass == APP_GRP_UPLOAD_MESSAGE_CMD)
							{
								rsp->header.mclass = APP_GRP_UPLOAD_MESSAGE_RSP;
							}
							memcpy(rsp->MsID, FixMsID, 9);
							BcdToStr(&(rsp->MsID[9]), req->MsID, 6);
							rsp->Result = 0;
							rsp->Name = p->Name;
							
							Data_To_MsID(rsp->MsID, buf);
							Pack_Del(p->Name);
							log_insert(rsp->MsID, buf, rsp->header.length);
						}
						else if(p->type == 1)
						{
							Msg_Upload_Voice_Req *req = (Msg_Upload_Voice_Req *)p->Data;
						
							MsgUploadVoiceRsp	*rsp = (MsgUploadVoiceRsp *)buf;
							rsp->header.length = 31;
							if(req->header.mclass == APP_UPLOAD_VOICE_CMD)
							{
								rsp->header.mclass = UPLOAD_VOICE_RSP;	
							}
							else if(req->header.mclass == APP_GRP_UPLOAD_VOICE_CMD)
							{
								rsp->header.mclass = APP_GRP_UPLOAD_VOICE_RSP;
							}
							memcpy(rsp->MsID, FixMsID, 9);
							BcdToStr(&(rsp->MsID[9]), req->MsID, 6);
							rsp->Result = 0;
							rsp->Name = p->Name;
							
							Data_To_MsID(rsp->MsID, buf);
							Pack_Del(p->Name);
							log_insert(rsp->MsID, buf, rsp->header.length);
						}
						else if(p->type == 2)
						{
							Msg_Upload_Picture_Req *req = (Msg_Upload_Picture_Req *)p->Data;
						
							MsgUploadPictureRsp *rsp = (MsgUploadPictureRsp *)buf;
							rsp->header.length = 31;
							if(req->header.mclass == APP_UPLOAD_PICTURE_CMD)
							{
								rsp->header.mclass = UPLOAD_PICTURE_RSP;	
							}
							else if(req->header.mclass == APP_GRP_UPLOAD_PICTURE_CMD)
							{
								rsp->header.mclass = APP_GRP_UPLOAD_PICTURE_RSP;
							}
							memcpy(rsp->MsID, FixMsID, 9);
							BcdToStr(&(rsp->MsID[9]), req->MsID, 6);
							rsp->Result = 0;
							rsp->Name = p->Name;
							
							Data_To_MsID(rsp->MsID, buf);
							Pack_Del(p->Name);
							log_insert(rsp->MsID, buf, rsp->header.length);
						}
					}
					break;
				}
				p = p->next;
			}
		}
		else
		{
			cnt3 = 0;
		}
	}
	return 0;
}

int log_insert(char *MsID, void *data, unsigned int datalen)
{
	LOG *log = gp_log;
	while(log)
	{
		if(strncmp(log->MsID, MsID, USERID_LLEN) == 0)
		{
			log->data = realloc(log->data, datalen);
			if(log->data)
			{
				satfi_log("log_insert %.21s datalen1=%d\n",MsID,datalen);
				memcpy(log->data, data, datalen);
			}
			break;
		}
		
		log = log->next;
	}

	if(log == NULL)
	{
		log = (LOG *)malloc(sizeof(LOG));
		if(log)
		{
			bzero(log,sizeof(LOG));
			memcpy(log->MsID, MsID, USERID_LLEN);			
			log->data = malloc(datalen);
			if(log->data)
			{
				satfi_log("log_insert %.21s datalen2=%d\n",MsID,datalen);
				memcpy(log->data, data, datalen);
			}
			else
			{	
				free(log);
				return -1;
			}
			
			log->next = gp_log;
			gp_log = log;
		}
		else
		{
			return -1;
		}
	}
	
	return 0;
}

void* get_log_data(char *MsID)
{
	LOG *log = gp_log;
	LOG *q = log;
	void * data = NULL;
	while(log)
	{
		if(strncmp(log->MsID, MsID, USERID_LLEN) == 0)
		{
			//satfi_log("get_log_data %.21s\n",MsID);
			data = log->data;
			break;
		}
		q = log;
		log = log->next;
	}
	
	return data;
}

/* 初始化串口设备
 * @fd
 * @device
 * @baud_rate
 */
int init_serial(int *fd, char *device, int baud_rate)
{
	satfi_log("open serial port : %s ...\n", device);
	int fd_serial = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd_serial < 0)
	{
		perror("open fd_serial");
		return -1;
	} 

	*fd = fd_serial;

	/* 串口主要设置结构体termios <termios.h> */
	struct termios options;

	/* 1.tcgetattr()用于获取与终端相关的参数
	*	参数fd为终端的文件描述符，返回的结果保存在termios结构体中
	*/
	tcgetattr(fd_serial, &options);

	switch(baud_rate)
	{
		case 2400:
			baud_rate = B2400;		
		break;
		case 4800:
			baud_rate = B4800;			
		break;
		case 9600:
			baud_rate = B9600;			
		break;
		case 19200: 
			baud_rate = B19200; 		
		break;
		case 38400: 	
			baud_rate = B38400; 		
		break;
		case 115200:			
			baud_rate = B115200;			
		default:
		break;
	}	
	
	/* 2.修改获得的参数 */
	options.c_cflag |= CLOCAL | CREAD; /* 设置控制模块状态：本地连接，接收使能 */
	options.c_cflag &= ~CSIZE;		   /* 字符长度，设置数据位之前，一定要屏蔽这一位 */
	options.c_cflag &= ~CRTSCTS;	   /* 无硬件流控 */
	options.c_cflag |= CS8; 		   /* 8位数据长度 */
	options.c_cflag &= ~CSTOPB; 	   /* 1位停止位 */
	options.c_iflag |= IGNPAR;		   /* 无奇偶校验 */
	options.c_oflag = 0;			   /* 输出模式 */
	options.c_lflag = 0;			   /* 不激活终端模式 */
	cfsetospeed(&options, baud_rate);	 /* 设置波特率 */

	/* 3.设置新属性: TCSANOW，所有改变立即生效 */
	tcflush(fd_serial, TCIFLUSH);	   /* 溢出数据可以接收，但不读 */
	tcsetattr(fd_serial, TCSANOW, &options);

	satfi_log("open serial port : %s successfully!!!\n", device);
	return 0;
}


/* 串口发送数据
 * @fd:     串口描述符
 * @data:   待发送数据
 * @datalen:数据长度
 */
int uart_send(int fd, char *data, int datalen)
{
  int len = 0;
  len = write(fd, data, datalen); /* 实际写入的长度 */
  if (len == datalen)
  {
    return len;
  }
  else
  {
    tcflush(fd, TCOFLUSH); /* TCOFLUSH，刷新写入的数据，但不传送 */
    return -1;
  }

  return 0;
}

/* 串口接收数据 */
int uart_recv(int fd, char *data, int datalen)
{
  int len = 0, ret = 0;
  fd_set fs_read;
  struct timeval tv = {1,0};

  FD_ZERO(&fs_read);
  FD_SET(fd, &fs_read);

  ret = select(fd+1, &fs_read, NULL, NULL, &tv);

  if (ret>0)
  {
    if(FD_ISSET(fd, &fs_read))
    {
      len = read(fd, data, datalen);
      return len;
    }
  }

  return 0;
}


#define MAXLINE 1024
int detect_interface(char *ifname)
{
  FILE *fp;
  char result[MAXLINE],command[MAXLINE];
  snprintf(command,sizeof(command),"ifconfig %s", ifname);
  fp=popen(command,"r");
  if(NULL==fp) return -1;
  while(fgets(result,sizeof(result),fp)!=NULL)
  {
    if('\n' == result[strlen(result)-1])
      result[strlen(result)-1] = '\0';
    //satfi_log("%s\n",result);
  }
  int rc = pclose(fp);
  return rc;
}

int myexec(char *command, char *result, int *maxline)
{
  FILE *pp = popen(command, "r");
  if(NULL == pp) return -1;
  //satfi_log("execute shell : %s\n", command);
  char tmp[MAXLINE];
  if(result!=NULL)
  {
    int line=0;
    while(fgets(tmp,sizeof(tmp),pp)!=NULL)
    {
      if('\n' == tmp[strlen(tmp)-1]) tmp[strlen(tmp)-1] = '\0';
      if(maxline!=NULL && line++<*maxline) strcat(result,tmp);
      //satfi_log("tmp=%s\n", tmp);
    }
    *maxline = line;
  }
  int rc = pclose(pp);
  return rc;
}

int SetSystemTime(time_t t)  
{
    struct timeval tv;  
	
    tv.tv_sec = t;  //北京时间
    tv.tv_usec = 0;  
    if(settimeofday (&tv, NULL) < 0)  
    {
		satfi_log("Set system time error!");  
		return -1;  
    }
	else
	{
		//satfi_log("Set system time Success!"); 
	}

    return 0;  
}

/* 解析北斗GPS模块GPS数据
 *
 */
int parseGpsData(char *buf, int len)
{
	char data[32][12] = {};
	char *del = ",";
	int i = 0;
	double Speed;
	double Lg;
	double Lt;
	char Lg_D;
	char Lt_D;

	int tm_sec;
	int tm_min; 
	int tm_hour; 
	int tm_mday;
	int tm_mon;
	int tm_year;

	char *p = strtok(buf, del);
	while(p!=NULL)
	{
		strncpy(&data[i++][0], p, 11);
		p = strtok(NULL, del);
		//if(p) printf("*** %02d : %s\n", i, p);
	}

	if(data[2][0]!='A')
	{
		bGetGpsData = 0;
		base.gps.Lg 	= 0;
		base.gps.Lg_D 	= 0;
		base.gps.Lt 	= 0;
		base.gps.Lt_D 	= 0;
		base.gps.Date 	= 0;
		base.gps.Speed	= 0;
		return 0;
	}
	
	strcpy(GpsData, "$GPRMC,"); // $GNRMC,$GPRMC --> $GPRMC
	strcat(GpsData, data[1]);   // UTC
	if(strncmp(data[0],"$GNRMC",6)==0)
	strcat(GpsData, "0,");      // 补一个0
	strcat(GpsData, data[2]);   // A
	strcat(GpsData, ",");
	strcat(GpsData, data[3]);   // 纬度
	strcat(GpsData, ",");
	strcat(GpsData, data[4]);   // 南北半球
	strcat(GpsData, ",");
	strcat(GpsData, data[5]);   // 经度
	strcat(GpsData, ",");
	strcat(GpsData, data[6]);   // 东西半球
	strcat(GpsData, ",");
	strcat(GpsData, data[7]);   // 地面速度
	strcat(GpsData, ",");
	strcat(GpsData, "2000");   // 定位精度
	strcat(GpsData, ",A*60,00,00,00000000,");
	//printf("%s\n",buf);
	
	sscanf(data[7],"%lf",&Speed);
	
	sscanf(data[5],"%lf",&Lg);
	sscanf(data[3],"%lf",&Lt);
	
	sscanf(data[6],"%c",&Lg_D);
	sscanf(data[4],"%c",&Lt_D);
	
	tm_hour = (data[1][0] - '0') * 10 + (data[1][1] - '0');
	tm_min = (data[1][2] - '0') * 10 + (data[1][3] - '0');
	tm_sec = (data[1][4] - '0') * 10 + (data[1][5] - '0');

	tm_mday = (data[9][0] - '0') * 10 + (data[9][1] - '0');
	tm_mon = (data[9][2] - '0') * 10 + (data[9][3] - '0');
	tm_year = (data[9][4] - '0') * 10 + (data[9][5] - '0');
	
	//printf("%d %d %d %d %d %d \n",
	//tm_year, tm_mon, tm_mday, tm_hour, tm_min, tm_sec);

	struct tm Tm;
	
	Tm.tm_year = tm_year + 100 ;
	Tm.tm_mon = tm_mon - 1;
	Tm.tm_mday = tm_mday;
	Tm.tm_hour = tm_hour + 8;
	Tm.tm_min = tm_min;
	Tm.tm_sec = tm_sec;
	
	base.gps.Lg 	= (int)(Lg*10000);
	base.gps.Lg_D 	= Lg_D;
	base.gps.Lt 	= (int)(Lt*10000);
	base.gps.Lt_D 	= Lt_D;
	base.gps.Date 	= (unsigned long long)mktime(&Tm) * 1000;
	base.gps.Speed	= (int)(Speed*100);

	//printf("Lg=%d Lt=%d Lg_D=%c Lt_D=%c Speed=%d Date=%lld\n",
	//base.gps.Lg, base.gps.Lt, base.gps.Lg_D, base.gps.Lt_D, base.gps.Speed, base.gps.Date);

	if(Speed < 40.0)
	{
		bGetGpsData = 1;
	}
	else
	{
		bGetGpsData = 0;
	}

	static int isSetTime = 0;
	if(isSetTime == 0)
	{
		time_t t = mktime(&Tm);
		time_t tN = time(0);
		//satfi_log("sat_csq_ltime=%d t=%d tN=%d %d", base.sat.sat_csq_ltime, t, tN, t - tN);
		if(base.sat.sat_csq_ltime > 0) base.sat.sat_csq_ltime += (t - tN);

		APPSOCKET *pApp = gp_appsocket;
		while(pApp)
		{
			pApp->Update += (t - tN);
			pApp = pApp->next;
		}

		if(t > 0 && SetSystemTime(t) == 0) isSetTime = 1;
	}
	
	return 0;
}

/* 获取北斗GPS模块GPS数据某一项
 * n=0得到GpsData第一项$GPRMC
 * n=1得到GpsData第二项140039.000
 */
int GetGpsData(int n, char *buf)
{
	char *p;
	int i=0;
	char tmp[256];
	strcpy(tmp,GpsData);
	p=strtok(tmp,",");
	
	while(p)
	{
		if(i == n)
		{
			strcpy(buf, p);
			break;
		}
		
		i++;
		p=strtok(NULL,",");
	}
	
	return 0;
}

/* 检测文件是否存在
 *
 */
int isFileExists(const char *path)
{
  return !access(path, F_OK);
}

/* 解析SAT模块+GPSUTC
 *
 */
#define MAX_STR_LEN    32
#define MAX_GPSUTC_IDX 10

typedef enum {
  GPSUTC_Head_IDX = 0,
  GPSUTC_ISDST_IDX = 1,
  GPSUTC_YDAY_IDX = 2,
  GPSUTC_WDAY_IDX = 3,
  GPSUTC_YEAR_IDX = 4,
  GPSUTC_MON_IDX = 5,
  GPSUTC_MDAY_IDX = 6,
  GPSUTC_HOUR_IDX = 7,
  GPSUTC_MIN_IDX = 8,
  GPSUTC_SEC_IDX = 9,
}GPSUTC_Field_Name_Idx;

char ucGpsutcFieldName[MAX_GPSUTC_IDX][MAX_STR_LEN] =
{
  {"InformationHeader"},
  {"ISDST"},
  {"YDAY"},
  {"WDAY"},
  {"YEAR"},
  {"MON"},
  {"MDAY"},
  {"HOUR"},
  {"MIN"},
  {"SEC"},
};

char *gps_find_key(char *buf, char key)
{
  int i,len=strlen(buf);
  char *ret = NULL;
  for(i=0;i<len;i++)
  {
    if(buf[i]==key)
    {
      ret = &buf[i];
      break;
    }
  }
  return ret;
}

int gps_parse(char *buf)
{
  int i;
  char *head;
  char *field1, *field2;

  //satfi_log("parse +GPSUTC ...\n");

  if(strcasestr(buf, "+GPSUTC: "))
  {
    field1 = buf;

    for(i=0;i<MAX_GPSUTC_IDX;i++)
    {
      if(i==0)
      {
        field2 = gps_find_key(field1,':');
      }
      else
      {
        field2 = gps_find_key(field1,',');
      }

      if(field2!=NULL)
      {
        if(field2==field1)
        {
          //satfi_log("%8s ---- null\n", ucGpsutcFieldName[i]);
        }
        else
        {
          *field2 = '\0';
          //satfi_log("%8s ----%s\n", ucGpsutcFieldName[i], field1);
        }
        field1 = field2+1;
      }
      else
      {
        //if(field1!=NULL)
          //satfi_log("%8s ----%s\n", ucGpsutcFieldName[i], field1);
        //else
          //satfi_log("%8s ---- null\n", ucGpsutcFieldName[i]);
      }
    }
  }

  //satfi_log("parse +GPSUTC .\n");
}

/* 解析CSQ
 *
 */
int parsecsq(char *buf, int n)
{
  char *p = gps_find_key(buf,' ');
  char *q = gps_find_key(buf,',');
  if(p && q && q>p)
  {
    *q='\0';
    return atoi(p);
  }
  return -1;
}

/* 解析SAT模块GPS数据
 *
 */
int parsegps(char *buf, int n)
{
  int i=0;
  int rslt = 0;
  char *p=strtok(buf,"\"");
  char *q=NULL;
  char *r=NULL;
  while(p)
  {
    if(q=strstr(p,"$GPRMC"))
    {
      //satfi_log("parsegps");
      strcpy(base.sat.sat_gps, q);
      //satfi_log("SAT GPS ::: %s\n", q);
      //printf("%d %s\n",strlen(base.sat.sat_gps),base.sat.sat_gps);
      sat_unlock();
      break;
    }
    p=strtok(NULL,"\"");
  }

  if(q)
  {
    r = strtok(q,",");
    while(r)
    {
      i++;
      //satfi_log("*** ### %d : %s\n", i, r);
      if(i==3 && r[0]=='A')
        rslt=1;
      r=strtok(NULL,",");
    }
  }

  if(rslt==1)
  {
  }

  return rslt;
}

int IsAllDigit(char *buf, int n)
{
  int i=0;
  for(i=0;i<n;i++)
  {
    if(!isdigit(buf[i])) return 0;
  }
  return 1;
}

//find /var/log/pppsatlog|xargs grep -ri "QoS not accepted"
int isFileHaveString(char *Filename, char *String)
{
	char rbuf[256]={0};
	char buf[256]={0};
	int  maxline = 1;

	sprintf(buf, "find %s|xargs grep -ri \"%s\" -l", Filename, String);
	
	myexec(buf, rbuf, &maxline);
	satfi_log("CheckFileHaveString %s %s %d\n",buf, rbuf, strlen(rbuf));

	if(strlen(rbuf) > 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//0 存在ifname
int checkroute(char *ifname, char *addr, int checkaddr)
{
	int rslt = 0;
	char buf[256]={0};
	char rbuf[256]={0};
	int  maxline = 1;
	if(ifname==NULL||addr==NULL||strlen(ifname)==0||strlen(addr)==0)
	{
		rslt = 1;
	}
	else
	{
		if(checkaddr)
		{
			sprintf(buf, "route | grep %s | grep %s | awk '{ print $NF }'", ifname, addr);
		}
		else
		{
			sprintf(buf, "route | grep %s | awk '{ print $NF }'", ifname);
		}
		myexec(buf, rbuf, &maxline);
		if(strlen(rbuf)==0)
		{
			rslt = 1;
		}
		else
		{
			if(strcmp(rbuf, ifname) != 0)
			{
				rslt = 1;
			}
		}
	}

	return rslt;
}

//北斗模块电源控制
void BdGpsPowerControl(int on)
{
	int fd_power;
	static int state = 0;
	fd_power = open("/dev/power_mode",O_RDWR | O_NONBLOCK);
	if(fd_power < 0)
	{
		return;
	}
	
	if(on == 1)
	{
	  if(state == 0)
	  {
	  	//satfi_log("bd_gps on\n");
		ioctl(fd_power, 6);	//bd_gps on
		state = 1;
	  }
	}
	else if(on == 0)
	{
	  //if(state == 1)
	  {
	  	//satfi_log("bd_gps off\n");
		ioctl(fd_power, 7);	//bd_gps off
		state = 0;
	  }
	}

	close(fd_power);
}

/* 3G模块工作状态
 *
 */
short get_n3g_status()
{
  short status = 0;
  if(base.n3g.n3g_status == 1) status = 0;
  else if(strlen(base.n3g.n3g_imei)==0) status = 2;       //没有读到IMEI
  else if(strlen(base.n3g.n3g_imsi)==0 || base.n3g.n3g_state == N3G_SIM_NOT_INSERTED) status = 3;  //没有读到IMSI
  else if(base.n3g.n3g_dialing == 1 || 
  	base.n3g.n3g_state == N3G_STATE_DIALING) status = 1;     //正在拨号
  else if(base.n3g.n3g_state == N3G_STATE_CSQ ||
          base.n3g.n3g_state == N3G_STATE_CSQ_W)
  {
    status = 4; //信号强度不够
  }
  else status = 2;//加载失败
  return status;
}

/* 卫星模块工作状态
 *
 */
short get_sat_status()
{
  short status = 0;
  if(bSatNetWorkActive == 0) status = 9;				  //冻结
  else if(base.sat.sat_calling == 1) status = 6;     //正在进行电路域的呼叫
  else if(base.sat.sat_status == 1) status = 0;			  //工作正常
  else if(strlen(base.sat.sat_imei)==0) status = 2;       //没有读到IMEI
  else if(strlen(base.sat.sat_imsi)==0 || 
  	base.sat.sat_state == SAT_SIM_NOT_INSERTED) status = 3;  //没有读到IMSI
  else if(base.sat.sat_state == SAT_SIM_ARREARAGE) status = 8;     //欠费
  else if(base.sat.sat_dialing == 1 || 
  	base.sat.sat_state == SAT_STATE_DIALING) status = 1;     //正在拨号
  //else if(base.sat.sat_status == 0) status = 1;      //没有拨号，暂时设置状态：正在拨号中
  else if(base.sat.sat_state == SAT_STATE_GPSTRACK_START ||
          base.sat.sat_state == SAT_STATE_GPSTRACK_START_W ||
          base.sat.sat_state == SAT_STATE_GPSTRACK_STOP ||
          base.sat.sat_state == SAT_STATE_GPSTRACK_STOP_W)
  {
    status = 5; //正在获取定位数据
  }
  else if(base.sat.sat_state == SAT_STATE_CSQ ||
          base.sat.sat_state == SAT_STATE_CSQ_W)
  {
    status = 4; //信号强度不够
  }
  else
  {
	 status = 2;//加载失败
	 //satfi_log("base.sat.sat_state=%d",base.sat.sat_state);
  }
  return status;
}

/* 卫星模块拨打电话状态
 *	0-等待拨号
 *	1-正在通话
 *	2-正在呼叫
 *	3-正在振铃
 *	4-对方无应答
 *	5-接听成功
 *	6-电话已挂断
 *	7-拨号失败
 *	8-有来电
 */
short get_sat_dailstatus()
{
  short status = 0;
  //sat_lock();
  if(base.sat.sat_state_phone == SAT_STATE_PHONE_IDLE) status = 0;
  else if(base.sat.sat_state_phone == SAT_STATE_PHONE_ONLINE) status = 1;
  else if(base.sat.sat_state_phone == SAT_STATE_PHONE_DIALING ||
  	base.sat.sat_state_phone == SAT_STATE_PHONE_CLCC ||
  	base.sat.sat_state_phone == SAT_STATE_PHONE_CLCC_OK ||
  	base.sat.sat_state_phone == SAT_STATE_PHONE_ATD_W) status = 2;
  else if(base.sat.sat_state_phone == SAT_STATE_PHONE_DIALING_RING) status = 3;
  else if(base.sat.sat_state_phone == SAT_STATE_PHONE_NOANSWER) status = 4;
  else if(base.sat.sat_state_phone == SAT_STATE_PHONE_DIALING_SUCCESS) status = 5;
  else if(base.sat.sat_state_phone == SAT_STATE_PHONE_HANGUP || 
  	base.sat.sat_state_phone == SAT_STATE_PHONE_COMING_HANGUP ||
  	base.sat.sat_state_phone == SAT_STATE_PHONE_ATH_W) status = 6;
  else if(base.sat.sat_state_phone == SAT_STATE_PHONE_DIALINGFAILE || 
  	base.sat.sat_state_phone == SAT_STATE_PHONE_DIALING_FAILE_AND_ERROR) status = 7;
  else if(base.sat.sat_state_phone == SAT_STATE_PHONE_RING_COMING) status = 8;
  else status = 9;
  //sat_unlock();
  return status;
}

/* 3G模块线程
 *
 */
static void *func_z(void *p)
{
	BASE *base = (BASE *)p;
	int counter = 0;
	int dialingcount = 0;

	int checkaddr = 0;
	int isttyGPRS0Bad = 0;
	int isttyGPRS1Bad = 0;

	while(1)
	{
		n3g_lock();
		//判断3g module是否上电正常
		if (!isFileExists(base->n3g.n3g_dev_name))
		{
			if(base->n3g.n3g_dialing)
			{
				char ucbuf[256];
				base->n3g.n3g_dialing = 0;
				sprintf(ucbuf, "ifdown %s", base->n3g.n3g_ifname_a);
				satfi_log("%s %d\n",ucbuf, __LINE__);
				myexec(ucbuf, NULL, NULL);
			}
			n3g_unlock();
			seconds_sleep(30);
			continue;
		}

		if (detect_interface(base->n3g.n3g_ifname) == 0)
		{
			uart_send(base->n3g.n3g_fd, "AT+CSQ\r\n", 8);
			if(checkroute(base->n3g.n3g_ifname,base->tsc.tsc_addr, checkaddr) == 0)
			{
				if(checkaddr == 0 && strlen(base->tsc.tsc_addr)>0)
				{
					char buf[256];
					sprintf(buf, "route add %s dev %s", base->tsc.tsc_addr, base->n3g.n3g_ifname);
					satfi_log("%s %d\n",buf, __LINE__);
					myexec(buf, NULL, NULL);
					checkaddr = 1;
				}
				else
				{
					if(base->n3g.n3g_status == 0)
					{
						base->n3g.n3g_status = 1;
						base->n3g.n3g_state = N3G_STATE_IDLE;
						base->n3g.n3g_dialing = 0;
					}
				}
			}
			else
			{
				satfi_log("base->n3g.n3g_status=%d %d\n", base->n3g.n3g_status, __LINE__);
				checkaddr = 0;
				base->n3g.n3g_status = 0;
			}

			n3g_unlock();
			seconds_sleep(10);
			continue;
		}

		base->n3g.n3g_status = 0;
		if(base->n3g.n3g_dialing)
		{
			//信号弱时将链路断开
			if(base->n3g.n3g_state == N3G_STATE_CSQ)
			{
				char ucbuf[256];
				base->n3g.n3g_dialing = 0;
				sprintf(ucbuf, "ifdown %s", base->n3g.n3g_ifname_a);
				satfi_log("%s %d\n",ucbuf, __LINE__);
				myexec(ucbuf, NULL, NULL);
			}

			uart_send(base->n3g.n3g_fd, "AT+CSQ\r\n", 8);
			n3g_unlock();

			if(++dialingcount>=30)
			{
				myexec("ifdown wan0", NULL, NULL);
				dialingcount = 0;
				continue;
			}
			else
			{
				n3g_unlock();
				satfi_log("n3g module is trying to connect to GPRS\n");
				seconds_sleep(10);
			}
			continue;
		}

		if(base->n3g.n3g_fd == -1)
		{
			n3g_unlock();	
			satfi_log("func_z:init_serial(%s,%d)\n", base->n3g.n3g_dev_name, base->n3g.n3g_baud_rate);
			time_t start = time(0);
			init_serial(&base->n3g.n3g_fd, base->n3g.n3g_dev_name, base->n3g.n3g_baud_rate);
			time_t end = time(0);
			satfi_log("start=%d end=%d\n", start, end);
			if(end - start > 5)
			{
				close(base->n3g.n3g_fd);
				base->n3g.n3g_fd -1;
				myexec("power_mode gprs on", NULL, NULL);
			}
			
			if(base->n3g.n3g_fd>=0)
			{
				base->n3g.n3g_state = N3G_STATE_AT;
			}
			else
			{
				n3g_unlock();
				continue;
			}
		}

		//satfi_log("3g module state = %d\n", base->n3g.n3g_state);
		switch(base->n3g.n3g_state)
		{
			case N3G_STATE_AT:
				//printf("func_z:send AT to module\n");
				uart_send(base->n3g.n3g_fd, "AT\r\n", 4);
				base->n3g.n3g_state = N3G_STATE_AT_W;
				counter=0;
			break;
			case N3G_STATE_AT_W:
				//printf("func_z:send AT to module\n");
				uart_send(base->n3g.n3g_fd, "AT\r\n", 4);
				base->n3g.n3g_state = N3G_STATE_AT_W;
				//counter=0;
			break;
			case N3G_STATE_IMEI:
				//printf("func_z:send AT+CGSN to module\n");
				uart_send(base->n3g.n3g_fd, "AT+CGSN\r\n", 9);
				base->n3g.n3g_state = N3G_STATE_IMEI_W;
				counter=0;
			break;
			case N3G_STATE_IMSI:
				//printf("func_z:send AT+CIMI to module\n");
				uart_send(base->n3g.n3g_fd, "AT+CIMI\r\n", 9);
				base->n3g.n3g_state = N3G_STATE_IMSI_W;
				counter=0;
			break;
			case N3G_STATE_CSQ:
				//printf("func_z:send AT+CSQ to module\n");
				uart_send(base->n3g.n3g_fd, "AT+CSQ\r\n", 8);
				base->n3g.n3g_state = N3G_STATE_CSQ_W;
				counter=0;
			break;
			case N3G_STATE_DIALING:
				satfi_log("N3G_STATE_DIALING");
				base->n3g.n3g_state = N3G_STATE_IDLE;
				base->n3g.n3g_dialing = 1;
				dialingcount = 0;
				{
					char ucbuf[256];
					sprintf(ucbuf, "ifup %s", base->n3g.n3g_ifname_a);
					myexec(ucbuf, NULL, NULL);
				}
				counter=0;
				break;
				case N3G_SIM_NOT_INSERTED:
				counter=0;
			break;
		}

		if(++counter>=10)
		{
			if(base->n3g.n3g_state == N3G_STATE_AT_W)
			{
				if(strcmp(base->n3g.n3g_dev_name, "/dev/ttyGPRS0") == 0)
				{
					bzero(base->n3g.n3g_dev_name,sizeof(base->n3g.n3g_dev_name));
					strcpy(base->n3g.n3g_dev_name, "/dev/ttyGPRS1");
					isttyGPRS0Bad = 1;
				}
				else if(strcmp(base->n3g.n3g_dev_name, "/dev/ttyGPRS1") == 0)
				{
					bzero(base->n3g.n3g_dev_name,sizeof(base->n3g.n3g_dev_name));
					strcpy(base->n3g.n3g_dev_name, "/dev/ttyGPRS0");
					isttyGPRS1Bad = 1;
				}

				if(isttyGPRS0Bad && isttyGPRS1Bad)
				{
					satfi_log("isttyGPRS0Bad=%d isttyGPRS1Bad=%d",isttyGPRS0Bad, isttyGPRS1Bad);
					myexec("power_mode gprs on", NULL, NULL);
					isttyGPRS0Bad = 0;
					isttyGPRS1Bad = 0;
				}
			}

			if(base->n3g.n3g_fd>=0)
			{
				close(base->n3g.n3g_fd);
				base->n3g.n3g_fd = -1;
			}
			
			base->n3g.n3g_state = N3G_STATE_IDLE;
			n3g_unlock();
			continue;
		}

		n3g_unlock();
		seconds_sleep(2);
	}
}


/* 卫星模块线程
 *
 */
static void *func_y(void *p)
{
	BASE *base = (BASE*)p;
	int counter = 0;

//#define SAT_MESSAGE_DEV	"/dev/ttyS0"

	while(1)
	{
		//sat_lock();
		if (!isFileExists(base->sat.sat_dev_name) || base->sat.sat_state == SAT_STATE_RESTART)
		{
			if(!isFileExists(base->sat.sat_dev_name))satfi_log("no exist %s", base->sat.sat_dev_name);
		
			if(base->sat.sat_fd > 0)
			{
				close(base->sat.sat_fd);
				base->sat.sat_fd = -1;
			}
			
			satfi_log("power_mode msm01a reset %d %d\n",__LINE__, base->sat.sat_state);
			base->sat.sat_state = SAT_STATE_RESTART_W;
			base->sat.sat_status = 0;
			base->sat.sat_msg_sending = 0;
			myexec("power_mode msm01a reset", NULL, NULL);
			seconds_sleep(30);
			continue;
		}
		else
		{
			if(base->sat.sat_fd < 0)
			{
				init_serial(&base->sat.sat_fd, base->sat.sat_dev_name, base->sat.sat_baud_rate);
				base->sat.sat_state = SAT_STATE_AT;
				base->sat.sat_fd_message = base->sat.sat_fd;
			}
		}		
		
		if(base->sat.sat_fd > 0 && base->sat.sat_calling == 0 && base->sat.sat_msg_sending == 0)
		{
			//satfi_log("sat module state = %d\n", base->sat.sat_state);
			switch(base->sat.sat_state)
			{
				case SAT_STATE_AT:
					satfi_log("func_y:send AT to SAT Module\n");
					uart_send(base->sat.sat_fd, "AT\r\n", 4);
					base->sat.sat_state = SAT_STATE_AT_W;
					counter=0;
					break;
				case SAT_STATE_AT_W:
					satfi_log("func_y:send AT to SAT Module %d\n", counter);
					uart_send(base->sat.sat_fd, "AT\r\n", 4);
					base->sat.sat_state = SAT_STATE_AT_W;
					counter++;
					if(counter >= 20)
					{
						satfi_log("SAT_STATE_RESTART %d\n", __LINE__);
						base->sat.sat_state = SAT_STATE_RESTART;
						counter=0;
					}
					break;
				case SAT_STATE_SIM_ACTIVE:
				case SAT_STATE_SIM_ACTIVE_W:
					satfi_log("func_y:send AT+CFUN=1 to SAT Module\n");
					uart_send(base->sat.sat_fd, "AT+CFUN=1\r\n", 11);
					base->sat.sat_state = SAT_STATE_SIM_ACTIVE_W;
					break;
				case SAT_STATE_CFUN:
					satfi_log("func_y:send AT+CFUN? to SAT Module\n");
					uart_send(base->sat.sat_fd, "AT+CFUN?\r\n", 10);
					base->sat.sat_state = SAT_STATE_CFUN_W;
					break;
				case SAT_STATE_IMSI:
				case SAT_STATE_IMSI_W:
					satfi_log("func_y:send AT+CIMI to SAT Module\n");
					uart_send(base->sat.sat_fd, "AT+CIMI\r\n", 9);
					base->sat.sat_state = SAT_STATE_IMSI_W;
					break;
				case SAT_STATE_FULL_FUN:
					satfi_log("func_y:send AT+CFUN=1 to SAT Module\n");
					uart_send(base->sat.sat_fd, "AT+CFUN=1\r\n", 11);
					base->sat.sat_state = SAT_STATE_FULL_FUN_W;
					seconds_sleep(20);
					break;
				case SAT_STATE_FULL_FUN_W:
					base->sat.sat_state = SAT_STATE_IMSI;
					break;
				case SAT_STATE_CREG:
				case SAT_STATE_CREG_W:
					//satfi_log("func_y:send AT+CREG? to SAT Module\n");
					uart_send(base->sat.sat_fd, "AT+CREG?\r\n", 10);
					base->sat.sat_state = SAT_STATE_CREG_W;
					break;
				case SAT_STATE_CSQ:
				case SAT_STATE_CSQ_W:
					//satfi_log("func_y:send AT+CSQ to SAT Module\n");
					uart_send(base->sat.sat_fd, "AT+CSQ\r\n", 8);
					base->sat.sat_state = SAT_STATE_CSQ_W;
					break;
				case SAT_STATE_DIALING:
					satfi_log("func_y:SAT Module Trying to connect to GmPRS\n");
					//base->sat.sat_state = SAT_STATE_IDLE;
					//base->sat.sat_dialing = 1;
					//net_lock();
					//char ucbuf[256];
					//bzero(ucbuf,sizeof(ucbuf));
					//sprintf(ucbuf, "ifup %s", base->sat.sat_ifname_a);
					//satfi_log("%s %d\n",ucbuf,__LINE__);
					//myexec(ucbuf, NULL, NULL);
					//net_unlock();
					//counter=0;
					break;
				case SAT_SIM_NOT_INSERTED:
				case SAT_SIM_ARREARAGE:
					break;
				case SAT_STATE_REGFAIL:
					base->sat.sat_state = SAT_SIM_ARREARAGE;
					satfi_log("SAT_STATE_REGFAIL=%d\n", base->sat.sat_state);
					break;
			}
		}
		
		//sat_unlock();
		if(base->sat.sat_status == 1)
		{
			seconds_sleep(5);
		}
		else
		{
			seconds_sleep(1);
		}
	}
}

void handle_gprs_data(int n3gfd)
{
	static char data[1024] = {0};
	static int idx=0,ncnt,flg=0,p1=0,p2=0;
	int n;

	while(1)
	{
		if(idx == 0)
		{
			memset(data, 0, 1024);
			ncnt=0;
		}
		
		n=read(n3gfd, &data[idx], 1);
		if(n>0)
		{
			if(data[idx] == '\r') data[idx] = '\n';
			if(data[idx] == '\n') ncnt++;
			else ncnt = 0;
			if(ncnt==4){idx=2;ncnt=0;continue;}
			idx++;
			if(idx==1 && data[0]!='\n'){idx=0;ncnt=0;continue;}
			else if(idx==2 && data[1]!='\n'){idx=0;ncnt=0;continue;}
			else if(idx==3 && data[2]=='\n'){idx=2;ncnt=2;continue;}
			else if(idx>4)
			{
				if(strstr(data, "+CGSN"))
				{
					if(data[idx-1] == '\"')
					{
						flg++;
						if(flg==1) p1=idx;
						else if(flg==2) p2=idx-1;
					}
					if(data[idx-1]=='\n' && data[idx-2]=='\n')
					{
						n3g_lock();
						//if(p2>p1 && p2-p1==15)
						{
							//strncpy(base.n3g.n3g_imei,&data[p1],IMSI_LEN);
							if(data[9] != '"')
							{
								strncpy(base.n3g.n3g_imei,&data[8],IMSI_LEN);
								satfi_log("3g_imei1:%s\n", base.n3g.n3g_imei);
								//SetKeyString("n3g", "imei", "/tmp/config.ini",NULL,base.n3g.n3g_imei);
								if(base.n3g.n3g_state == N3G_STATE_IMEI_W)
								{
									base.n3g.n3g_state = N3G_STATE_IMSI;
								}
							}
							else
							{
								strncpy(base.n3g.n3g_imei,&data[10],IMSI_LEN);
								satfi_log("3g_imei1:%s\n", base.n3g.n3g_imei);
								//SetKeyString("n3g", "imei", "/tmp/config.ini",NULL,base.n3g.n3g_imei);
								if(base.n3g.n3g_state == N3G_STATE_IMEI_W)
								{
									base.n3g.n3g_state = N3G_STATE_IMSI;
								}
							}
						}
						n3g_unlock();
						idx=0;ncnt=0;flg=0;p1=0;p2=0;
					}

				}
				else if(strstr(data, "+CSQ") && data[idx-1]=='\n' && data[idx-2]=='\n')
				{
					n3g_lock();
					base.n3g.n3g_csq_value = parsecsq(data,idx);
					//satfi_log("n3g_csq_value = %d\n", base.n3g.n3g_csq_value);
					base.n3g.n3g_csq_ltime = time(0);
					if(base.n3g.n3g_csq_value != 99 && base.n3g.n3g_csq_value>=base.n3g.n3g_csq_limit_value)
					{
						if(base.n3g.n3g_state == N3G_STATE_CSQ_W)
						{
							base.n3g.n3g_state = N3G_STATE_DIALING;
						}
					}
					else
					{
						if(base.n3g.n3g_state == N3G_STATE_CSQ_W)
						{
							base.n3g.n3g_state = N3G_STATE_CSQ;
						}
						
						if(base.n3g.n3g_csq_value == 99) base.n3g.n3g_csq_value = 0;
					}
					n3g_unlock();
					idx=0;ncnt=0;flg=0;p1=0;p2=0;
				}
				else if(strstr(data, "\n\nOK\n\n"))
				{
					n3g_lock();
					if(base.n3g.n3g_state==N3G_STATE_AT_W)
					{
						if(strlen(base.n3g.n3g_imei)==0)
						{
							base.n3g.n3g_state = N3G_STATE_IMEI;
						}
						else if(strlen(base.n3g.n3g_imsi)==0)
						{
							base.n3g.n3g_state = N3G_STATE_IMSI;
						}
						else
						{
							base.n3g.n3g_state = N3G_STATE_CSQ;
						}
					}
					n3g_unlock();
					idx=0;ncnt=0;flg=0;p1=0;p2=0;
				}
				else if(strstr(data, "\n\nERROR\n\n"))
				{
					idx=0;ncnt=0;flg=0;p1=0;p2=0;
					if(base.n3g.n3g_state == N3G_STATE_IMSI_W)
					{
						base.n3g.n3g_state = N3G_SIM_NOT_INSERTED;
					}
				}
				else if(strstr(data,"\n\nNO CARRIER\n\n"))
				{
					idx=0;ncnt=0;flg=0;p1=0;p2=0;
				}
				else if(data[idx-1]=='\n' && data[idx-2]=='\n')
				{
					if(strstr(data, "\n\n+"))
					{
						data[idx-2] = '\0';
						//satfi_log("func_zz: **>>**>> %s\n", &data[2]);
					}
					else if(idx==19 && IsAllDigit(&data[2], 15))
					{
						n3g_lock();
						if(base.n3g.n3g_state == N3G_STATE_IMSI_W)
						{
							strncpy(base.n3g.n3g_imsi, &data[2], 15);
							base.n3g.n3g_state = N3G_STATE_CSQ;
							satfi_log("3g_imsi:%s\n", base.n3g.n3g_imsi);
							//SetKeyString("n3g", "imsi", "/tmp/config.ini",NULL,base.n3g.n3g_imsi);
						}
						if(base.n3g.n3g_state == N3G_STATE_IMEI_W)
						{
							strncpy(base.n3g.n3g_imei, &data[2], 15);
							base.n3g.n3g_state = N3G_STATE_IMSI;
							satfi_log("3g_imei2:%s\n", base.n3g.n3g_imei);
							//SetKeyString("n3g", "imei", "/tmp/config.ini",NULL,base.n3g.n3g_imei);
						}
						n3g_unlock();
					}
					idx=0;ncnt=0;flg=0;p1=0;p2=0;
				}
			}
		}
		else
		{
			break;
		}
	}

}

int handle_sat_data(int *satfd, char *data, int *ofs)
{
	int idx=*ofs;
	int n;
	while(1)
	{
		if(idx==0)
		{
			memset(data,0,1024);
		}
		n = read(*satfd, &data[idx], 1);
		if(n>0)
		{			
			if(data[idx]=='\r') data[idx]='\n';
			idx++;
			if(idx==1 && data[0] != '\n')
			{
				idx = 0;
				continue;
			}
			else if(idx==2 && data[1] != '\n')
			{
				idx = 0;
				continue;
			}
			else if(idx==3 && data[2] == '\n')
			{
				idx = 2;
				continue;
			}
			else if(idx==4)
			{
				if(data[idx-2] == '>' && data[idx-1] == ' ')	//<GRT><SP>
				{
					//satfi_log("%s\n", data);
					MessageSend(*satfd);
					idx=0;
				}
			}
			else if(idx>4)
			{
				static int clcccnt=0;
				static int clcccntpre=0;
				
				if(strstr(data,"+CFUN"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						if(data[9] == '0')
						{
							if(base.sat.sat_state==SAT_STATE_CFUN_W)
							{
								base.sat.sat_state=SAT_STATE_SIM_ACTIVE;
							}
						}
						else if(data[9] == '4')
						{
							satfi_log("%s %d\n",data, __LINE__);
							base.sat.sat_state=SAT_STATE_RESTART;
						}
						else 
						{
							if(base.sat.sat_state==SAT_STATE_CFUN_W)
							{
								base.sat.sat_state=SAT_STATE_IMSI;
							}						
						}
					}
				}
				else if(strstr(data,"+RCIPH"))
				{
					if(base.sat.sat_state_phone == SAT_STATE_PHONE_ATD_W)
					{
						satfi_log("+RCIPH = %d\n", base.sat.sat_state_phone);
						base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING_CLCC;
					}
				}
				else if(strstr(data,"+CMT"))
				{
					if(idx>18 && data[idx-2]=='\n' && data[idx-1]=='\n')
					{					
						satfi_log("%d %d %d %s",strlen(data), base.sat.sat_fd_message, idx, data);
						unsigned char Decode[1024] = {0};
						char OAphonenum[21] = {0};	//发送方地址（手机号码）
						int i=0;
						
						for(i=2;i<strlen(data);i++)
						{
							if(data[i] == '\n' && data[i+1] == '\n')
							{
								break;
							}
						}
						
						int len = MessageParse(&data[i+2], OAphonenum, Decode);
						satfi_log("MessageParse %s %s %d %d %d", OAphonenum, Decode, *satfd, i, strlen(data));
						char tmp[1024]={0};
						MsgGetMessageRsp *rsp = (MsgGetMessageRsp *)tmp;
						rsp->header.length = sizeof(MsgGetMessageRsp)+strlen(Decode)+1;
						rsp->header.mclass = RECV_MESSAGE;
						//strncpy(rsp1->MsID, req->MsID, 21);
						rsp->Result = 1;
						strncpy(rsp->SrcMsID, OAphonenum, 21);
						rsp->Type = 3;
						rsp->ID = time(0);
						unsigned long long t = (unsigned long long)rsp->ID*1000;
						memcpy(rsp->Date, &t, sizeof(rsp->Date));
						memcpy(rsp->message, Decode, strlen(Decode)+1);


						USER * toUser = gp_users;
						int toUserSocket=-1;
						while(toUser)
						{
							for(i=0; i<toUser->famNumConut && i<10; i++)
							{
								if(strstr(toUser->FamiliarityNumber[i], OAphonenum))
								{
									satfi_log("%s OAphonenum=%s %.21s", toUser->FamiliarityNumber[i], OAphonenum, toUser->userid);
									toUserSocket = toUser->socketfd;
									break;
								}
							}
						
							if(toUserSocket>0)
							{
								break;
							}
							
							toUser=toUser->next;
						}

						if(toUserSocket == -1)
						{
							Data_To_ExceptMsID("TOALLMSID", rsp);
						}
						else
						{
							strncpy(rsp->MsID, toUser->userid, USERID_LLEN);
							Data_To_MsID(toUser->userid, rsp);
						}
						
						idx = 0;
					}
				}								
				else if(strstr(data,"+CSQ"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						static int sat_csq_bad = 0;
						sat_lock();
						base.sat.sat_csq_value = parsecsq(data, idx);
						base.sat.sat_csq_ltime = time(0);
						//satfi_log("sat_csq_value = %d\n", base.sat.sat_csq_value);
						if(base.sat.sat_csq_value == 31 || base.sat.sat_csq_value == 99) base.sat.sat_csq_value = 0;

						if(base.sat.sat_csq_value>=base.sat.sat_csq_limit_value)
						{
							//satfi_log("sat_csq_good_cnt %d %d\n", sat_csq_good_cnt, base.sat.sat_csq_value);
							NotifyAllUserSatState(SAT_LINK_NORMAL);
						}
						else
						{							
							NotifyAllUserSatState(SAT_LINK_DISCONNECT);
						}

						if(base.sat.sat_state==SAT_STATE_CSQ_W)
						{
							base.sat.sat_state = SAT_STATE_CREG;
						}
						
						sat_unlock();
						idx=0;
					}
				}				
				else if(strstr(data,"+CREG"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						//satfi_log("%s\n",data);
						if(base.sat.sat_state == SAT_STATE_CREG_W)
						{
							if(data[11] == '0')
							{
								satfi_log("%s %d\n",data, __LINE__);
								base.sat.sat_state = SAT_STATE_FULL_FUN;
								base.sat.sat_status = 0;
							}
							else if(data[11] == '1' || data[11] == '5')
							{
								base.sat.sat_state = SAT_STATE_CSQ;
								base.sat.sat_status = 1;
							}
							else if(data[11] == '2')
							{
								if(base.sat.sat_status == 1)
								{
									satfi_log("%s %d\n",data, __LINE__);
									//base.sat.sat_state = SAT_STATE_RESTART;
									base.sat.sat_state = SAT_STATE_CSQ;
								}
								else
								{
									base.sat.sat_state = SAT_STATE_CSQ;
									base.sat.sat_status = 0;
								}
							}
							else if(data[11] == '3')
							{
								satfi_log("%s %d\n",data, __LINE__);
								base.sat.sat_state = SAT_STATE_REGFAIL;//??????
								base.sat.sat_status = 0;
							}
							else if(data[11] == '4')
							{
								satfi_log("%s %d\n",data, __LINE__);
								base.sat.sat_state = SAT_STATE_RESTART;
								base.sat.sat_status = 0;
							}
							else
							{
								satfi_log("%s %d\n",data, __LINE__);
								base.sat.sat_state = SAT_STATE_CREG;
								//base.sat.sat_status = 0;								
							}
						}
						
						idx=0;
					}
				}
				else if(strstr(data,"RING"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						satfi_log("%s %d\n",data,__LINE__);
						base.sat.sat_state_phone = SAT_STATE_PHONE_RING_COMING;
						idx=0;
					}
				}
				else if(strstr(data,"+CMS ERROR"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						satfi_log("%d,%d,%s:%s",base.sat.sat_calling, *satfd, __FUNCTION__, data);

						if(MessagesHead != NULL)
						{
							MsgSendMsgRsp rsp;
							rsp.header.length = sizeof(MsgSendMsgRsp);
							rsp.header.mclass = SEND_MESSAGE_RESP;
							strncpy(rsp.MsID, MessagesHead->MsID, 21);
							rsp.Result = 1;//短信发送失败
							rsp.ID = MessagesHead->ID;
							Data_To_MsID(rsp.MsID, &rsp);
							MessageDel();
						}
						
						base.sat.sat_msg_sending = 0;
						idx=0;
					}
				}
				else if(strstr(data,"+CME ERROR"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						satfi_log("%s %d\n",data, __LINE__);
						//printf("%s\n",data);
						sat_lock();
						if(strstr(data,"SIM not inserted"))
						{
							base.sat.sat_state = SAT_SIM_NOT_INSERTED;
						}
						else
						{
							if(base.sat.sat_state == SAT_STATE_IMSI_W)
							{
								base.sat.sat_state = SAT_STATE_FULL_FUN;
							}
						}

						if(base.sat.sat_calling == 1)
						{
							base.sat.sat_state_phone = SAT_STATE_PHONE_DIALINGFAILE;
						}

						sat_unlock();	
						idx=0;
					}
				}
				else if(strstr(data,"+CLCC"))
				{
					//satfi_log("%s", data);
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						sat_lock();
						if(base.sat.sat_state_phone == SAT_STATE_PHONE_RING_COMING)
						{
							clcccnt++;
						}
						
						if(data[13] == '0')//通话中
						{
							//base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING_SUCCESS;
							base.sat.sat_state_phone = SAT_STATE_PHONE_ONLINE;
						}
						else if(data[13] == '2')//拨号中
						{
							if(base.sat.sat_state_phone != SAT_STATE_PHONE_DIALING_ATH_W)base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING;
						}
						else if(data[13] == '3')//振铃中
						{
							if(base.sat.sat_state_phone != SAT_STATE_PHONE_DIALING_ATH_W)base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING_RING;
						}
						else if(data[13] == '4')//有来电,提取来电电话号码
						{
							//satfi_log("%s", data);
							bzero(base.sat.called_number,15);
							//base.sat.sat_state_phone = SAT_STATE_PHONE_RING_COMING;
							
							int i=0;
							for(i;i<15;i++)
							{
								if(data[i+20] == '"')
								{
									break;
								}
								
								base.sat.called_number[i] = data[i+20];
							}
							satfi_log("+CLCC called_number=%s %d\n", base.sat.called_number, strlen(base.sat.called_number));
						}
						sat_unlock();

						idx=0;
					}
				}				
				else if(strstr(data,"+CIMI"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						satfi_log("%s %d", data, __LINE__);
						strncpy(base.sat.sat_imsi, &data[9], 15);
						if(base.sat.sat_state==SAT_STATE_IMSI_W)
						{
							base.sat.sat_state = SAT_STATE_CREG;
						}
						idx=0;
					}
				}
				else if(strstr(data,"\n\nOK\n\n"))
				{
					//satfi_log("%s %d", data, __LINE__);
					sat_lock();
					if(base.sat.sat_state_phone==SAT_STATE_PHONE_CLCC) 
					{
						base.sat.sat_state_phone = SAT_STATE_PHONE_CLCC_OK;//可进行拨号
					}
					else if(base.sat.sat_state_phone==SAT_STATE_PHONE_ATH_W) 
					{
						satfi_log("SAT_STATE_PHONE_ATH_W SAT_STATE_PHONE_HANGUP\n");
						base.sat.sat_state_phone = SAT_STATE_PHONE_HANGUP;//已挂断
						clcccnt = 0;
						clcccntpre = 0;
					}
					else if(base.sat.sat_state_phone==SAT_STATE_PHONE_DIALING_ATH_W) 
					{
						satfi_log("SAT_STATE_PHONE_DIALING_ATH_W\n");
						base.sat.sat_state_phone = SAT_STATE_PHONE_DIALINGFAILE;
						clcccnt = 0;
						clcccntpre = 0;
					}
					else if(base.sat.sat_state_phone==SAT_STATE_PHONE_ATA_W) 
					{
						base.sat.sat_state_phone = SAT_STATE_PHONE_ONLINE;//已接通
						clcccnt = 0;
						clcccntpre = 0;
					}
					else if(base.sat.sat_state_phone==SAT_STATE_PHONE_CLIP) 
					{
						base.sat.sat_state_phone = SAT_STATE_PHONE_CLIP_OK;
					}
					else if(base.sat.sat_state_phone==SAT_STATE_PHONE_RING_COMING) 
					{
						if(base.sat.sat_fd == *satfd)
						{
							//satfi_log("clcccnt %d %d %d", clcccnt, clcccntpre, base.sat.sat_state_phone);
							//if(clcccntpre >= 1 && clcccntpre == clcccnt)
							//{
							//		base.sat.sat_state_phone = SAT_STATE_PHONE_COMING_HANGUP;
							//		clcccnt=0;
							//}
							//clcccntpre = clcccnt;
						}
					}
					else if(base.sat.sat_state==SAT_STATE_AT_W)
					{
						base.sat.sat_state = SAT_STATE_IMSI;
					}
					else if(base.sat.sat_state==SAT_STATE_SIM_ACTIVE_W)
					{
						base.sat.sat_state = SAT_STATE_IMSI;
					}
					else 
					{
						//printf("none %d\n",base->sat.sat_state_phone);
						//base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
					}
					sat_unlock();
					idx=0;
				}
				else if(strstr(data,"\n\nERROR\n\n"))
				{
					satfi_log("%s %d\n",data, __LINE__);
					idx=0;

					if(base.sat.sat_calling == 1)
					{
						base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING_ERROR;
					}

					if(base.sat.sat_state == SAT_STATE_IMSI_W)
					{
						base.sat.sat_state = SAT_SIM_NOT_INSERTED;
					}
					
					if(base.sat.sat_state == SAT_STATE_FULL_FUN_W)
					{
						base.sat.sat_state = SAT_STATE_RESTART;
					}
						
				}
				else if(strstr(data,"\n\nNO CARRIER\n\n"))
				{
					satfi_log("%s %d\n",data, __LINE__);
					clcccnt = 0;
					clcccntpre = 0;
					//satfi_log("NO CARRIER %d\n",base.sat.sat_state_phone);
					//printf("NO CARRIER %d\n",base.sat.sat_state_phone);
					sat_lock();
					if(base.sat.sat_calling == 1)
					{
						if(base.sat.sat_state_phone == SAT_STATE_PHONE_ONLINE)
						{
							base.sat.EndTime = time(0);
							base.sat.sat_state_phone = SAT_STATE_PHONE_HANGUP;
						}
						else if(base.sat.sat_state_phone == SAT_STATE_PHONE_RING_COMING)
						{
							base.sat.sat_state_phone = SAT_STATE_PHONE_COMING_HANGUP;
						}
						else if(base.sat.sat_state_phone == SAT_STATE_PHONE_DIALING_RING)
						{
							base.sat.sat_state_phone = SAT_STATE_PHONE_NOANSWER;
						}
						else if(base.sat.sat_state_phone == SAT_STATE_PHONE_DIALING)
						{
							base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING_FAILE_AND_ERROR;
						}
						else if(base.sat.sat_state_phone == SAT_STATE_PHONE_DIALING_ERROR)
						{
							base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING_FAILE_AND_ERROR;
						}
						else
						{
							base.sat.sat_state_phone = SAT_STATE_PHONE_DIALINGFAILE;
						}
					}
					sat_unlock();
					idx=0;
				}
				else if(strstr(data,"\n\nNO ANSWER\n\n"))
				{
					satfi_log("NO ANSWER %d\n",base.sat.sat_state_phone);
					//printf("NO ANSWER %d\n",base.sat.sat_state_phone);
					sat_lock();
					if(base.sat.sat_state_phone == SAT_STATE_PHONE_DIALING_RING)
					{
						base.sat.sat_state_phone = SAT_STATE_PHONE_NOANSWER;
					}
					else
					{
						base.sat.sat_state_phone = SAT_STATE_PHONE_DIALINGFAILE;
					}
					sat_unlock();
					idx=0;
				}
				else if(strstr(data,"+CMGS"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						if(base.sat.sat_msg_sending)
						{
							if(strlen(MessagesHead->MsID) != 0)
							{
								satfi_log("MessagesHead->MsID=%.21s", MessagesHead->MsID);
								Msg_Message_Money_Req req;
								bzero(&req, sizeof(req));
								req.header.length = sizeof(Msg_Message_Money_Req);
								req.header.mclass = APP_MESSAGE_MONEY_CMD;
								StrToBcd(req.MsID, &(MessagesHead->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
								memcpy(req.MsPhoneNum, req.MsID, sizeof(req.MsPhoneNum));
								StrToBcd(req.DesPhoneNum, MessagesHead->toPhoneNum, 11);
								req.StartTime = (unsigned long long)time(0) * 1000;
								req.Money = 40;

								SaveDataToFile(CALL_RECORDS_FILE ,(char *)&req, req.header.length);
							} 

							MsgSendMsgRsp rsp;
							rsp.header.length = sizeof(MsgSendMsgRsp);
							rsp.header.mclass = SEND_MESSAGE_RESP;
							strncpy(rsp.MsID, MessagesHead->MsID, 21);
							rsp.Result = 0;//短信发送成功
							rsp.ID = MessagesHead->ID;
							Data_To_MsID(rsp.MsID, &rsp);
							satfi_log("message send success %d", rsp.ID);
							MessageDel();
							base.sat.sat_msg_sending = 0;
						}							  
						idx=0;
					}
				}					
				else if(data[idx-2]=='\n' && data[idx-1]=='\n')
				{
					//satfi_log("%s", data);
					if(strstr(data, "\n\n+"))
					{
						//过滤其他指令
						data[idx-2] = '\0';
						//satfi_log("func_x: **>>**>> %s\n", &data[2]);
					}
					else if(idx==19 && IsAllDigit(&data[2],15)) //检查是否15位数字
					{
						sat_lock();
						if(base.sat.sat_state == SAT_STATE_IMEI_W)
						{
							base.sat.sat_state = SAT_STATE_IMSI;
							//strncpy(base.sat.sat_imei, &data[2], 15);
							//satfi_log("sat_imei:%s\n", base.sat.sat_imei);
						}
						else if(base.sat.sat_state == SAT_STATE_IMSI_W)
						{
							strncpy(base.sat.sat_imsi, &data[2], 15);
							satfi_log("sat_imsi:%s\n", base.sat.sat_imsi);
							//base.sat.sat_state = SAT_STATE_GPSTRACK_START;	
							base.sat.sat_state = SAT_STATE_CSQ;
						}
						sat_unlock();
					}
					else if(strstr(data, "CEND"))
					{
						satfi_log("%s sat_calling=%d", data, base.sat.sat_calling);
						if(base.sat.sat_calling)
						{
							base.sat.sat_state_phone = SAT_STATE_PHONE_HANGUP;
						}
						
					}
					else if(strstr(data, "VOICEFORMAT: 3,0"))
					{
						satfi_log("%s sat_calling=%d", data, base.sat.sat_calling);
						if(base.sat.sat_calling == 0)
						{
							base.sat.sat_state_phone = SAT_STATE_PHONE_RING_COMING;
						}
					}
					else
					{
						satfi_log("not parse sat data %d:%d:%s",base.sat.sat_fd_message, *satfd, data);						
					}
					idx=0;
				}
			}
		}
		else
		{
			*ofs = idx;
			break;
		}
	}	

}

void PrintSocketBufSize(int socket)
{
	int size;
	ioctl(socket, SIOCOUTQ, &size);//获取tcp未发送缓冲区数据大小
	satfi_log("PrintSocketBufSize=%d\n", size);
}

static int sendConnect(void)
{
	char buf[1024] = {0};
	MsgHeader *p = (MsgHeader *)buf;
	p->length = sizeof(MsgHeader) + IMSI_LEN + sizeof(version_num);
	p->mclass = SATFI_CONNECT_CMD;
	strncpy(&buf[4], base.sat.sat_imei, IMSI_LEN);
	memcpy(&buf[4 + 15], &version_num, 4);

	satfi_log("Send Connect Message to TSC Server version_num=%d sat_imei=%s\n", version_num, base.sat.sat_imei);

	if(strlen(base.sat.sat_imei) == 0)
	{
		satfi_log("sat_imei empty");
		return -1;
	}

	int n = Data_To_Tsc(buf, p->length);
	if (n<0)
	{
		satfi_log("sendto tsc return error: errno=%d %s %d\n", errno, strerror(errno),__LINE__);
		return -1;
	}

	return 0;
}

static int sendHeartBeat()
{
	char buf[1024];
	char users[1024]={0};
	MsgHeader *p = (MsgHeader *)buf;
	int offset = 4;
	p->mclass = SATFI_HEART_BEAT_CMD;
	strncpy(&buf[4], base.sat.sat_imei, IMSI_LEN);
	offset += IMSI_LEN;

	*(unsigned short *)&buf[offset] = strlen(GpsData);
	offset += 2;

	satfi_log("GpsData=%s %d", GpsData, strlen(GpsData));

	strcpy(&buf[offset], GpsData);
	offset += strlen(GpsData);

	int iCntUser = 0;
	USER *pUser = gp_users;

	while(pUser)
	{
		if(pUser->socketfd > 0)
		{
			if(strncmp(pUser->userid, "000000000000000000000", USERID_LLEN) != 0)
			{
				strncpy(&users[iCntUser * USERID_LEN], &pUser->userid[USERID_LLEN - USERID_LEN], USERID_LEN); 
				iCntUser ++;
			}
		}
		pUser = pUser->next;
	}

	*(unsigned short *)&buf[offset] = iCntUser;
	offset += 2;
	strncpy(&buf[offset], users, iCntUser * USERID_LEN);
	offset += iCntUser * USERID_LEN;
	p->length = offset;

	satfi_log("Send HeartBeat Message to TSC Server iCntUser=%d users=%s length=%d %d\n", iCntUser, users, p->length,__LINE__);
	//printfhex(buf, p->length);
	int n = Data_To_Tsc(buf, p->length);
	if (n<0)
	{
		satfi_log("sendto tsc return error: errno=%d %s %d\n", errno, strerror(errno),__LINE__);
		return -1;
	}
	
	return 0;
}

static int sendHeartbeatSP(void)
{
	char buf[128];
	MsgHeader *p = (MsgHeader *)buf;
	p->length = 19;
	p->mclass = SATFI_HEART_BEAT_SP_CMD;
	strncpy(&buf[4], base.sat.sat_imei, IMSI_LEN);

	satfi_log("Send heartbeatSP Message to TSC Server\n");
	int n = Data_To_Tsc(buf, p->length);
	if (n<0)
	{
		satfi_log("sendto tsc return error: errno=%d %s %d\n", errno, strerror(errno),__LINE__);
		return -1;
	}

	return 0;
}

void updateMSList(void)
{	
	char tmp[1024] = {0};
	char userlist[512] = {0};
	Msg_MS_List_Rep *rep = (Msg_MS_List_Rep *)tmp;
	rep->header.mclass = SATFI_MS_LIST_CMD;

	strncpy(rep->sat_imsi, base.sat.sat_imei, IMSI_LEN);
	
	int iCntUser = 0;
	USER *pUser = gp_users;
	time_t now = time(0);
	int offset = 0;
	while(pUser)
	{
		if(pUser->socketfd > 0)
		{
			if(strncmp(pUser->userid, "000000000000000000000", USERID_LLEN) != 0)
			{
				strncpy(&(userlist[offset*2]), &(pUser->userid[9]), USERID_LEN); 
				StrToBcd(&(rep->Users[offset]), &(pUser->userid[9]), 12);
				iCntUser ++;
				offset+=6;
			}
		}
		pUser = pUser->next;
	}

	rep->usercnt = iCntUser;
	rep->header.length = sizeof(Msg_MS_List_Rep) + iCntUser*6;
	AppCnt = iCntUser;
	satfi_log("SATFI_MS_LIST_CMD useridlist=%s\n", userlist);

	int n = Data_To_Tsc(tmp, rep->header.length);
	if (n<0)
	{
		satfi_log("write tsc return error: errno=%d %s %d\n", errno, strerror(errno),__LINE__);
		return;
	}

}

int CheckIsProjectApp(char *msid)
{
	if(msid && strncmp(msid, "000000000000000000000", USERID_LLEN) == 0)
	{
		return 1;
	}

	return 0;
}

int CheckProjectAppIsConnect(char *msid)
{
	USER *pUser = gp_users;
	int stat = 0;

	if(CheckIsProjectApp(msid))
	{
		while(pUser)
		{
			if(strncmp(pUser->userid, "000000000000000000000", USERID_LLEN) == 0)
			{
				if(pUser->socketfd > 0)
				{
					stat = 1;
				}
			}

			pUser = pUser->next;
		}		
	}

	return stat;
}

void add_user(char *msid, int socketfd, int count ,char FamiliarityNumber[][USERID_LLEN])
{
	USER *pUser = gp_users;
	int i;
	
	while(pUser)
	{
		if(strncmp(pUser->userid, msid, USERID_LLEN) == 0)
		{
			satfi_log("UPDATE FIND USER %.21s %d %d\n",msid, pUser->socketfd, sizeof(pUser->FamiliarityNumber)/sizeof(pUser->FamiliarityNumber[0]));
			pUser->socketfd = socketfd;
			strncpy(pUser->userid, msid, USERID_LLEN);
			pUser->famNumConut=count;
			for(i=0;i<count && i<sizeof(pUser->FamiliarityNumber)/sizeof(pUser->FamiliarityNumber[0]);i++)
			{
				strncpy(pUser->FamiliarityNumber[i], FamiliarityNumber[i], USERID_LLEN);
				satfi_log("FamiliarityNumber[%d] %.21s\n",i, pUser->FamiliarityNumber[i]);
			}

			if(count==0)
			{
				for(i=0;i<sizeof(pUser->FamiliarityNumber)/sizeof(pUser->FamiliarityNumber[0]);i++)
				{
					bzero(pUser->FamiliarityNumber[i], USERID_LLEN);
				}
			}
			
			break;
		}
		pUser = pUser->next;
	}

	if(pUser == NULL)
	{
		satfi_log("MALLOC ADD USER %.21s size=%d\n",msid, sizeof(USER));
		pUser = (USER *)malloc(sizeof(USER));
		if(pUser)
		{
			bzero(pUser,sizeof(USER));
			pUser->socketfd = socketfd;
			strncpy(pUser->userid, msid, USERID_LLEN);
			pUser->famNumConut=count;
			for(i=0;i<count && i<sizeof(pUser->FamiliarityNumber)/sizeof(pUser->FamiliarityNumber[0]);i++)
			{
				strncpy(pUser->FamiliarityNumber[i], FamiliarityNumber[i], USERID_LLEN);
				satfi_log("FamiliarityNumber[%d] %.21s\n",i, pUser->FamiliarityNumber[i]);
			}
			pUser->next = gp_users;
			gp_users = pUser;
		}
	}
}

void *WifireLoad(void *p)
{
	seconds_sleep(10);
	myexec("wifi reload",NULL,NULL);
	seconds_sleep(10);
	pthread_exit(NULL);
}

void udp_voice_calc_money(char *msid, int money)
{
	char tmp[128] = {0};
	Msg_Phone_Money_Req *rsp = (Msg_Phone_Money_Req*)tmp;
	rsp->header.length = sizeof(Msg_Phone_Money_Req);
	rsp->header.mclass = APP_PHONE_MONEY_CMD;
	StrToBcd(rsp->MsID, &(msid[USERID_LLEN - USERID_LEN]), USERID_LEN);
	memcpy(rsp->MsPhoneNum, rsp->MsID, sizeof(rsp->MsID));
	memcpy(rsp->DesPhoneNum, rsp->MsID, sizeof(rsp->MsID));
	rsp->StartTime = 0;
	rsp->CallTime = 0;
	rsp->Money = money;
	
	SaveDataToFile(CALL_RECORDS_FILE ,(char *)rsp, rsp->header.length);
}

int handle_app_msg_tcp(int socket, char *pack, char *tscbuf)
{
	char *tmp = tscbuf;
	int n = 0;
	MsgHeader *p = (MsgHeader *)pack;
	static int chat = 0;
	static char chatMsid[21];
	
	switch(p->mclass)
	{
		case CONNECT_CMD:
		{
			MsgAppConnectReq* req = (MsgAppConnectReq*)p;
			if(p->length == 25)
			{
				//satfi_log("CONNECT_CMD DATA ERROR %d %d", p->length, sizeof(MsgAppConnectReq), socket);
				//break;
				req->Count = 0;
			}
			satfi_log("FamiliarityNumber req->Count=%d %d", req->Count, p->length);

			int result = CheckProjectAppIsConnect(req->MsID);

			satfi_log("CheckProjectAppIsConnect %d %d", result, socket);

			if(result == 0)
			{
				add_user(req->MsID, socket, req->Count, req->FamiliarityNumber);
			}

			if(base.sat.sat_state_phone != SAT_STATE_PHONE_IDLE && strncmp(req->MsID, base.sat.MsID, USERID_LLEN) == 0)
			{
				satfi_log("update base.sat.socket = %d socket = %d", base.sat.socket, socket);
				base.sat.socket = socket;
			}
			
			if(bTscConnected)
			{
				updateMSList();
			}

			//response
			{
				memset(tmp,0,2048);
				MsgAppConnectRsp *rsp = (MsgAppConnectRsp *)tmp;
				rsp->header.length = sizeof(MsgAppConnectRsp);
				rsp->header.mclass = CONNECT_RSP;
				rsp->result = result;
				rsp->n3g_status = get_n3g_status();
				rsp->sat_status = get_sat_status();
				strncpy(rsp->n3g_imei, base.n3g.n3g_imei, IMSI_LEN);
				strncpy(rsp->n3g_imsi, base.n3g.n3g_imsi, IMSI_LEN);
				strncpy(rsp->sat_imei, base.sat.sat_imei, IMSI_LEN);
				strncpy(rsp->sat_imsi, base.sat.sat_imsi, IMSI_LEN);
				strncpy(rsp->version, satfi_version, 32);
				n = write(socket, tmp, rsp->header.length);
				if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),__LINE__,socket);
			}
		}
		break;
		case HEART_BEAT_CMD:
		{
			if(p->length != sizeof(MsgAppHeartbeatReq))
			{
				satfi_log("HEART_BEAT_CMD DATA ERROR");
				return -1;
			}

			//satfi_log("HEART_BEAT_CMD %d", socket);

			MsgAppHeartbeatReq *req = (MsgAppHeartbeatReq *)pack;
			//satfi_log("req->Type=%d %d",req->Type,base.sat.captain_socket);
			if(req->Type == 6)
			{
				if(base.sat.captain_socket < 0)
				{
					satfi_log("captain_socket=%d",socket);
					base.sat.captain_socket = socket;
				}
			}
			
			//response
			{
				memset(tmp, 0, 2048);
				MsgAppHeartbeatRsp *rsp = (MsgAppHeartbeatRsp *)tmp;
				rsp->header.length = sizeof(MsgAppHeartbeatRsp);
				rsp->header.mclass = HEART_BEAT_RSP;
				rsp->result = 0;
				rsp->n3g_status = get_n3g_status();
				strncpy(rsp->n3g_imei, base.n3g.n3g_imei, IMSI_LEN);
				strncpy(rsp->n3g_imsi, base.n3g.n3g_imsi, IMSI_LEN);
				make_csq(rsp->n3g_csq,(time_t *)&base.n3g.n3g_csq_ltime, base.n3g.n3g_csq_value);
				rsp->sat_status = get_sat_status();
				strncpy(rsp->sat_imei, base.sat.sat_imei, IMSI_LEN);
				strncpy(rsp->sat_imsi, base.sat.sat_imsi, IMSI_LEN);
				make_csq(rsp->sat_csq,(time_t *)&base.sat.sat_csq_ltime, base.sat.sat_csq_value);
				strncpy(rsp->sat_gps, base.sat.sat_gps, 127);
				strncpy(rsp->bd_gps, base.gps.gps_bd, 127);
				strncpy(rsp->version, satfi_version, 32);
				n = write(socket, tmp, rsp->header.length);
				if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),__LINE__,socket);
			}

			//satfi_log("HEART_BEAT_CMD %.21s\n",&buf[8]);
		}
		break;

		case QUERY_MS_LIST_CMD:
		{
			if(p->length != sizeof(MsgQueryMsList_Req))
			{
				break;
			}
			MsgQueryMsList_Rsp* rsp = (MsgQueryMsList_Rsp*)tmp;
			rsp->header.mclass = QUERY_MS_LIST_RSP;

			int iCntUser = 0;
			USER *pUser = gp_users;
			USER *q = pUser;
			time_t now = time(0);
			int offset = 0;
			while(pUser)
			{
				if(pUser->socketfd > 0)
				{
					strncpy(&(rsp->UserList[offset]), pUser->userid, USERID_LLEN); 
					iCntUser ++;
					offset+=USERID_LLEN;
				}
				q = pUser;
				pUser = pUser->next;
			}

			//printf("%d %s\n",iCntUser,rsp->UserList);

			rsp->MsNum = iCntUser;
			rsp->header.length = sizeof(MsgQueryMsList_Rsp) + iCntUser * USERID_LLEN;
			n = write(socket, tmp, rsp->header.length);
			if(n<0) satfi_log("write return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case SENDOPERATION_CMD:
		{
			//printf("get from app SENDOPERATION_CMD\n");
		}
		break;

		case SENDLANMESSAGE1_CMD:
		{
			USER *pUser = gp_users;
			char msid[21];
			unsigned short type = *((unsigned short*)(&pack[4]));
			if(type == 0x0001)
			{
				strncpy(msid,&pack[16],USERID_LLEN);
			}
			else if(type == 0x0801)
			{
				strncpy(msid,&pack[12],USERID_LLEN);
			}
			else
			{
				satfi_log("SENDLANMESSAGE1_CMD DATA ERROR\n");
				break;
			}

			while(pUser)
			{
				if(strncmp(pUser->userid, msid, USERID_LLEN) == 0)
				{
					//satfi_log("%.21s\n",pUser->userid);
					if(pUser->socketfd < 0)break;
					n = write(pUser->socketfd, pack, p->length);
					if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),__LINE__,pUser->socketfd);
					break;
				}
				pUser = pUser->next;
			}
		}
		break;

		case SENDLANMESSAGE2_CMD:
		{
			USER *pUser = gp_users;
			while(pUser)
			{
				if(pUser->socketfd > 0)
				{
					if(pUser->socketfd != socket)
					{
						//satfi_log("SENDLANMESSAGE2_CMD %s %d\n", pUser->userid, p->length);
						n = write(pUser->socketfd, pack, p->length);
						if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),__LINE__,pUser->socketfd);
					}
				}

				pUser = pUser->next;
			}

			//satfi_log("SENDLANMESSAGE2_CMD END\n");
		}
		break;

		case SAT_MODE_SWITCH_CMD:
		{
		}
		break;

		case SET_WIFI_PASSWD_CMD:
		{
			char Cmd[128]={0};
			char passwd[32];
			strncpy(passwd,&pack[4],32);

			satfi_log("SET_WIFI_PASSWD_CMD passwd=%s\n", passwd);
			
			sprintf(Cmd,"uci set wireless.@wifi-iface[0].key=%s", passwd);
			satfi_log("%s",Cmd);
			myexec(Cmd,NULL,NULL);
			satfi_log("uci commit");
			myexec("uci commit",NULL,NULL);
			//myexec("wifi reload",NULL,NULL);
			pthread_t WifireLoadThread;
			pthread_create(&WifireLoadThread,NULL,WifireLoad,NULL);
		}
		break;
		
		case SATFI_RESTART_CMD:
		{
			satfi_log("SATFI_RESTART_CMD\n");
			exit(8);
		}
		break;
		
		case SATFI_UPDATE_CMD:
		{
			satfi_log("SATFI_UPDATE_CMD\n");
		}
		break;
		
		case GRP_CHAT_CMD: 
		{
			#define GROUP_MESSAGE_ID_NUM 100
			static unsigned int GroupMessageIDNum = 0;
			static unsigned int GroupMessageID[GROUP_MESSAGE_ID_NUM] = {0};
			
			MsgGrpChatReq *req = (MsgGrpChatReq *)pack;
			int datalen = req->header.length - ((int)req->data - (int)req);
			
			satfi_log("GRP_CHAT_CMD %.21s %d %d\n",req->MsID, req->Name,req->header.length);

			MsgGrpChatRsp *rsp = (MsgGrpChatRsp*)tmp;
			rsp->header.length = sizeof(MsgGrpChatRsp) + datalen;
			rsp->header.mclass = GRP_CHAT_RSP;
			memcpy(rsp->MsID, req->ID, USERID_LLEN);
			memcpy(rsp->TargetGrpID, req->TargetGrpID, USERID_LLEN);
			rsp->Name = req->Name;
			rsp->Type = req->Type;
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);

			int i;
			int IsSend = 0;
			for(i=0;i<GROUP_MESSAGE_ID_NUM;i++)
			{
				if(req->Name == GroupMessageID[i])
				{
					IsSend = 1;
					break;
				}				
			}

			if(IsSend == 1)
			{
				satfi_log("SEND GRP_CHAT_CMD %.21s %d %d\n",req->MsID, req->Name, GroupMessageIDNum);
				break;
			}
			else
			{
				GroupMessageID[GroupMessageIDNum] = req->Name;
				GroupMessageIDNum++;
				satfi_log("ADD GRP_CHAT_CMD %.21s %d %d\n",req->MsID, req->Name, GroupMessageIDNum);
				if(GroupMessageIDNum == GROUP_MESSAGE_ID_NUM)
				{
					GroupMessageIDNum = 0;
				}
			}

			n = write(socket, tmp, rsp->header.length);
			if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),__LINE__,socket);
		}
		break;

		case CALLUP_CMD:
		{
			short sat_state_phone = get_sat_dailstatus();
			satfi_log("CALLUP_CMD");
			MsgAppCallUpRsp rsp;
			rsp.header.length = sizeof(MsgAppCallUpRsp);
			rsp.header.mclass = CALLUP_RSP;
			if(base.sat.sat_calling == 0)
			{
				bzero(base.sat.MsID,sizeof(base.sat.MsID));
				bzero(base.sat.MsPhoneNum,sizeof(base.sat.MsPhoneNum));
				bzero(base.sat.DesPhoneNum,sizeof(base.sat.DesPhoneNum));
				
				USER *pUser = gp_users;
				while(pUser)
				{
					if(pUser->socketfd == socket)
					{
						memcpy(base.sat.MsID, pUser->userid, 21);
						memcpy(base.sat.MsPhoneNum, &(pUser->userid[USERID_LLEN - 11]), 11);
						break;
					}
					pUser = pUser->next;
				}

				int len = strlen(&pack[4]);
				if(len >= 11 && len <= 15)memcpy(base.sat.DesPhoneNum, &pack[4 + len - 11], 11);

				base.sat.EndTime = 0;
				base.sat.StartTime = 0;
				base.sat.CallTime = 0;
				base.sat.Money = 0;

				satfi_log("%d %.21s %.11s %.11s",len, base.sat.MsID, base.sat.MsPhoneNum, base.sat.DesPhoneNum);
				
				base.sat.sat_calling = 1;
				base.sat.socket = socket;
				strncpy(base.sat.calling_number, &pack[4],sizeof(base.sat.called_number));
				StartCallUp(base.sat.calling_number);
			}
			else
			{
				sat_state_phone = 7;//返回拨号失败,当前已有正在拨号
			}
			
			rsp.result = sat_state_phone;			
			n = write(socket, &rsp, rsp.header.length);
			
			satfi_log("CALLUP_CMD %d %d calling_number=%.32s\n", base.sat.sat_calling, base.sat.sat_state_phone, base.sat.calling_number);
			//printf("CALLUP_CMD %d %d calling_number=%.15s\n", base.sat.sat_calling, sat_state_phone, base.sat.calling_number);
		}
		break;
		
		case HANGINGUP_CMD:
		{
			satfi_log("HANGINGUP_CMD %d %d %d phone=%s\n",base.sat.sat_calling, base.sat.socket, socket, &pack[4]);

			if(base.sat.sat_calling == 1)
			{
				MsgAppHangUpRsp rsp;
				rsp.header.length = sizeof(MsgAppHangUpRsp);
				rsp.header.mclass = HANGINGUP_RSP;
				rsp.result = 0;
				n = write(socket, &rsp, rsp.header.length);
				
				base.sat.sat_state_phone = SAT_STATE_PHONE_ATH_W;
			}
		}
		break;
		
		case RINGUP_CMD:
		{
			printf("RINGUP_CMD\n");
		}
		break;
		
		case ANSWERINGPHONE_CMD:
		{
			//printf("ANSWERINGPHONE_CMD\n");
			if(base.sat.sat_calling == 1)
			{
				satfi_log("ANSWERINGPHONE_CMD %d\n", base.sat.sat_state_phone);
				if(base.sat.sat_state_phone==SAT_STATE_PHONE_RING_COMING)
				{
					MsgAppHangUpRsp rsp;
					rsp.header.length = sizeof(MsgAppHangUpRsp);
					rsp.header.mclass = ANSWERINGPHONE_RSP;
					rsp.result = 0;
					n = write(socket, &rsp, rsp.header.length);			
				}
				else
				{
					AppCallUpRsp(socket, 6);
					base.sat.sat_calling = 0;
				}
				
				AnsweringPhone();
			}
		}
		break;
		
		case GET_LOG_CMD:
		{
			if(p = get_log_data(&pack[4]))
			{
				satfi_log("GET_LOG_CMD %.21s %d %x\n",&pack[4],p->length,p->mclass);
				n = write(socket, p, p->length);
				if(n != p->length)
				{
					satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,p->length,__LINE__);
				}
				if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),__LINE__,socket);
			}
		}
		break;
		
		case QUERY_GROUP_CMD:
		{
			if(p->length != sizeof(MsgQueryGroupReq))
			{
				break;
			}
			MsgQueryGroupReq *req = (MsgQueryGroupReq *)pack;

			Msg_Query_Group_Req* rsp = (Msg_Query_Group_Req*)tmp;
			rsp->header.length = sizeof(Msg_Query_Group_Req);
			rsp->header.mclass = APP_QUERY_GROUP_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			satfi_log("APP_QUERY_GROUP_CMD %.21s %d %d\n",req->MsID, socket,rsp->header.length);
			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case QUERY_MS_CMD:
		{			
			if(p->length != sizeof(MsgQueryMsReq))
			{
				break;
			}

			MsgQueryMsReq *req = (MsgQueryMsReq *)pack;

			Msg_Query_Ms_Req* rsp = (Msg_Query_Ms_Req*)tmp;
			rsp->header.length = sizeof(Msg_Query_Ms_Req);
			rsp->header.mclass = APP_QUERY_MS_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);

			satfi_log("APP_QUERY_MS_CMD %.21s %d %d\n",req->MsID, socket,rsp->header.length);
			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case SET_BLOCK_CMD:
		{
			if(p->length != sizeof(MsgSetBlockReq))
			{
				//printf("diff get from app SET_BLOCK_CMD\n");
				break;
			}
			MsgSetBlockReq *req = (MsgSetBlockReq *)pack;

			Msg_SetBlock_Req* rsp = (Msg_SetBlock_Req*)tmp;
			rsp->header.length = sizeof(Msg_SetBlock_Req);
			rsp->header.mclass = APP_SET_BLOCK_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);

			satfi_log("APP_SET_BLOCK_CMD %.21s %.21s\n",req->MsID, req->GrpID);
			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);

			USER *pUser = gp_users;
			while(pUser)
			{
				if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
				{
					if(pUser->socketfd > 0)
					{
						strncpy(pUser->defgrp, req->GrpID, USERID_LLEN);
						satfi_log("MsID=%.21s defgrp=%.21s", req->MsID, pUser->defgrp);
					}
					break;
				}
				pUser = pUser->next;
			}

		}
		break;

		case UPLOAD_MESSAGE_CMD:
		{
			satfi_log("UPLOAD_MESSAGE_CMD");
			int msglen;
			MsgUploadMessageReq *req = (MsgUploadMessageReq *)pack;
			msglen = req->header.length - ((int)req->Message - (int)req);

			Msg_Upload_Message_Req* rsp = (Msg_Upload_Message_Req*)tmp;
			rsp->header.length = sizeof(Msg_Upload_Message_Req) + msglen;
			rsp->header.mclass = APP_UPLOAD_MESSAGE_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TargetGrpID, &(req->TargetGrpID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Name = req->Name;
			if(msglen > 0)memcpy(rsp->Message, req->Message, msglen);

			MsgUploadMessageRsp *MsgRsp = get_log_data(req->MsID);
			if(MsgRsp != NULL)
			{
				satfi_log("MsgRsp->Name=%d, rsp->Name=%d", MsgRsp->Name, rsp->Name);
				if(MsgRsp->Name == rsp->Name)
				{
					break;
				}
			}
			Message_Pack_Add(tmp, rsp->header.length, rsp->Name);
			break;
		}
		break;

		case UPLOAD_VOICE_CMD:
		{				
			int datalen;
			MsgUploadVoiceReq *req = (MsgUploadVoiceReq *)pack;
			datalen = req->header.length - ((int)req->data - (int)req);

			Msg_Upload_Voice_Req* rsp = (Msg_Upload_Voice_Req*)tmp;
			rsp->header.length = sizeof(Msg_Upload_Voice_Req) + datalen;
			rsp->header.mclass = APP_UPLOAD_VOICE_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TargetGrpID, &(req->TargetGrpID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Name = req->Name;
			rsp->lengthtotal= req->lengthtotal;
			rsp->packseq = req->packseq;
			rsp->packtotal = req->packtotal;
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);

			MsgUploadVoiceRsp *VoiceRsp = get_log_data(req->MsID);
			if(VoiceRsp != NULL)
			{
				if(VoiceRsp->Name == rsp->Name)
				{
					break;
				}
			}
			
			if(rsp->packseq == 1)satfi_log("APP_UPLOAD_VOICE_CMD %.21s rsp->name=%d lengthtotal=%d packseq=%d\n",req->MsID, rsp->Name, rsp->lengthtotal, rsp->packseq);
			if(rsp->packseq == rsp->packtotal)satfi_log("APP_UPLOAD_VOICE_CMD %.21s rsp->name=%d lengthtotal=%d packtotal=%d\n",req->MsID, rsp->Name, rsp->lengthtotal, rsp->packtotal);
			Voice_Pic_Pack_Add(1, tmp, rsp->header.length, rsp->packseq, rsp->packtotal, rsp->Name);
			break;

		}
		break;

		case UPLOAD_PICTURE_CMD:
		{
			int datalen;
			MsgUploadPictureReq *req = (MsgUploadPictureReq *)pack;
			datalen = req->header.length - ((int)req->data - (int)req);

			Msg_Upload_Picture_Req* rsp = (Msg_Upload_Picture_Req*)tmp;
			rsp->header.length = sizeof(Msg_Upload_Picture_Req) + datalen;
			rsp->header.mclass = APP_UPLOAD_PICTURE_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), 12);
			StrToBcd(rsp->TargetGrpID, &(req->TargetGrpID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Name = req->Name;
			rsp->lengthtotal= req->lengthtotal;
			rsp->packseq = req->packseq;
			rsp->packtotal = req->packtotal;
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);

			MsgUploadPictureRsp *PicRsp = get_log_data(req->MsID);
			if(PicRsp != NULL)
			{
				if(PicRsp->Name == rsp->Name)
				{
					break;
				}
			}
			
			if(rsp->packseq == 1)satfi_log("APP_UPLOAD_PICTURE_CMD %.21s rsp->name=%d packseq=%d %d\n", req->MsID, rsp->Name, rsp->packseq, req->lengthtotal);
			if(rsp->packseq == rsp->packtotal)satfi_log("APP_UPLOAD_PICTURE_CMD %.21s rsp->name=%d packseq=%d packtotal=%d\n", req->MsID, rsp->Name, rsp->packseq, rsp->packtotal);
			Voice_Pic_Pack_Add(2, tmp, rsp->header.length, rsp->packseq, rsp->packtotal, rsp->Name);
			break;
		}
		break;

		case READ_OK_CMD:
		{	
			MsgReadOKReq *req = (MsgReadOKReq *)pack;

			Msg_Read_OK_Req* rsp = (Msg_Read_OK_Req*)tmp;
			rsp->header.length = sizeof(Msg_Read_OK_Req);
			rsp->header.mclass = APP_READ_OK_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->ID = req->ID;
			rsp->Type = req->Type;

			satfi_log("APP_READ_OK_CMD %.21s %d %d\n",req->MsID, socket,rsp->header.length);
			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case GET_MESSAGE_CMD:
		{
			MsgGetMessageReq *req = (MsgGetMessageReq *)pack;
			if(p->length != sizeof(MsgGetMessageReq))
			{
				//printf("diff get from app GET_MESSAGE_CMD %d %d\n",p->length,sizeof(MsgGetMessageReq));
				break;
			}
			
			Msg_Get_Message_Req* rsp = (Msg_Get_Message_Req*)tmp;
			rsp->header.length = sizeof(Msg_Get_Message_Req);
			rsp->header.mclass = APP_GET_MESSAGE_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			satfi_log("get from app APP_GET_MESSAGE_CMD %.21s %d\n",req->MsID, socket);
			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case ZF_MESSAGE_CMD:
		{
			int datalen;
			MsgZfMessageReq *req = (MsgZfMessageReq *)pack;
			datalen = req->header.length - ((int)req->Path - (int)req);

			Msg_Zf_Message1_Req* rsp1 = (Msg_Zf_Message1_Req*)tmp;
			Msg_Zf_Message2_Req* rsp2 = (Msg_Zf_Message2_Req*)tmp;

			if(req->Type == 0)
			{
				rsp1->header.length = sizeof(Msg_Zf_Message1_Req) + datalen;
				rsp1->header.mclass = APP_ZF_MESSAGE_CMD;
				StrToBcd(rsp1->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
				rsp1->Type = req->Type;
				StrToBcd(rsp1->TargetID, &(req->TargetID[USERID_LLEN - USERID_LEN]), USERID_LEN);
				rsp1->ID = req->ID;
				rsp1->MessageType = req->MessageType;
				if(datalen > 0)memcpy(rsp1->Path, req->Path, datalen);
				n = Data_To_Tsc(tmp, rsp1->header.length);
				if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);	
			}
			else if(req->Type == 1)
			{
				rsp2->header.length = sizeof(Msg_Zf_Message2_Req) + datalen;
				rsp2->header.mclass = APP_ZF_MESSAGE_CMD;
				StrToBcd(rsp2->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
				rsp2->Type = req->Type;
				StrToBcd(rsp2->TargetID, req->TargetID, USERID_LLEN);
				rsp2->ID = req->ID;
				rsp2->MessageType = req->MessageType;
				if(datalen > 0)memcpy(rsp2->Path, req->Path, datalen);
				n = Data_To_Tsc(tmp, rsp2->header.length);
				if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);	
			}
		}
		break;

		case EMERGENCY_ALARM_CMD:
		{
			MsgEmergencyAlarmReq *req = (MsgEmergencyAlarmReq *)pack;

			Msg_Emergency_Alarm_Req* rsp = (Msg_Emergency_Alarm_Req*)tmp;
			rsp->header.length = sizeof(Msg_Emergency_Alarm_Req);
			rsp->header.mclass = APP_EMERGENCY_ALARM_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Lg 		= req->Lg;
			rsp->Lt 		= req->Lt;
			rsp->AlarmType	= req->AlarmType;

			CreateSOSMessage(req->MsID, req->Lg, req->Lt, req->AlarmType);

			satfi_log("APP_EMERGENCY_ALARM_CMD %.21s %d %d %d\n",req->MsID, rsp->Lg, rsp->Lt, rsp->AlarmType);
			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case CANCEL_EMERGENCY_ALARM_CMD:
		{
			int Remarklen;
			MsgCancelEmergencyAlarm_Req *req = (MsgCancelEmergencyAlarm_Req *)pack;
			Remarklen = req->header.length - ((int)req->Remark - (int)req);

			Msg_Cancel_Emergency_Alarm_Req* rsp = (Msg_Cancel_Emergency_Alarm_Req*)tmp;
			rsp->header.length = sizeof(Msg_Cancel_Emergency_Alarm_Req) + Remarklen;
			rsp->header.mclass = APP_CANCEL_EMERGENCY_ALARM_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Type = req->Type;
			if(Remarklen > 0)memcpy(rsp->Remark, req->Remark, Remarklen);

			satfi_log("APP_CANCEL_EMERGENCY_ALARM_CMD %.21s %d %d\n",req->MsID, socket,rsp->header.length);
			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case OPERAT_CMD:
		{
			int GrpNamelen;
			MsgAppOperat_Req *req = (MsgAppOperat_Req *)pack;
			GrpNamelen = req->header.length - ((int)req->GrpName - (int)req);

			Msg_App_Operat_Req *rsp = (Msg_App_Operat_Req*)tmp;
			rsp->header.length = sizeof(Msg_App_Operat_Req) + GrpNamelen;
			rsp->header.mclass = APP_OPERAT_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Operation = req->Operation;
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			if(GrpNamelen > 0)memcpy(rsp->GrpName, req->GrpName, GrpNamelen);

			satfi_log("APP_OPERAT_CMD %.21s %d %d\n",req->MsID, socket,rsp->header.length);
			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case PHONE_MONEY_CMD:
		{
			MsgPhoneMoneyReq *req = (MsgPhoneMoneyReq *)pack;
			satfi_log("PHONE_MONEY_CMD %.21s %d %d\n",req->MsID, socket,req->header.length);

			Msg_Phone_Money_Req *rsp = (Msg_Phone_Money_Req*)tmp;
			rsp->header.length = sizeof(Msg_Phone_Money_Req);
			rsp->header.mclass = APP_PHONE_MONEY_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->MsPhoneNum, req->MsPhoneNum, 11);
			StrToBcd(rsp->DesPhoneNum, req->DesPhoneNum, 11);
			rsp->StartTime = req->StartTime;
			rsp->CallTime = req->CallTime;
			rsp->Money = req->Money;

			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case GRP_UPLOAD_MESSAGE_CMD:
		{
			MsgGrpUploadMessageReq *req = (MsgGrpUploadMessageReq *)pack;
			int msglen = req->header.length - ((int)req->Message - (int)req);
			satfi_log("GRP_UPLOAD_MESSAGE_CMD %.21s req->ID=%d %d\n",req->MsID, req->Name, req->header.length);
			
			Msg_Grp_Upload_Message_Req *rsp = (Msg_Grp_Upload_Message_Req*)tmp;
			rsp->header.length = sizeof(Msg_Grp_Upload_Message_Req) + msglen;
			rsp->header.mclass = APP_GRP_UPLOAD_MESSAGE_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TargetGrpID, req->TargetGrpID, USERID_LLEN);
			rsp->Name = req->Name;
			if(msglen > 0)memcpy(rsp->Message, req->Message, msglen);

			MsgGrpUploadMessageRsp *GrpMsgRsp = get_log_data(req->MsID);
			if(GrpMsgRsp != NULL)
			{
				satfi_log("GrpMsgRsp->Name=%d, rsp->Name=%d", GrpMsgRsp->Name, rsp->Name);
				if(GrpMsgRsp->Name == rsp->Name)
				{
					break;
				}
			}
			Message_Pack_Add(tmp, rsp->header.length, rsp->Name);
			//n = Data_To_Tsc(tmp, rsp->header.length);
			//if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case GRP_UPLOAD_VOICE_CMD:
		{
			MsgGrpUploadVoiceReq *req = (MsgGrpUploadVoiceReq *)pack;
			int datalen = req->header.length - ((int)req->data - (int)req);
			
			Msg_Grp_Upload_Voice_Req *rsp = (Msg_Grp_Upload_Voice_Req*)tmp;
			rsp->header.length = sizeof(Msg_Grp_Upload_Voice_Req) + datalen;
			rsp->header.mclass = APP_GRP_UPLOAD_VOICE_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TargetGrpID, req->TargetGrpID, USERID_LLEN);
			rsp->Name = req->Name;
			rsp->lengthtotal= req->lengthtotal;
			rsp->packseq = req->packseq;
			rsp->packtotal = req->packtotal;
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);

			MsgGrpUploadVoiceRsp *GrpVoiceRsp = get_log_data(req->MsID);
			if(GrpVoiceRsp != NULL)
			{
				if(GrpVoiceRsp->Name == rsp->Name)
				{
					break;
				}
			}
			
			if(rsp->packseq == 1)satfi_log("GRP_UPLOAD_VOICE_CMD %.21s rsp->name=%d lengthtotal=%d packseq=%d\n",req->MsID, rsp->Name, rsp->lengthtotal, rsp->packseq);
			if(rsp->packseq == rsp->packtotal)satfi_log("GRP_UPLOAD_VOICE_CMD %.21s rsp->name=%d lengthtotal=%d packtotal=%d\n",req->MsID, rsp->Name, rsp->lengthtotal, rsp->packtotal);
			Voice_Pic_Pack_Add(1, tmp, rsp->header.length, rsp->packseq, rsp->packtotal, rsp->Name);
			//if(rsp->packseq == 1 || rsp->packseq == rsp->packtotal)satfi_log("GRP_UPLOAD_VOICE_CMD %.21s rsp->name=%d packseq=%d packtotal=%d\n",req->MsID, rsp->name, rsp->packseq, rsp->packtotal);
			//n = Data_To_Tsc(tmp, rsp->header.length);
			//if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case GRP_UPLOAD_PICTURE_CMD:
		{
			MsgGrpUploadPictureReq *req = (MsgGrpUploadPictureReq *)pack;
			int datalen = req->header.length - ((int)req->data - (int)req);
			
			Msg_Grp_Upload_Picture_Req *rsp = (Msg_Grp_Upload_Picture_Req*)tmp;
			rsp->header.length = sizeof(Msg_Grp_Upload_Picture_Req) + datalen;
			rsp->header.mclass = APP_GRP_UPLOAD_PICTURE_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TargetGrpID, req->TargetGrpID, USERID_LLEN);
			rsp->Name = req->Name;
			rsp->lengthtotal= req->lengthtotal;
			rsp->packseq = req->packseq;
			rsp->packtotal = req->packtotal;
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);

			MsgGrpUploadPictureRsp *GrpPicRsp = get_log_data(req->MsID);
			if(GrpPicRsp != NULL)
			{
				if(GrpPicRsp->Name == rsp->Name)
				{
					//satfi_log("had send GrpPicRsp->Name=%d, rsp->Name=%d", GrpPicRsp->Name, rsp->Name);
					//Pack_Del(GrpPicRsp->Name);
					//write(socket, GrpPicRsp, GrpPicRsp->header.length);
					break;
				}
			}
			
			if(rsp->packseq == 1)satfi_log("GRP_UPLOAD_PICTURE_CMD %.21s rsp->name=%d packtotal=%d %d\n", req->MsID, rsp->Name, rsp->packtotal, req->packseq);
			if(rsp->packseq == rsp->packtotal)satfi_log("GRP_UPLOAD_PICTURE_CMD %.21s rsp->name=%d packtotal=%d %d\n", req->MsID, rsp->Name, rsp->packtotal, req->packseq);
			Voice_Pic_Pack_Add(2, tmp, rsp->header.length, rsp->packseq, rsp->packtotal, rsp->Name);
			//if(rsp->packseq == 1 || rsp->packseq == rsp->packtotal)satfi_log("GRP_UPLOAD_PICTURE_CMD %.21s rsp->name=%d packtotal=%d %d\n", req->MsID, rsp->name, rsp->packtotal, req->packseq);		
			//n = Data_To_Tsc(tmp, rsp->header.length);
			//if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case GRP_READ_OK_CMD:
		{
			MsgGrpReadOKReq *req = (MsgGrpReadOKReq *)pack;
			satfi_log("GRP_READ_OK_CMD %.21s %d %d %d\n",req->MsID, socket,req->header.length,req->ID);

			Msg_Grp_Read_OK_Req *rsp = (Msg_Grp_Read_OK_Req*)tmp;
			rsp->header.length = sizeof(Msg_Grp_Read_OK_Req);
			rsp->header.mclass = APP_GRP_READ_OK_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->ID = req->ID;
			rsp->Type = req->Type;

			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case SET_GRP_CHAT_CMD:
		{
			MsgSetGrpChatReq *req = (MsgSetGrpChatReq *)pack;
			satfi_log("SET_GRP_CHAT_CMD %.21s %d %d\n",req->MsID, socket,req->header.length);

			Msg_Set_Grp_Chat_Req *rsp = (Msg_Set_Grp_Chat_Req*)tmp;
			rsp->header.length = sizeof(Msg_Set_Grp_Chat_Req);
			rsp->header.mclass = APP_SET_GRP_CAHT_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			rsp->Type = req->Type;
			rsp->MsgType = req->MsgType;

			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case SET_MSG_CMD:
		{
			MsgSetMsgReq *req = (MsgSetMsgReq *)pack;
			satfi_log("SET_MSG_CMD %.21s %d %d\n",req->MsID, socket,req->header.length);

			Msg_Set_Msg_Req *rsp = (Msg_Set_Msg_Req*)tmp;
			rsp->header.length = sizeof(Msg_Set_Msg_Req);
			rsp->header.mclass = APP_SET_MSG_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Type = req->Type;
			StrToBcd(rsp->TargetMsID, &(req->TargetMsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case WL_COMMAND2:
		{
			MsgWlCommand *req = (MsgWlCommand *)pack;
			satfi_log("WL_COMMAND2 %.21s %d %d\n",req->MsID, socket,req->header.length);
			int Msglen = req->header.length - ((int)req->Msg - (int)req);

			Msg_Wl_Command *rsp = (Msg_Wl_Command*)tmp;
			rsp->header.length = sizeof(Msg_Wl_Command) + Msglen;
			rsp->header.mclass = APP_WL_COMMAND;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			if(Msglen > 0)memcpy(rsp->Msg, req->Msg, Msglen);
			
			n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case SET_ONLINE:
		{
			MsgSetOnlineReq *req = (MsgSetOnlineReq *)pack;
			satfi_log("SET_ONLINE %.21s %d %d\n",req->MsID, socket, req->header.length);
			
			Msg_Set_Online_Req *rsp = (Msg_Set_Online_Req*)tmp;
			rsp->header.length = sizeof(Msg_Set_Online_Req);
			rsp->header.mclass = APP_SET_ONLINE;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			rsp->Type = req->Type;
			
			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case GPS_POINT:
		{
			MsgGpsPointRsp *req = (MsgGpsPointRsp *)pack;

			Msg_Gps_Point_Rsp *rsp = (Msg_Gps_Point_Rsp*)tmp;
			rsp->header.length = sizeof(Msg_Gps_Point_Rsp);
			rsp->header.mclass = APP_GPS_POINT_RSP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Id = req->Id;
			rsp->Result = req->Result;
			satfi_log("GPS_POINT %.21s %d %d\n",req->MsID, rsp->header.length, req->header.length);

			n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case PTT:
		{
			MsgPtt *req = (MsgPtt *)pack;

			Msg_Ptt *rsp = (Msg_Ptt*)tmp;
			rsp->header.length = sizeof(Msg_Ptt);
			rsp->header.mclass = APP_PTT;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			satfi_log("PTT %.21s %.21s %d\n",req->MsID, req->GrpID, chat);
			
			n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);

			if(chat == 1)
			{
				MsgPttRsp *req1 = (MsgPttRsp *)tmp;
				req1->header.length = sizeof(MsgPttRsp);
				req1->header.mclass = PTT_RSP;
				req1->Result = 0x6108;//sendto app BUSY
				Data_To_MsID(req->MsID, tmp);
			}
			else
			{
				memcpy(chatMsid, req->MsID, 21);
				chat = 1;
				
				MsgPttIn *req2 = (MsgPttIn *)tmp;
				req2->header.length = sizeof(MsgPttIn);
				req2->header.mclass = PTT_IN;
				bzero(req2->PTT_MS_Name, sizeof(req2->PTT_MS_Name));
				memcpy(req2->PTT_MS_Name, &(req->MsID[USERID_LLEN - 11]), 11);
				//Data_To_ExceptMsID(req->MsID, tmp);
				Data_Transmit(req->MsID, tmp);
			}
		}
		break;

		case PTT_IN_RSP:
		{
			MsgPttInRsp *req = (MsgPttInRsp *)pack;

			Msg_Ptt_In_Rsp *rsp = (Msg_Ptt_In_Rsp*)tmp;
			rsp->header.length = sizeof(Msg_Ptt_In_Rsp);
			rsp->header.mclass = APP_PTT_IN_RSP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Result = req->Result;
			satfi_log("PTT_IN_RSP  %.21s %.21s\n",req->MsID, req->MsID);
			
			n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;
		
		case HUP:
		{
			MsgHup *req = (MsgHup *)pack;

			Msg_Hup *rsp = (Msg_Hup*)tmp;
			rsp->header.length = sizeof(Msg_Hup);
			rsp->header.mclass = APP_HUP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			satfi_log("HUP %.21s %.21s %.21s chatMsid=%.21s\n",req->MsID, req->MsID, req->GrpID, chatMsid);

			n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);

			if(strncmp(req->MsID, chatMsid, 21) == 0)
			{
				chat = 0;
				bzero(chatMsid, 21);
				MsgHupin *req1 = (MsgHupin *)tmp;
				req1->header.length = sizeof(MsgHupin);
				req1->header.mclass = HUPIN;
				//Data_To_ExceptMsID(req->MsID, req1);
				Data_Transmit(req->MsID, req1);
			}
		}
		break;

		case SPTT:
		{
			MsgSptt *req = (MsgSptt *)pack;
			int datalen = req->header.length - ((int)req->data - (int)req);

			Msg_Sptt *rsp = (Msg_Sptt*)tmp;
			rsp->header.length = sizeof(Msg_Sptt) + datalen;
			rsp->header.mclass = APP_SPTT;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			rsp->MS_Count = req->MS_Count;
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);
			satfi_log("SPTT %.21s datalen=%d %d %d\n",req->MsID, datalen, rsp->header.length, req->header.length);
			
			n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;

		case SPTT_HUP:
		{
			MsgSpttHup *req = (MsgSpttHup *)pack;

			Msg_Sptt_Hup *rsp = (Msg_Sptt_Hup*)tmp;
			rsp->header.length = sizeof(Msg_Sptt_Hup);
			rsp->header.mclass = APP_SPTT_HUP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			memcpy(rsp->Reserve, req->Reserve, sizeof(rsp->Reserve));
			satfi_log("SPTT_HUP %.21s %d %d\n",req->MsID, rsp->header.length, req->header.length);
			
			n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;

		case MO_VOICE:
		{
			char Reserve[9] = {0};
			MsgMoVoice *req = (MsgMoVoice *)pack;
			int datalen = req->header.length - ((int)req->data - (int)req);
			memcpy(Reserve, &pack[req->header.length - 8], 8);

			USER *pUser = gp_users;
			int res = 0;
			while(pUser)
			{
				if(strncmp(&pUser->userid[USERID_LLEN - 8], Reserve, 8) == 0)
				{
					if(pUser->socketfd < 0)
					{
						break;
					}
					
					MsgMtVoice *rsp = (MsgMtVoice *)tmp;
					rsp->header.length = sizeof(MsgMtVoice) + datalen - 8;
					rsp->header.mclass = MT_VOICE;
					memcpy(rsp->MsID, pUser->userid, USERID_LLEN);
					memcpy(rsp->data, req->data, datalen - 8);
					
					satfi_log("IN_MO_VOICE %d %04x %d %s %.21s", rsp->header.length, rsp->header.mclass, datalen, Reserve, pUser->userid);
					write(pUser->socketfd, rsp, rsp->header.length);
					res = 1;
					break;
				}
				pUser = pUser->next;
			}

			if(res == 0)
			{
				Msg_Mo_Voice *rsp = (Msg_Mo_Voice*)tmp;
				rsp->header.length = sizeof(Msg_Mo_Voice) + datalen;
				rsp->header.mclass = APP_MO_VOICE;
				StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
				if(datalen > 0)memcpy(rsp->data, req->data, datalen);
				satfi_log("OUT_MO_VOICE %.21s chatMsid=%.21s Reserve=%s length=%d\n",req->MsID, chatMsid, Reserve, rsp->header.length);

				if(strlen(Reserve) == 0)
				{
					if(strncmp(chatMsid, req->MsID, 21) == 0)
					{
						req->header.mclass = MT_VOICE;
						Data_Transmit(req->MsID, req);
						
						struct sockaddr_in tsc_addr;
						bzero(&tsc_addr, sizeof(tsc_addr));
						tsc_addr.sin_family = AF_INET;
						tsc_addr.sin_port = htons(UDP_VOICE_DSTPORT);
						tsc_addr.sin_addr.s_addr =	inet_addr(base.tsc.tsc_addr);
						/*APP_MO_VOICE*/
						sendto(sock_udp, tmp, rsp->header.length, 0, (struct sockaddr*)&tsc_addr, sizeof(tsc_addr));
					}
				}
				else
				{
					//satfi_log("OUT_MO_VOICE %.21s chatMsid=%.21s Reserve=%s\n",req->MsID, chatMsid, Reserve);
					struct sockaddr_in tsc_addr;
					bzero(&tsc_addr, sizeof(tsc_addr));
					tsc_addr.sin_family = AF_INET;
					tsc_addr.sin_port = htons(UDP_VOICE_DSTPORT);
					tsc_addr.sin_addr.s_addr =	inet_addr(base.tsc.tsc_addr);
					/*APP_MO_VOICE*/
					sendto(sock_udp, tmp, rsp->header.length, 0, (struct sockaddr*)&tsc_addr, sizeof(tsc_addr));
				}

				if(bTscConnected == 1)
				{
					//网络连接时，计数语音包数，用于计费
					USER *pUser = gp_users;
					while(pUser)
					{
						if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
						{
							++pUser->udp_voice_packnum;
							//satfi_log("%s udp_voice_packnum=%d rsp->header.length=%d", pUser->userid, pUser->udp_voice_packnum, rsp->header.length);
							//udp_voice_calc_money(req->MsID, );
						}
						pUser = pUser->next;
					}
				}
			}
		}
		break;

		case HUPIN_RSP:
		{
			MsgHupinRsp *req = (MsgHupinRsp *)pack;

			Msg_Hupin_Rsp *rsp = (Msg_Hupin_Rsp*)tmp;
			rsp->header.length = sizeof(Msg_Hupin_Rsp);
			rsp->header.mclass = APP_HUPIN_RSP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Result = req->Result;
			
			satfi_log("HUPIN_RSP %.21s %d %d\n",req->MsID, rsp->header.length, req->header.length);
			
			n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;
				
		case MS_HUP:
		{
			MsgMsHup *req = (MsgMsHup *)pack;

			Msg_Ms_Hup *rsp = (Msg_Ms_Hup*)tmp;
			rsp->header.length = sizeof(Msg_Ms_Hup);
			rsp->header.mclass = APP_MS_HUP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			memcpy(rsp->Src_Name, req->Src_Name, sizeof(req->Src_Name));
			rsp->Operation = req->Operation;
			
			satfi_log("MS_HUP %.21s %.21s %.21s\n",req->MsID, req->MsID, req->Src_Name);
			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;

		case CLR_SPTT:
		{
			MsgClrSptt *req = (MsgClrSptt *)pack;

			Msg_Clr_Sptt *rsp = (Msg_Clr_Sptt*)tmp;
			rsp->header.length = sizeof(Msg_Clr_Sptt);
			rsp->header.mclass = APP_CLR_SPTT;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			satfi_log("CLR_SPTT %.21s\n",req->MsID);
			n = Data_To_Tsc(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;

		case ASPTT_REQ:
		{
			MsgAsptt *req = (MsgAsptt *)pack;
			
			USER *pUser = gp_users;
			int res = 0;
			while(pUser)
			{
				if(strncmp(pUser->userid, req->TARGET_MS_Id, USERID_LLEN) == 0)
				{
					if(pUser->socketfd < 0)
					{
						break;
					}
					satfi_log("IN_ASPTT_REQ %.21s %.21s\n",req->MsID, req->TARGET_MS_Id);
					write(pUser->socketfd, req, req->header.length);
					res = 1;
					break;
				}
				pUser = pUser->next;
			}

			if(res == 0)
			{
				Msg_Asptt *rsp = (Msg_Asptt*)tmp;
				rsp->header.length = sizeof(Msg_Asptt);
				rsp->header.mclass = APP_ASPTT_REQ;
				StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
				StrToBcd(rsp->TARGET_MS_Id, &(req->TARGET_MS_Id[USERID_LLEN - USERID_LEN]), USERID_LEN);
				memcpy(rsp->Reserve, req->Reserve, sizeof(rsp->Reserve));
				
				satfi_log("OUT_ASPTT_REQ %.21s %.21s\n",req->MsID, req->TARGET_MS_Id);
				n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
				if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);

			}
		}
		break;
		
		case ASPTT_REQ_RSP:
		{
			MsgAspttRsp *req = (MsgAspttRsp *)pack;

			USER *pUser = gp_users;
			int res = 0;
			while(pUser)
			{
				if(strncmp(pUser->userid, req->TARGET_MS_Id, USERID_LLEN) == 0)
				{
					if(pUser->socketfd < 0)
					{
						break;
					}

					pUser = gp_users;
					while(pUser)
					{
						if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
						{
							if(pUser->socketfd < 0)
							{
								break;
							}
							satfi_log("IN_ASPTT_REQ_RSP %.21s %.21s %d\n",req->MsID, req->TARGET_MS_Id, req->Result);
							write(pUser->socketfd, req, req->header.length);	
							res = 1;
							break;
						}
						pUser = pUser->next;
					}
					break;
				}
				pUser = pUser->next;
			}

			if(res == 0)
			{
				Msg_Asptt_Rsp *rsp = (Msg_Asptt_Rsp*)tmp;
				rsp->header.length = sizeof(Msg_Asptt_Rsp);
				rsp->header.mclass = APP_ASPTT_REQ_RSP;
				StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
				StrToBcd(rsp->TARGET_MS_Id, &(req->TARGET_MS_Id[USERID_LLEN - USERID_LEN]), USERID_LEN);
				rsp->Result = req->Result;
				
				satfi_log("OUT_ASPTT_REQ_RSP %.21s %.21s\n",req->MsID, req->TARGET_MS_Id);
				n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
				if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
			}
		}
		break;
				
		case ASPTT_HUP:
		{
			MsgAspttHup *req = (MsgAspttHup *)pack;
		
			USER *pUser = gp_users;
			int res = 0;
			while(pUser)
			{
				if(strncmp(&pUser->userid[USERID_LLEN - 8], req->Reserve, 8) == 0)
				{
					if(pUser->socketfd < 0)
					{
						break;
					}

					satfi_log("IN_ASPTT_HUP %.21s %.21s", req->MsID, pUser->userid);
					res = 1;
					break;
				}
				pUser = pUser->next;
			}

			if(res == 0)
			{
				Msg_Asptt_Hup *rsp = (Msg_Asptt_Hup*)tmp;
				rsp->header.length = sizeof(Msg_Asptt_Hup);
				rsp->header.mclass = APP_ASPTT_HUP;
				StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
				memcpy(rsp->Reserve, req->Reserve, sizeof(rsp->Reserve));
				
				satfi_log("OUT_ASPTT_HUP MsID=%.21s Reserve=%.8s\n", req->MsID, req->Reserve);
				n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
				if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);

				if(strncmp(req->MsID, chatMsid, 21) == 0)
				{
					chat = 0;
					bzero(chatMsid, 21);
					MsgHupin *req1 = (MsgHupin *)tmp;
					req1->header.length = sizeof(MsgHupin);
					req1->header.mclass = HUPIN;
					//Data_To_ExceptMsID(req->MsID, req1);
					Data_Transmit(req->MsID, req1);
				}
			}
		}
		break;
		
		case ASPTT_HUP_RSP:
		{
			MsgAspttHupRsp *req = (MsgAspttHupRsp *)pack;
		
			Msg_Asptt_Hup_Rsp *rsp = (Msg_Asptt_Hup_Rsp*)tmp;
			rsp->header.length = sizeof(Msg_Asptt_Hup_Rsp);
			rsp->header.mclass = APP_ASPTT_HUP_RSP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Result = req->Result;
			
			satfi_log("ASPTT_HUP_RSP %.21s\n", req->MsID);
			n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;
						
		case ASPTT_CLER:
		{
			MsgAspttCler *req = (MsgAspttCler *)pack;

			USER *pUser = gp_users;
			int res = 0;
			while(pUser)
			{
				if(strncmp(pUser->userid, req->TARGET_MS_Id, USERID_LLEN) == 0)
				{
					if(pUser->socketfd < 0)
					{
						break;
					}
					satfi_log("IN_ASPTT_CLER %.21s\n", req->MsID);
					write(pUser->socketfd, req, req->header.length);
					res = 1;
					break;
				}
				pUser = pUser->next;
			}

			if(res == 0)
			{
				Msg_Asptt_Cler *rsp = (Msg_Asptt_Cler*)tmp;
				rsp->header.length = sizeof(Msg_Asptt_Cler);
				rsp->header.mclass = APP_ASPTT_CLER;
				StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
				StrToBcd(rsp->TARGET_MS_Id, &(req->TARGET_MS_Id[USERID_LLEN - USERID_LEN]), USERID_LEN);
				memcpy(rsp->Reserve, req->Reserve, sizeof(rsp->Reserve));
				
				satfi_log("OUT_ASPTT_CLER %.21s\n", req->MsID);
				n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
				if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
			}
		}
		break;
		
		case ASPTT_CLER_RSP:
		{
			MsgAspttClerRsp *req = (MsgAspttClerRsp *)pack;
		
			Msg_Asptt_Cler_Rsp *rsp = (Msg_Asptt_Cler_Rsp*)tmp;
			rsp->header.length = sizeof(Msg_Asptt_Cler_Rsp);
			rsp->header.mclass = APP_ASPTT_CLER_RSP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Result = req->Result;
			
			satfi_log("ASPTT_CLER_RSP %.21s\n", req->MsID);
			n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;
								
		case ASPTT_CANCEL:
		{
			MsgAspttCancel *req = (MsgAspttCancel *)pack;

			USER *pUser = gp_users;
			int res = 0;
			while(pUser)
			{
				if(strncmp(pUser->userid, req->TARGET_MS_Id, USERID_LLEN) == 0)
				{
					if(pUser->socketfd < 0)
					{
						break;
					}
					satfi_log("IN_ASPTT_CANCEL %.21s\n", req->MsID);
					write(pUser->socketfd, req, req->header.length);
					res = 1;
					break;
				}
				pUser = pUser->next;
			}

			if(res == 0)
			{
				Msg_Asptt_Cancel *rsp = (Msg_Asptt_Cancel*)tmp;
				rsp->header.length = sizeof(Msg_Asptt_Cancel);
				rsp->header.mclass = APP_ASPTT_CANCEL;
				StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
				StrToBcd(rsp->TARGET_MS_Id, &(req->TARGET_MS_Id[USERID_LLEN - USERID_LEN]), USERID_LEN);
				memcpy(rsp->Reserve, req->Reserve, sizeof(rsp->Reserve));
				
				satfi_log("OUT_ASPTT_CANCEL %.21s\n", req->MsID);
				n = Data_To_Tsc_No_Rsp(tmp, rsp->header.length);
				if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
			}
		}
		break;

		case SATFI_PTT_IN:
		{
			MsgPttIn *req = (MsgPttIn *)pack;			
			Data_To_MsID(req->ToMsID, (Header *)pack);
		}
		break;
		
		case SATFI_PTT_HUPIN:
		{
			MsgHupin *req = (MsgHupin *)pack;
			Data_To_MsID(req->Dest_MsID, (Header *)pack);
		}
		break;

		case SEND_MESSAGE:
		{
			char outdata[1024]={0};
			MsgSendMsg *req = (MsgSendMsg *)pack;
			int datalen = req->header.length - ((int)req->messagedata - (int)req);
			req->messagedata[datalen] = 0;
			satfi_log("SEND_MESSAGE %.21s %.21s ID=%d %s\n", req->MsID, req->phonenum, req->ID, req->messagedata);
			CreateMessage(req->MsID, req->phonenum, req->ID, req->messagedata);

#if 0
			MsgSendMsgRsp rsp;
			rsp.header.length = sizeof(MsgSendMsgRsp);
			rsp.header.mclass = SEND_MESSAGE_RESP;
			strncpy(rsp.MsID, req->MsID, 21);
			rsp.Result = 1;//短信发送成功
			rsp.ID = req->ID;
			write(socket, &rsp, rsp.header.length);
			
			unsigned char Decode[1024] = "看喀喀喀abc";
			char OAphonenum[21] = "13112121509";	//发送方地址（手机号码）
			
			char tmp[1024]={0};
			MsgGetMessageRsp *rsp1 = (MsgGetMessageRsp *)tmp;
			rsp1->header.length = sizeof(MsgGetMessageRsp)+strlen(Decode)+1;
			rsp1->header.mclass = RECV_MESSAGE;
			strncpy(rsp1->MsID, req->MsID, 21);
			rsp1->Result = 1;
			strncpy(rsp1->SrcMsID, OAphonenum, 21);
			rsp1->Type = 3;
			rsp1->ID = time(0);
			unsigned long long t = (unsigned long long)rsp1->ID*1000;
			memcpy(rsp1->Date, &t, sizeof(rsp1->Date));
			memcpy(rsp1->message, Decode, strlen(Decode)+1);
			satfi_log("RECV_MESSAGE %d %x %s %d\n",rsp1->ID, *(rsp1->Date), rsp1->message, rsp1->header.length);
			
			write(socket, rsp1, rsp1->header.length);
#endif
		}
		break;

		case MODIFY_FAMILY_PHONE:
		{
			MsgModifyPamilyPhone *req = (MsgModifyPamilyPhone *)pack;
			USER *pUser = gp_users;
			int i=0;
			while(pUser)
			{
				if(strncmp(pUser->userid, req->MsID, 21) == 0)
				{
					for(i=0;i<pUser->famNumConut;i++)
					{
						if(strstr(pUser->FamiliarityNumber[i], req->oldPhone))
						{
							strncpy(pUser->FamiliarityNumber[i], req->newPhone, USERID_LLEN);
							satfi_log("newPhone=%s oldPhone=%s", pUser->FamiliarityNumber[i], req->oldPhone);
							break;
						}
					}
					break;
				}
				pUser = pUser->next;
			}
		}
		break;
		
		case ADD_FAMILY_PHONE:
		{
			MsgAddPamilyPhone *req = (MsgAddPamilyPhone *)pack;
			USER *pUser = gp_users;
			int i=0;
			while(pUser)
			{
				if(strncmp(pUser->userid, req->MsID, 21) == 0)
				{
					for(i=0;i<pUser->famNumConut;i++)
					{
						if(strlen(pUser->FamiliarityNumber[i]) == 0)
						{
							strncpy(pUser->FamiliarityNumber[i], req->Phone, USERID_LLEN);
							satfi_log("1famNumConut=%d, addPhone=%s", pUser->famNumConut, req->Phone);
							break;
						}
					}

					if(i == pUser->famNumConut && i<sizeof(pUser->FamiliarityNumber)/sizeof(pUser->FamiliarityNumber[0]))
					{
						strncpy(pUser->FamiliarityNumber[pUser->famNumConut], req->Phone, USERID_LLEN);
						pUser->famNumConut++;							
						satfi_log("2famNumConut=%d, addPhone=%s", pUser->famNumConut, req->Phone);
					}
					
					break;
				}
				pUser = pUser->next;
			}
		}
		break;
		
		case REMOVE_FAMILY_PHONE:
		{
			MsgRemovePamilyPhone *req = (MsgRemovePamilyPhone *)pack;
			USER *pUser = gp_users;
			int i=0;
			while(pUser)
			{
				if(strncmp(pUser->userid, req->MsID, 21) == 0)
				{
					for(i=0;i<pUser->famNumConut;i++)
					{
						if(strstr(pUser->FamiliarityNumber[i], req->Phone))
						{
							bzero(pUser->FamiliarityNumber[i], USERID_LLEN);
							satfi_log("removePhone=%s %d", req->Phone, strlen(pUser->FamiliarityNumber[i]));
							break;
						}
					}
					break;
				}
				pUser = pUser->next;
			}
		}
		break;
		
		default:
			satfi_log("received unrecognized message from app : mclass=0X%04X length=%d\n", p->mclass, p->length);
			return -1;
	}
	
	return 0;
}

static void remove_user(int socketfd)
{
	USER *pUser = gp_users;
	USER *q = pUser;
	
	while(pUser)
	{
		//satfi_log("%d %d\n",pUser->socketfd, socketfd);
		if(pUser->socketfd == socketfd)
		{
			satfi_log("find and remove_user socketfd=%d %d %.21s\n",socketfd,base.sat.captain_socket,pUser->userid);
			pUser->socketfd = -1;

			if(strncmp(pUser->userid,"000000000000000000000",21) == 0)
			{
				satfi_log("%s quit_debug_mode\n",pUser->userid);
			}

			break;
		}
		pUser = pUser->next;
	}
	
	if(base.sat.sat_calling)
	{
		if(base.sat.socket == socketfd)
		{
			//base.sat.sat_state_phone = SAT_STATE_PHONE_ATH_W;
		}
	}
	
	if(base.sat.captain_socket == socketfd)
	{
		base.sat.captain_socket = -1;
	}
}

#define RECV_BUF_SIZE	1024*24
int App_Add(fd_set *set, int AppSocketFd, char *ip)
{
	APPSOCKET *pApp = gp_appsocket;

	while(pApp)
	{
		if(strcmp(pApp->ip, ip) == 0)
		{
			satfi_log("pApp->ip = %s exist AppSocketFd %d %d", pApp->ip, AppSocketFd, pApp->AppSocketFd);
			FD_CLR(pApp->AppSocketFd, set);
			close(pApp->AppSocketFd);
			
			FD_SET(AppSocketFd, set); 
			pApp->AppSocketFd = AppSocketFd;
			pApp->Update = time(0);
			pApp->DataSaveOffset = 0;
			return 0;
		}
		pApp = pApp->next;
	}

	if(pApp == NULL)
	{
		pApp = (APPSOCKET *)malloc(sizeof(APPSOCKET));
		if(pApp)
		{
			//satfi_log("malloc %s %d\n",ip, AppSocketFd);
			bzero(pApp,sizeof(APPSOCKET));
			pApp->AppSocketFd = AppSocketFd;
			pApp->Update = time(0);
			pApp->DataSaveOffset = 0;
			bzero(pApp->ip, sizeof(pApp->ip));
			strcpy(pApp->ip, ip);
			pApp->next = gp_appsocket;
			gp_appsocket = pApp;

			FD_SET(pApp->AppSocketFd, set); 
		}
		else
		{
			satfi_log("malloc error for APPSOCKET\n");
			return -1;
		}
	}
	
	return 0;
}


void App_Remove(int AppSocketFd)
{
	APPSOCKET *pApp = gp_appsocket;
	APPSOCKET *q = pApp;
	
	while(pApp)
	{
		if(pApp->AppSocketFd == AppSocketFd)
		{
			if(pApp == gp_appsocket)
			{
				//satfi_log("App_Remove1 %d\n",AppSocketFd);
				gp_appsocket = pApp->next;
				free(pApp);
			}
			else
			{
				//satfi_log("App_Remove2 %d\n",AppSocketFd);
				q->next = pApp->next;
				free(pApp);
			}
			break;
		}
		q = pApp;
		pApp = pApp->next;
	}
}

int App_Fd_Set(fd_set *set, int TimeOut, int *maxfd)
{
	APPSOCKET *pApp = gp_appsocket;
	APPSOCKET *q = pApp;
	time_t now = time(0);
	while(pApp)
	{
		if(now - pApp->Update < TimeOut)
		{
			if(pApp->AppSocketFd > *maxfd)
			{
				*maxfd = pApp->AppSocketFd;
			}
			q = pApp;
			pApp = pApp->next;
		}
		else
		{
			if(pApp == gp_appsocket)
			{
				satfi_log("TimeOut1 FD_CLR %d %d\n",pApp->AppSocketFd, now - pApp->Update);
				gp_appsocket = pApp->next;
				FD_CLR(pApp->AppSocketFd, set);
				close(pApp->AppSocketFd);
				remove_user(pApp->AppSocketFd);
				updateMSList();
				free(pApp);
				pApp = gp_appsocket;
				q = pApp;
			}
			else
			{
				satfi_log("TimeOut2 FD_CLR %d %d\n",pApp->AppSocketFd, now - pApp->Update);
				q->next = pApp->next;
				FD_CLR(pApp->AppSocketFd, set);
				close(pApp->AppSocketFd);
				remove_user(pApp->AppSocketFd);
				updateMSList();
				free(pApp);
				pApp = q->next;
			}
		}

	}

	return 0;
}

static void *select_app(void *p)
{
	BASE *base = (BASE *)p;
	fd_set fds;
	fd_set fdread;
	struct timeval timeout;
	int max_fd = -1;
	int ret = -1;
	int new_conn_fd = -1;
	struct sockaddr_in cli_addr;  
	int len = sizeof(struct sockaddr_in);
	Header *pHeader = NULL;
	char tscbuf[8192] = {0};
    int sock_app_tcp = appsocket_init(base->app.app_port);

	FD_ZERO(&fds);
	FD_ZERO(&fdread);

	FD_SET(sock_app_tcp, &fdread);
	max_fd = sock_app_tcp;
	
	while(1)
	{
		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;
		max_fd = sock_app_tcp;
		App_Fd_Set(&fdread, base->app.app_timeout, &max_fd);
		fds = fdread;
		ret = select(max_fd + 1, &fds, NULL, NULL, &timeout);  
        if (ret == 0)  
        {
            //printf("app select timeout\n");  
        }
        else if (ret < 0)  
        {  
            satfi_log("app error occur\n"); 
			//sleep(1);
        }
		else
		{
			if(FD_ISSET(sock_app_tcp, &fds))
			{
				new_conn_fd = accept(sock_app_tcp, (struct sockaddr*)&cli_addr, &len);
				satfi_log("new client comes %d %s:%d\n", new_conn_fd, inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));
				if(new_conn_fd > 0)
				{
					App_Add(&fdread, new_conn_fd, (char *)inet_ntoa(cli_addr.sin_addr));
				}
				else
				{
					satfi_log("Fail to accept");
					exit(0);
				}
			}
			else
			{
				APPSOCKET *pApp = gp_appsocket;
				int nread;
				while(pApp)
				{
					if(FD_ISSET(pApp->AppSocketFd, &fds))
					{
						nread = read(pApp->AppSocketFd, pApp->Data + pApp->DataSaveOffset, RECV_BUF_SIZE);
						if(nread > 0)
						{
							nread += pApp->DataSaveOffset;
							pApp->DataSaveOffset = 0;
							int c=0;
							while(1)
							{
								pHeader = (Header *)&(pApp->Data[pApp->DataSaveOffset]);
								++c;if(c>10){satfi_log("app data error mclass=%x length=%d", pHeader->mclass, pHeader->length);pApp->DataSaveOffset=0;break;}
								if((sizeof(pHeader->length) + pApp->DataSaveOffset > nread) || (nread < pHeader->length + pApp->DataSaveOffset))
								{
									if(pApp->DataSaveOffset > 0)memcpy(pApp->Data, pHeader, nread - pApp->DataSaveOffset);
									pApp->DataSaveOffset = nread - pApp->DataSaveOffset;
									satfi_log("pApp->DataSaveOffset=%d\n", pApp->DataSaveOffset);
									break;
								}
								
								if(handle_app_msg_tcp(pApp->AppSocketFd, (char*)pHeader, tscbuf) == -1)
								{
									pApp->DataSaveOffset = 0;
									break;
								}
								
								pApp->DataSaveOffset += pHeader->length;
								if(pApp->DataSaveOffset == nread)
								{
									pApp->DataSaveOffset = 0;
									break;
								}
							}
							
							pApp->Update = time(0);
						}
						else
						{
							remove_user(pApp->AppSocketFd);
							satfi_log("FD_CLR %d\n",pApp->AppSocketFd);
							FD_CLR(pApp->AppSocketFd, &fdread);
						}
					}
					pApp = pApp->next;
				}
			}
		}
	}
	return NULL;
}

void get_app_message_group_from_tsc(void)
{
	USER *pUser = gp_users;
	char tmp[512];
	
	while(pUser)
	{
		if(pUser->socketfd > 0)
		{
			Msg_Get_Message_Req* getmsg = (Msg_Get_Message_Req*)tmp;
			getmsg->header.length = sizeof(Msg_Get_Message_Req);
			getmsg->header.mclass = APP_GET_MESSAGE_CMD;
			StrToBcd(getmsg->MsID, &(pUser->userid[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("get_message_from_tsc %s\n",pUser->userid);
			Data_To_Tsc(tmp, getmsg->header.length);

			Msg_Query_Group_Req* querygrp = (Msg_Query_Group_Req*)tmp;
			querygrp->header.length = sizeof(Msg_Query_Group_Req);
			querygrp->header.mclass = APP_QUERY_GROUP_CMD;
			StrToBcd(querygrp->MsID, &(pUser->userid[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("get_group_from_tsc %s\n",pUser->userid);
			Data_To_Tsc(tmp, querygrp->header.length);
		}
		pUser = pUser->next;
	}

}

void Data_To_MsID(char *MsID, void *data)
{
	int n;
	USER *pUser = gp_users;
	Header *pHeader = data;
	while(pUser)
	{
		if(strncmp(pUser->userid, MsID, USERID_LLEN) == 0)
		{
			if(pUser->socketfd < 0)
			{
				satfi_log("%.21s no exist length=%d mclass=0X%04X", pUser->userid, pHeader->length, pHeader->mclass);
				break;
			}
			
			//satfi_log("sendto app %.21s length=%d mclass=0X%04X\n", pUser->userid, pHeader->length, pHeader->mclass);
			n = write(pUser->socketfd, pHeader, pHeader->length);
			if(n < 0)
			{
				satfi_log("sendto return error: errno=%d (%s)\n", errno, strerror(errno));
				App_Remove(pUser->socketfd);
				close(pUser->socketfd);
				pUser->socketfd = -1;
				updateMSList();
			}
			break;
		}
		pUser = pUser->next;
	}	
}


void Data_To_ExceptMsID(char *MsID, void *data)
{
	int n;
	USER *pUser = gp_users;
	Header *pHeader = data;
	while(pUser)
	{
		if(strncmp(pUser->userid, MsID, USERID_LLEN) != 0)
		{
			if(pUser->socketfd > 0)
			{
				satfi_log("sendto app %.21s length=%d mclass=0X%04X %d\n", pUser->userid, pHeader->length, pHeader->mclass, pUser->socketfd);
				strncpy(data+sizeof(Header), pUser->userid, USERID_LLEN);
				n = write(pUser->socketfd, pHeader, pHeader->length);
				if(n < 0)
				{
					satfi_log("sendto return error: errno=%d (%s)\n", errno, strerror(errno));
					App_Remove(pUser->socketfd);
					close(pUser->socketfd);
					pUser->socketfd = -1;
					updateMSList();
				}
			}		
		}
		pUser = pUser->next;
	}	
}

//同群组数据转发
void Data_Transmit(char *MsID, void *data)
{
	int n;
	int i,j;
	USER *pUser = gp_users;
	USER *pTmp;
	Header *pHeader = data;

	while(pUser)
	{
		if(strncmp(pUser->userid, MsID, USERID_LLEN) == 0)
		{
			pTmp = pUser;
			for(i=0; i<pTmp->grpCount; i++)
			{
				if(strncmp(pTmp->grpinfo[i].GrpID, pTmp->defgrp, USERID_LLEN) == 0)
				{
					//satfi_log("%.21s %.21s\n", pTmp->grpinfo[i].GrpID, pUser->userid);
					break;
				}
			}
			break;
		}
		pUser = pUser->next;
	}

	if(pUser == NULL)
	{
		return;
	}

	pUser = gp_users;
	while(pUser)
	{
		if(strncmp(pUser->userid, MsID, USERID_LLEN) != 0)
		{
			if(pUser->socketfd > 0)
			{
				for(j=0; j<pTmp->grpinfo[i].Count; j++)
				{
					//satfi_log("%.21s %.21s\n", pTmp->grpinfo[i].msid[j], pUser->userid);
					if(strncmp(pTmp->grpinfo[i].msid[j], pUser->userid, USERID_LLEN) == 0)
					{
						satfi_log("%.21s ==> %.21s length=%d mclass=0X%04X %d\n", MsID, pUser->userid, pHeader->length, pHeader->mclass, pUser->socketfd);
						n = write(pUser->socketfd, pHeader, pHeader->length);
						if(n < 0)
						{
							satfi_log("sendto return error: errno=%d (%s)\n", errno, strerror(errno));
							App_Remove(pUser->socketfd);
							close(pUser->socketfd);
							pUser->socketfd = -1;
							updateMSList();
						}					
						break;
					}
				}
			}		
		}
		pUser = pUser->next;
	}	
}


int handle_tsc_msg(char *DataFromTsc, char *DataToApp)
{
	int ret = 0;
	Header *pHeader = (Header *)DataFromTsc;
	Header *tmp = (Header *)DataToApp;
	
	switch(pHeader->mclass)
	{
		case SATFI_CONNECT_CMD_RSP:
		{
			satfi_log("from tsc connect rsp\n");
			Msg_Connect_Rsp *rsp = (Msg_Connect_Rsp *)pHeader;
			strncpy(FixMsID, rsp->FixMsID, 9);
			bTscConnected = 1;
			if(sizeof(Msg_Connect_Rsp) <= pHeader->length)
			{
				bSatNetWorkActive = rsp->Flag;
			}

			satfi_log("bSatNetWorkActive=%d %d\n",bSatNetWorkActive, pHeader->length);
			
			get_app_message_group_from_tsc();
			ReadGpsDataAndSendToTSC(GPS_DATA_FILE);
			updateMSList();
		}
		break;
		
		case SATFI_HEART_BEAT_CMD_RSP:
		{
			base.tsc.tsc_hb_rsp_ltime = time(0);
			satfi_log("from tsc heartbeat rsp %d\n", base.tsc.tsc_hb_rsp_ltime - base.tsc.tsc_hb_req_ltime);

			ReadCallRecordsAndSendToTSC(CALL_RECORDS_FILE);
			int size = DelGpsData(GPS_DATA_FILE);
			if(size > 0)
			{
				ReadGpsDataAndSendToTSC(GPS_DATA_FILE);
			}
		}
		break;
		
		case SATFI_HEART_BEAT_SP_CMD_RSP:
		{
			base.tsc.tsc_hb_rsp_ltime = time(0);
			satfi_log("from tsc heartbeatSP rsp\n");
			int size = DelGpsData(GPS_DATA_FILE);
			if(size > 0)
			{
				ReadGpsDataAndSendToTSC(GPS_DATA_FILE);
			}
		}
		break;
		
		case SATFI_MS_LIST_CMD_RSP:
		{
			satfi_log("from tsc SATFI_MS_LIST_CMD rsp\n");
			base.tsc.tsc_hb_rsp_ltime = time(0);
			bTscConnected = 1;
			//int size = DelGpsData(GPS_DATA_FILE);
			//if(size > 0)
			//{
			//	ReadGpsDataAndSendToTSC(GPS_DATA_FILE);
			//}
		}
		break;
		
		case GS_CJB_DATA:
		{
			Msg_Gs_Cjb_Data *rsp = (Msg_Gs_Cjb_Data *)pHeader;
			
			satfi_log("GS_CJB_DATA Type=0x%02x\n",rsp->Type);

			if(rsp->Type == 0x01)
			{
				Msg_Gs_Cjb_Gps_Rsp * req = (Msg_Gs_Cjb_Gps_Rsp *)tmp;
				req->header.length = sizeof(Msg_Gs_Cjb_Gps_Rsp);
				req->header.mclass = GS_CJB_DATA_RSP;
				memcpy(req->Sat_IMSI, rsp->Sat_IMSI, 15);
				req->Type = rsp->Type;
				req->Lg = base.gps.Lg;
				req->Lg_D = base.gps.Lg_D;
				req->Lt = base.gps.Lt;
				req->Lt_D = base.gps.Lt_D;

				satfi_log("Lg=%d Lg_D=%c Lt=%d Lt_D=%c\n",req->Lg, req->Lg_D, req->Lt, req->Lt_D);
			}				
		}
		break;
		
		case APP_QUERY_GROUP_CMD_RSP:
		{
			Msg_Query_Group_Rsp *rsp = (Msg_Query_Group_Rsp *)pHeader;
			
			MsgQueryGroupRsp * req = (MsgQueryGroupRsp *)tmp;
			req->header.length = sizeof(MsgQueryGroupRsp) + rsp->Count * sizeof(MsgGroup);
			req->header.mclass = QUERY_GROUP_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Count = rsp->Count;
			
			USER *pUser = gp_users;
			while(pUser)
			{
				if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
				{
					pUser->grpCount = rsp->Count;
					//satfi_log("pUser->grpCount=%d", pUser->grpCount);
					break;
				}
				pUser = pUser->next;
			}	

			int cnt;
			for(cnt=0;cnt<rsp->Count;cnt++)
			{
				BcdToStr(req->msggroup[cnt].GrpID, rsp->group[cnt].GrpID, 11);
				memcpy(req->msggroup[cnt].GrpName,rsp->group[cnt].GrpName, 30);
				req->msggroup[cnt].Config = rsp->group[cnt].Config;
				req->msggroup[cnt].Creator = rsp->group[cnt].Creator;
				req->msggroup[cnt].IsGrpChat = rsp->group[cnt].IsGrpChat;

				if(pUser)
				{
					memcpy(pUser->grpinfo[cnt].GrpID, req->msggroup[cnt].GrpID, 21);
					//satfi_log("[%d].GrpID=%.21s", cnt, pUser->grpinfo[cnt].GrpID);
				}	
			}

			Data_To_MsID(req->MsID, tmp);
		}
		break;
		
		case APP_QUERY_MS_CMD_RSP:
		{						
			Msg_QueryMs_Rsp *rsp = (Msg_QueryMs_Rsp *)pHeader;
			
			MsgQueryMsRsp * req = (MsgQueryMsRsp *)tmp;
			req->header.length = sizeof(MsgQueryMsRsp) + rsp->Count * sizeof(MsgTerminal);
			req->header.mclass = QUERY_MS_RSP;					
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(req->GrpID,rsp->GrpID,11);
			req->Count = rsp->Count;
			
			int cnt;
			int i=0;
			USER *pUser = gp_users;
			while(pUser)
			{
				if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
				{
					for(cnt=0;cnt<pUser->grpCount;cnt++)
					{
						if(strncmp(pUser->grpinfo[cnt].GrpID, req->GrpID, USERID_LLEN) == 0)
						{
							i=cnt;
							pUser->grpinfo[cnt].Count = rsp->Count;
							//satfi_log("GrpID=%.21s rsp->Count=%d", pUser->grpinfo[cnt].GrpID, pUser->grpinfo[cnt].Count);
							break;
						}
					}
					break;
				}
				pUser = pUser->next;
			}

			for(cnt=0;cnt<rsp->Count;cnt++)
			{
				memcpy(req->msgterminal[cnt].MsID, FixMsID, 9);
				BcdToStr(&(req->msgterminal[cnt].MsID[9]), rsp->terminal[cnt].MsID, 6);
				memcpy(req->msgterminal[cnt].MsName,rsp->terminal[cnt].MsName, 30);
				req->msgterminal[cnt].Config = rsp->terminal[cnt].Config;
				req->msgterminal[cnt].Type = rsp->terminal[cnt].Type;
				req->msgterminal[cnt].OnlineStatus = rsp->terminal[cnt].OnlineStatus;

				if(pUser)
				{
					memcpy(pUser->grpinfo[i].msid[cnt], req->msgterminal[cnt].MsID, 21);
					//satfi_log("[%d].%.21s", cnt, pUser->grpinfo[i].msid[cnt]);
				}
			}

			Data_To_MsID(req->MsID, tmp);
		}
		break;
		
		case APP_SET_BLOCK_CMD_RSP:
		{
			Msg_SetBlock_Rsp *rsp = (Msg_SetBlock_Rsp *)pHeader;
			
			MsgSetBlockRsp * req = (MsgSetBlockRsp *)tmp;
			req->header.length = sizeof(MsgSetBlockRsp);
			req->header.mclass = SET_BLOCK_RSP; 				
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(req->GrpID,rsp->GrpID,11);
			req->Result = rsp->Result;

			Data_To_MsID(req->MsID, tmp);
		}
		break;						
		
		case APP_UPLOAD_MESSAGE_CMD_RSP:
		{
			Msg_Upload_Message_Rsp *rsp = (Msg_Upload_Message_Rsp *)pHeader;

			MsgUploadMessageRsp * req = (MsgUploadMessageRsp *)tmp;
			req->header.length = sizeof(MsgUploadMessageRsp);
			req->header.mclass = UPLOAD_MESSAGE_RSP;					
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;
			req->Name = rsp->Name;
			
			//satfi_log("APP_UPLOAD_MESSAGE_CMD_RSP %d\n", req->ID);
			log_insert(req->MsID, tmp, req->header.length);
			Data_To_MsID(req->MsID, tmp);
			Pack_Del(req->Name);
		}
		break;
		
		case APP_UPLOAD_VOICE_CMD_RSP:
		{
			int cnt;
			Msg_Upload_Voice_Rsp *rsp = (Msg_Upload_Voice_Rsp *)pHeader;
			MsgUploadVoiceRsp * req = (MsgUploadVoiceRsp *)tmp;
			req->header.mclass = UPLOAD_VOICE_RSP;		
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;
			req->Name = rsp->Name;
			if(rsp->Result == 1)
			{
				req->header.length = sizeof(MsgUploadVoiceRsp) + rsp->packnum * sizeof(MsgPackSeq);
				req->packnum = rsp->packnum;
				for(cnt=0;cnt<rsp->packnum;cnt++)
				{
					req->msgpackseq[cnt].pack_seq_1 = rsp->packseq[cnt].pack_seq_1;
					req->msgpackseq[cnt].pack_seq_2 = rsp->packseq[cnt].pack_seq_2;
				}
			}
			else
			{	
				req->header.length = 31;
				log_insert(req->MsID, tmp, req->header.length);
			}

			//satfi_log("APP_UPLOAD_VOICE_CMD_RSP %d %d\n", req->Name, req->Result);
			Data_To_MsID(req->MsID, tmp);
			Pack_Del(req->Name);
		}
		break;
		
		case APP_UPLOAD_PICTURE_CMD_RSP:
		{
			Msg_Upload_Picture_Rsp *rsp = (Msg_Upload_Picture_Rsp *)pHeader;
			
			MsgUploadPictureRsp * req = (MsgUploadPictureRsp *)tmp;
			req->header.mclass = UPLOAD_PICTURE_RSP;		
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;
			req->Name = rsp->Name;
			if(rsp->Result == 1)
			{
				req->header.length = sizeof(MsgUploadPictureRsp) + rsp->packnum * sizeof(MsgPackSeq);
				req->packnum = rsp->packnum;
				satfi_log("Result=%d Name=%d packnum=%d",req->Result, req->Name, req->packnum);
				int cnt;
				for(cnt=0;cnt<rsp->packnum;cnt++)
				{
					req->msgpackseq[cnt].pack_seq_1 = rsp->packseq[cnt].pack_seq_1;
					req->msgpackseq[cnt].pack_seq_2 = rsp->packseq[cnt].pack_seq_2;
					satfi_log("pack_seq_1=%d, pack_seq_2=%d",rsp->packseq[cnt].pack_seq_1, rsp->packseq[cnt].pack_seq_2);
				}
				
			}
			else
			{
				satfi_log("APP_UPLOAD_PICTURE_CMD_RSP Name=%d Result=%d\n", req->Name, req->Result);
				req->header.length = 31;
				log_insert(req->MsID, tmp, req->header.length);
			}

			Data_To_MsID(req->MsID, tmp);
			Pack_Del(req->Name);
		}
		break;
		
		case APP_READ_OK_CMD_RSP:
		{
			Msg_Read_OK_Rsp *rsp = (Msg_Read_OK_Rsp *)pHeader;
			
			MsgReadOKRsp * req = (MsgReadOKRsp *)tmp;
			req->header.length = sizeof(MsgReadOKRsp);
			req->header.mclass = READ_OK_RSP;	
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->ID= rsp->ID;
			req->Result = rsp->Result;

			Data_To_MsID(req->MsID, tmp); 					
		}
		break;
		
		case APP_GET_MESSAGE_CMD_RSP:
		{
			int messagelen = 0;
			Msg_Get_Message_Rsp *rsp = (Msg_Get_Message_Rsp *)pHeader;

			MsgGetMessageRsp * req = (MsgGetMessageRsp *)tmp;
			req->header.mclass = GET_MESSAGE_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;
			if(rsp->Result == 1)
			{
				messagelen = rsp->header.length - ((int)rsp->message - (int)rsp);
				req->header.length = sizeof(MsgGetMessageRsp) + messagelen;
				memcpy(req->SrcMsID, FixMsID, 9);
				BcdToStr(&(req->SrcMsID[9]), rsp->SrcMsID, 6);
				req->Type = rsp->Type;
				req->ID = rsp->ID;
				memcpy(req->Date, rsp->Date, 8);
				if(messagelen>0)memcpy(req->message, rsp->message, messagelen);

				if(req->Type != 0)satfi_log("path=%s",req->message);

				#define MESSAGE_ID_NUM	100
				static unsigned int MessageIdNum = 0;
				static unsigned int MessageIdList[MESSAGE_ID_NUM] = {0};	//已发送消息ID

				int i;
				int isMessageSend = 0;
				for(i=0;i<MESSAGE_ID_NUM;i++)
				{
					if(MessageIdList[i] == rsp->ID)
					{
						isMessageSend = 1;
						break;
					}
				}
				
				if(isMessageSend)
				{
					break;
				}
				else
				{
					MessageIdList[MessageIdNum] = rsp->ID;
					MessageIdNum++;
					if(MessageIdNum == MESSAGE_ID_NUM)
					{
						MessageIdNum = 0;
					}
				}
			}
			else
			{
				req->header.length = 27;
			}

			satfi_log("Result=%d Type=%d ID=%x %d\n",req->Result,req->Type,req->ID,req->header.length);
			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_ZF_MESSAGE_RSP:
		{
			Msg_Zf_Message_Rsp *rsp = (Msg_Zf_Message_Rsp *)pHeader;

			MsgZfMessageRsp * req = (MsgZfMessageRsp *)tmp;
			req->header.length = sizeof(MsgZfMessageRsp);
			req->header.mclass = ZF_MESSAGE_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;
			req->ID = rsp->ID;

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;
		
		case APP_EMERGENCY_ALARM_CMD_RSP:
		{
			Msg_Emergency_Alarm_Rsp *rsp = (Msg_Emergency_Alarm_Rsp *)pHeader;

			MsgEmergencyAlarmRsp * req = (MsgEmergencyAlarmRsp *)tmp;
			req->header.length = sizeof(MsgEmergencyAlarmRsp);
			req->header.mclass = EMERGENCY_ALARM_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;
		
		case APP_CANCEL_EMERGENCY_ALARM_CMD_RSP:
		{
			Msg_Cancel_Emergency_Alarm_Rsp *rsp = (Msg_Cancel_Emergency_Alarm_Rsp *)pHeader;

			MsgCancelEmergencyAlarm_Rsp * req = (MsgCancelEmergencyAlarm_Rsp *)tmp;
			req->header.length = sizeof(MsgCancelEmergencyAlarm_Rsp);
			req->header.mclass = CANCEL_EMERGENCY_ALARM_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_OPERAT_CMD_RSP:
		{
			Msg_App_Operat_Rsp *rsp = (Msg_App_Operat_Rsp *)pHeader;
			
			MsgAppOperat_Rsp * req = (MsgAppOperat_Rsp *)tmp;
			req->header.length = sizeof(MsgCancelEmergencyAlarm_Rsp);
			req->header.mclass = OPERAT_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;
		
		case TSC_NOTIFY_RSP:
		{
			Msg_Notify_Rsp *rsp = (Msg_Notify_Rsp *)pHeader;
			
			MsgNotifyRsp * req = (MsgNotifyRsp *)tmp;
			req->header.length = sizeof(MsgNotifyRsp);
			req->header.mclass = NOTIFY_CMD;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->ID = rsp->ID;
			req->Type = rsp->Type;
			
			satfi_log("TSC_NOTIFY_RSP %.21s ID=%x Type=%d\n", req->MsID, req->ID, req->Type);
			Data_To_MsID(req->MsID, tmp); 	
		}
		break;
		
		case TSC_NO_ENOUGHMONEY_RSP:
		{
			Msg_NoEnough_Money_Rsp *rsp = (Msg_NoEnough_Money_Rsp *)pHeader;

			MsgNoEnoughMoneyRsp * req = (MsgNoEnoughMoneyRsp *)tmp;
			req->header.length = sizeof(MsgNoEnoughMoneyRsp);
			req->header.mclass = NOTENOUGH_MONEY_CMD;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Money = rsp->Money;
			
			satfi_log("NOTENOUGH_MONEY_CMD %s\n", rsp->MsID);

			if(PackHead != NULL)
			{
				Pack_Del(PackHead->Name);
			}
			
			Data_To_MsID(req->MsID, tmp); 	
		}
		break;
		
		case TSC_NOTIFY_EMERGENCY_ALARM_CMD_RSP:
		{
			Msg_Notify_Emergency_Alarm_Rsp *rsp = (Msg_Notify_Emergency_Alarm_Rsp *)pHeader;
			
			MsgNotifyEmergencyAlarm_Rsp * req = (MsgNotifyEmergencyAlarm_Rsp *)tmp;
			req->header.length = sizeof(MsgNotifyEmergencyAlarm_Rsp);
			req->header.mclass = NOTIFY_EMERGENCY_ALARM_CMD;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			memcpy(req->MsName, rsp->MsName, 30);
			req->Lg = rsp->Lg;
			req->Lt = rsp->Lt;
			req->AlarmType = rsp->AlarmType;

			USER *pUser = gp_users; 											
			while(pUser)
			{						
				if(strncmp(pUser->userid, req->MsID, USERID_LLEN) != 0 && pUser->socketfd > 0)
				{
					satfi_log("sendto app NOTIFY_EMERGENCY_ALARM_CMD %.21s\n", req->MsID);							
					int n = write(pUser->socketfd, tmp, req->header.length);
					if(n < 0)
					{
						satfi_log("sendto return error: errno=%d (%s)\n", errno, strerror(errno));
						App_Remove(pUser->socketfd);
						close(pUser->socketfd);
						pUser->socketfd = -1;
						updateMSList();
					}
					//break;
				}
				pUser = pUser->next;
			}
		}
		break;

		case TSC_NOTIFY_CANCEL_EMERGENCY_CMD_RSP:
		{
			int Remarklen;
			Msg_Notify_Cancel_Emergency_Alarm_Rsp *rsp = (Msg_Notify_Cancel_Emergency_Alarm_Rsp *)pHeader;
			Remarklen = rsp->header.length - ((int)rsp->Remark - (int)rsp);
			
			MsgNotifyCancelEmergencyAlarm_Rsp * req = (MsgNotifyCancelEmergencyAlarm_Rsp *)tmp;
			req->header.length = sizeof(MsgNotifyCancelEmergencyAlarm_Rsp) + Remarklen;
			req->header.mclass = NOTIFY_CANCEL_EMERGENCY_CMD;
			
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			
			memcpy(req->toMsID, FixMsID, 9);
			BcdToStr(&(req->toMsID[9]), rsp->toMsID, 6);

			req->Type = rsp->Type;
			memcpy(req->Remark, rsp->Remark, Remarklen);

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_PHONE_MONEY_RSP:
		case APP_MESSAGE_MONEY_RSP:
		{
			Msg_Phone_Money_Rsp *rsp = (Msg_Phone_Money_Rsp *)pHeader;

			MsgPhoneMoneyRsp * req = (MsgPhoneMoneyRsp *)tmp;
			req->header.length = sizeof(MsgPhoneMoneyRsp);
			req->header.mclass = PHONE_MONEY_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Money = rsp->Money;
			satfi_log("APP_PHONE_MONEY_RSP/APP_MESSAGE_MONEY_RSP %.21s req->Money=%d\n", req->MsID, req->Money);
			if(DelCallRecordsData(CALL_RECORDS_FILE) > 0)
			{
				ReadCallRecordsAndSendToTSC(CALL_RECORDS_FILE);
			}
			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_GRP_UPLOAD_MESSAGE_RSP:
		{
			Msg_Grp_Upload_Message_Rsp *rsp = (Msg_Grp_Upload_Message_Rsp *)pHeader;

			MsgGrpUploadMessageRsp * req = (MsgGrpUploadMessageRsp *)tmp;
			req->header.length = sizeof(MsgGrpUploadMessageRsp);
			req->header.mclass = GRP_UPLOAD_MESSAGE_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;
			req->Name = rsp->Name;

			satfi_log("APP_GRP_UPLOAD_MESSAGE_RSP %d\n", req->Name);
			log_insert(req->MsID, tmp, req->header.length);
			Data_To_MsID(req->MsID, tmp); 	
			Pack_Del(req->Name);
		}
		break;

		case APP_GRP_UPLOAD_VOICE_RSP:
		{
			Msg_Grp_Upload_Voice_Rsp *rsp = (Msg_Grp_Upload_Voice_Rsp *)pHeader;

			MsgGrpUploadVoiceRsp * req = (MsgGrpUploadVoiceRsp *)tmp;
			req->header.mclass = GRP_UPLOAD_VOICE_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;
			req->Name = rsp->Name;
			
			if(rsp->Result == 0)
			{
				req->header.length = 31;
				log_insert(req->MsID, tmp, req->header.length);
			}
			else if(rsp->Result == 1)
			{
				req->header.length = sizeof(MsgGrpUploadVoiceRsp) + rsp->packnum * sizeof(MsgPackSeq);
				req->packnum = rsp->packnum;
				int cnt;
				for(cnt=0;cnt<rsp->packnum;cnt++)
				{
					req->msgpackseq[cnt].pack_seq_1 = rsp->packseq[cnt].pack_seq_1;
					req->msgpackseq[cnt].pack_seq_2 = rsp->packseq[cnt].pack_seq_2;
				}		
			}

			satfi_log("APP_GRP_UPLOAD_VOICE_RSP %d\n", req->Name);
			Data_To_MsID(req->MsID, tmp); 
			Pack_Del(req->Name);
		}
		break;

		case APP_GRP_UPLOAD_PICTURE_RSP:
		{
			Msg_Grp_Upload_Picture_Rsp *rsp = (Msg_Grp_Upload_Picture_Rsp *)pHeader;

			MsgGrpUploadPictureRsp * req = (MsgGrpUploadPictureRsp *)tmp;
			req->header.mclass = GRP_UPLOAD_PICTURE_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;
			req->Name = rsp->Name;
			
			if(rsp->Result == 0)
			{
				req->header.length = 31;
				log_insert(req->MsID, tmp, req->header.length);
			}
			else if(rsp->Result == 1)
			{
				req->header.length = sizeof(MsgGrpUploadVoiceRsp) + rsp->packnum * sizeof(MsgPackSeq);
				req->packnum = rsp->packnum;
				int cnt;
				for(cnt=0;cnt<rsp->packnum;cnt++)
				{
					req->msgpackseq[cnt].pack_seq_1 = rsp->packseq[cnt].pack_seq_1;
					req->msgpackseq[cnt].pack_seq_2 = rsp->packseq[cnt].pack_seq_2;
				}		
			}

			Data_To_MsID(req->MsID, tmp); 						
			Pack_Del(req->Name);
		}
		break;

		case APP_GRP_READ_OK_RSP:
		{
			Msg_Grp_Read_OK_Rsp *rsp = (Msg_Grp_Read_OK_Rsp *)pHeader;

			MsgGrpReadOKRsp * req = (MsgGrpReadOKRsp *)tmp;
			req->header.length = sizeof(MsgGrpReadOKRsp);
			req->header.mclass = GRP_READ_OK_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->ID = rsp->ID;
			req->Result = rsp->Result;
			
			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_SET_GRP_CAHT_RSP:
		{
			Msg_Set_Grp_Chat_Rsp *rsp = (Msg_Set_Grp_Chat_Rsp *)pHeader;

			MsgSetGrpChatRsp * req = (MsgSetGrpChatRsp *)tmp;
			req->header.length = sizeof(MsgSetGrpChatRsp);
			req->header.mclass = SET_GRP_CHAT_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(req->GrpID, rsp->GrpID, 11);
			req->Type = rsp->Type;
			req->MsgType = rsp->MsgType;

			Data_To_MsID(req->MsID, tmp); 							
		}
		break;

		case APP_GRP_NOTIFY_CMD:
		{
			Msg_Grp_Notify_Rsp *rsp = (Msg_Grp_Notify_Rsp *)pHeader;
			int datalen = rsp->header.length - ((int)rsp->data - (int)rsp);
			MsgGrpNotifyRsp * req = (MsgGrpNotifyRsp *)tmp;
			req->header.length = sizeof(MsgGrpNotifyRsp) + datalen;
			req->header.mclass = GRP_NOTIFY_CMD;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(req->TargetGrpID, rsp->TargetGrpID, 11);
			req->Name = rsp->Name;
			req->Type = rsp->Type;
			if(datalen>0)memcpy(req->data, rsp->data, datalen);

			USER *pUser = gp_users;
			while(pUser)
			{
				if(pUser->socketfd > 0)
				{
					satfi_log("sendto app GRP_NOTIFY_CMD %.21s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], rsp->Name);
					int n = write(pUser->socketfd, tmp, req->header.length);
					if(n < 0)
					{
						satfi_log("sendto return error: errno=%d (%s)\n", errno, strerror(errno));
						App_Remove(pUser->socketfd);
						close(pUser->socketfd);
						pUser->socketfd = -1;
						updateMSList();
					}
				}
				pUser = pUser->next;
			}
		}
		break;

		case APP_SET_MSG_RSP:
		{
			Msg_Set_Msg_Rsp *rsp = (Msg_Set_Msg_Rsp *)pHeader;

			MsgSetMsgRsp* req = (MsgSetMsgRsp *)tmp;
			req->header.length = sizeof(MsgSetMsgRsp);
			req->header.mclass = SET_MSG_RSP;
			req->Result = rsp->Result;
			
			memcpy(req->MsID, FixMsID, 9);
			memcpy(req->TargetMsID, FixMsID, 9);
			
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(&(req->TargetMsID[9]), rsp->TargetMsID, 6);

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case BLIND_AREA_GPS_RSP:
		{
			Msg_Blind_Area_Gps_Rsp *rsp = (Msg_Blind_Area_Gps_Rsp *)pHeader;

			int size = DelGpsData(GPS_DATA_FILE);
			if(size > 0)
			{
				ReadGpsDataAndSendToTSC(GPS_DATA_FILE);
			}
			satfi_log("BLIND_AREA_GPS_RSP %llu leftsize=%d\n", rsp->Date, size);

			base.tsc.tsc_hb_rsp_ltime = time(0);
		}
		break;

		case APP_SHARE_GPS:
		{
			Msg_Share_Gps *rsp = (Msg_Share_Gps *)pHeader;

			MsgShareGps *req = (MsgShareGps *)tmp;
			req->header.length = sizeof(MsgShareGps);
			req->header.mclass = SHARE_GPS;

			memcpy(req->MsID, FixMsID, 9);
			memcpy(req->SrcMsID, FixMsID, 9);
			
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(&(req->SrcMsID[9]), rsp->SrcMsID, 6);
			req->Lg 	= rsp->Lg;
			req->Lg_D	= rsp->Lg_D;
			req->Lt 	= rsp->Lt;
			req->Lt_D	= rsp->Lt_D;

			//satfi_log("APP_SHARE_GPS %.21s %.21s\n",req->MsID, req->SrcMsID);					
			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_WL_COMMAND:
		{
			Msg_Wl_Command *rsp = (Msg_Wl_Command *)pHeader;
			int MsgLen = rsp->header.length - ((int)rsp->Msg - (int)rsp);

			MsgWlCommand *req = (MsgWlCommand *)tmp;
			req->header.length = sizeof(MsgWlCommand) + MsgLen;
			req->header.mclass = WL_COMMAND1;

			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			memcpy(req->Msg, rsp->Msg, MsgLen);

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_SET_ONLINE_RSP:
		{
			Msg_Set_Online_Rsp *rsp = (Msg_Set_Online_Rsp *)pHeader;

			MsgSetOnlineRsp *req = (MsgSetOnlineRsp *)tmp;
			req->header.length = sizeof(MsgSetOnlineRsp);
			req->header.mclass = SET_ONLINE_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(req->GrpID, rsp->GrpID, 11);
			req->Type = rsp->Type;
			
			Data_To_MsID(req->MsID, tmp); 	
		}
		break;
		
		case TSC_NOTIFY_MS:
		{
			Msg_Notify_Ms *rsp = (Msg_Notify_Ms *)pHeader;

			MsgNotifyMs *req = (MsgNotifyMs *)tmp;
			req->header.length = sizeof(MsgNotifyMs);
			req->header.mclass = NOTIFY_MS;
			memcpy(req->MsID, FixMsID, 9);
			memcpy(req->SrcMsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(&(req->SrcMsID[9]), rsp->SrcMsID, 6);
			BcdToStr(req->GrpID, rsp->GrpID, 11);
			req->Type = rsp->Type;
			satfi_log("TSC_NOTIFY_MS %.21s %.21s\n",req->MsID, req->SrcMsID);

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_GPS_POINT:
		{
			Msg_Gps_Point_Req *rsp = (Msg_Gps_Point_Req*)pHeader;
			int datalen = rsp->header.length - ((int)rsp->data - (int)rsp);
			
			MsgGpsPointReq *req = (MsgGpsPointReq *)tmp;
			req->header.length = sizeof(MsgGpsPointReq) + datalen;
			req->header.mclass = GPS_POINT_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Type = rsp->Type;
			req->Id = rsp->Id;
			memcpy(req->Operator, rsp->Operator, sizeof(rsp->Operator));
			memcpy(req->data, rsp->data, datalen);
			
			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_PTT_RSP:
		{
			Msg_Ptt_Rsp *rsp = (Msg_Ptt_Rsp*)pHeader;
			
			MsgPttRsp *req = (MsgPttRsp *)tmp;
			req->header.length = sizeof(MsgPttRsp);
			req->header.mclass = PTT_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_PTT_IN:
		{
			Msg_Ptt_In *rsp = (Msg_Ptt_In*)pHeader;

			MsgPttIn *req = (MsgPttIn *)tmp;
			req->header.length = sizeof(MsgPttIn);
			req->header.mclass = PTT_IN;

			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);

			BcdToStr(req->GrpID, rsp->GrpID, 11);
			
			memcpy(req->ToMsID, FixMsID, 9);
			BcdToStr(&(req->ToMsID[9]), rsp->ToMsID, 6);
			
			memcpy(req->FromMsID, FixMsID, 9);
			BcdToStr(&(req->FromMsID[9]), rsp->FromMsID, 6);
			
			memcpy(req->TSC, rsp->TSC, sizeof(req->TSC));
			memcpy(req->PTT_MS_Name, rsp->PTT_MS_Name, sizeof(req->PTT_MS_Name));
			
			Data_To_MsID(req->MsID, tmp);
		}
		break;

		case APP_HUPIN:
		{
			Msg_Hupin *rsp = (Msg_Hupin*)pHeader;

			MsgHupin *req = (MsgHupin *)tmp;
			req->header.length = sizeof(MsgHupin);
			req->header.mclass = HUPIN;

			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);

			BcdToStr(req->GrpID, rsp->GrpID, 11);
			
			memcpy(req->Dest_MsID, FixMsID, 9);
			BcdToStr(&(req->Dest_MsID[9]), rsp->Dest_MsID, 6);
									
			req->Option = rsp->Option;
									
			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_HUP_RSP:
		{
			Msg_Hup_Rsp *rsp = (Msg_Hup_Rsp*)pHeader;
			
			MsgHupRsp *req = (MsgHupRsp *)tmp;
			req->header.length = sizeof(MsgHupRsp);
			req->header.mclass = HUP_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_MT_VOICE:
		{
			Msg_Mt_Voice *rsp = (Msg_Mt_Voice*)pHeader;
			int datalen = rsp->header.length - ((int)rsp->data - (int)rsp);
			
			MsgMtVoice *req = (MsgMtVoice *)tmp;
			req->header.length = sizeof(MsgMtVoice) + datalen;
			req->header.mclass = MT_VOICE;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			memcpy(req->data, rsp->data, datalen);

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_SPTT_NOTIFY:
		{
			Msg_Sptt_Notify *rsp = (Msg_Sptt_Notify*)pHeader;
			
			MsgSpttNotify *req = (MsgSpttNotify *)tmp;
			req->header.length = sizeof(MsgSpttNotify);
			req->header.mclass = SPTT_NOTIFY;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(req->GrpID, rsp->GrpID, 11);
			req->Operation = rsp->Operation;

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_MS_HUP_RSP:
		{
			Msg_Ms_Hup_Rsp *rsp = (Msg_Ms_Hup_Rsp*)pHeader;
			
			MsgMsHupRsp *req = (MsgMsHupRsp *)tmp;
			req->header.length = sizeof(MsgMsHupRsp);
			req->header.mclass = MS_HUP_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_CLR_SPTT_RSP:
		{
			Msg_Ms_Clr_Sptt_Rsp *rsp = (Msg_Ms_Clr_Sptt_Rsp*)pHeader;

			MsgClrSpttRsp *req = (MsgClrSpttRsp *)tmp;
			req->header.length = sizeof(MsgClrSpttRsp);
			req->header.mclass = CLR_SPTT_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_ASPTT_REQ:
		{						
			Msg_Asptt *rsp = (Msg_Asptt *)pHeader;
			
			MsgAsptt *req = (MsgAsptt *)tmp;
			req->header.length = sizeof(MsgAsptt);
			req->header.mclass = ASPTT_REQ;
			
			memcpy(req->MsID, FixMsID, 9);
			memcpy(req->TARGET_MS_Id, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(&(req->TARGET_MS_Id[9]), rsp->TARGET_MS_Id, 6);
			memcpy(req->Reserve, rsp->Reserve, sizeof(req->Reserve));
			
			Data_To_MsID(req->TARGET_MS_Id, tmp); 			
		}
		break;
		
		case APP_ASPTT_REQ_RSP:
		{
			Msg_Asptt_Rsp *rsp = (Msg_Asptt_Rsp*)pHeader;
			
			MsgAspttRsp *req = (MsgAspttRsp *)tmp;	
			req->header.mclass = ASPTT_REQ_RSP;
			req->header.length = sizeof(MsgAspttRsp);
			req->Result = rsp->Result;
			
			memcpy(req->MsID, FixMsID, 9);
			memcpy(req->TARGET_MS_Id, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(&(req->TARGET_MS_Id[9]), rsp->TARGET_MS_Id, 6);
			
			Data_To_MsID(req->MsID, tmp); 	
		}
		break;
		
		case APP_ASPTT_HUP:
		{
			Msg_Asptt_Hup *rsp = (Msg_Asptt_Hup *)pHeader;
			
			MsgAspttHup *req = (MsgAspttHup *)tmp;
			req->header.length = sizeof(MsgAspttHup);
			req->header.mclass = ASPTT_HUP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			memcpy(req->Reserve, rsp->Reserve, sizeof(req->Reserve));

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;
		
		case APP_ASPTT_HUP_RSP:
		{
			Msg_Asptt_Hup_Rsp *rsp = (Msg_Asptt_Hup_Rsp*)pHeader;
			
			MsgAspttHupRsp *req = (MsgAspttHupRsp *)tmp;
			req->header.length = sizeof(MsgAspttHupRsp);
			req->header.mclass = ASPTT_HUP_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;
		
		case APP_ASPTT_CLER:
		{
			Msg_Asptt_Cler *rsp = (Msg_Asptt_Cler*)pHeader;

			MsgAspttCler *req = (MsgAspttCler *)tmp;
			req->header.length = sizeof(MsgAspttCler);
			req->header.mclass = ASPTT_CLER;
			memcpy(req->MsID, FixMsID, 9);
			memcpy(req->TARGET_MS_Id, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(&(req->TARGET_MS_Id[9]), rsp->TARGET_MS_Id, 6);
			memcpy(req->Reserve, rsp->Reserve, sizeof(req->Reserve));

			Data_To_MsID(req->TARGET_MS_Id, tmp); 	
		}
		break;
		
		case APP_ASPTT_CLER_RSP:
		{
			Msg_Asptt_Cler_Rsp *rsp = (Msg_Asptt_Cler_Rsp*)pHeader;

			MsgAspttClerRsp *req = (MsgAspttClerRsp *)tmp;
			req->header.length = sizeof(MsgAspttClerRsp);
			req->header.mclass = ASPTT_CLER_RSP;
			memcpy(req->MsID, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			req->Result = rsp->Result;

			Data_To_MsID(req->MsID, tmp); 	
		}
		break;

		case APP_ASPTT_CANCEL:
		{
			Msg_Asptt_Cancel *rsp = (Msg_Asptt_Cancel*)pHeader;

			MsgAspttCancel *req = (MsgAspttCancel *)tmp;
			req->header.length = sizeof(MsgAspttCancel);
			req->header.mclass = ASPTT_CANCEL;
			memcpy(req->MsID, FixMsID, 9);
			memcpy(req->TARGET_MS_Id, FixMsID, 9);
			BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
			BcdToStr(&(req->TARGET_MS_Id[9]), rsp->TARGET_MS_Id, 6);
			memcpy(req->Reserve, rsp->Reserve, sizeof(req->Reserve));

			Data_To_MsID(req->TARGET_MS_Id, tmp); 	
		}
		break;
		
		default:
		{
			satfi_log("received unrecognized message from tsc: %04x\n", pHeader->mclass);
			ret = -1;
			break;	
		}
	}
	
	return ret;
}

static void *select_tsc_udp(void *p)
{
	struct timeval tv = {3,0};	
	fd_set fds;
	int maxfd = 0;
	int n;
	int on = 1;
	char buf[2048] = {0};
	char tmp[2048];
	MsgHeader *pHeader = (MsgHeader *) buf;
	struct sockaddr_in cliAppAddr;	
	int len = sizeof(cliAppAddr);
	struct ifreq struIR;
	
	sock_udp = socket(AF_INET, SOCK_DGRAM, 0);
	
	if(sock_udp < 0)
	{
		satfi_log("socket : select_tsc_udp error");
		return NULL;
	}

	struct sockaddr_in server_addr;
	bzero(&server_addr, sizeof(server_addr));
	server_addr.sin_family	  	= AF_INET;
	server_addr.sin_port		= htons(UDP_VOICE_SRCPORT);
	server_addr.sin_addr.s_addr =  htonl(INADDR_ANY);

	struct sockaddr_in viasataddr;
	bzero(&viasataddr, sizeof(viasataddr));
	viasataddr.sin_family 		= AF_INET;
	viasataddr.sin_port 		= htons(8926);//用于获取viasat信号
	viasataddr.sin_addr.s_addr 	= inet_addr("192.168.100.1");

	//if(bind(sock_udp, (struct sockaddr*)&server_addr, sizeof(server_addr))<0)
	//{
	//	satfi_log("bind : select_tsc_udp");
	//	return NULL;
	//}
	
	sendto(sock_udp, "AT+CGSN\n", 8, 0, (struct sockaddr*)&viasataddr, sizeof(viasataddr));
	while(1)
	{
		FD_ZERO(&fds);/* 每次循环都需要清空 */
		FD_SET(sock_udp, &fds); /* 添加描述符 */
		maxfd = sock_udp;
		FD_SET(sock_udp, &fds);
		tv.tv_sec = 10;
				
	    switch(select(maxfd+1,&fds,NULL,NULL,&tv))
	    {
	    case -1: break;
	    case  0:
			{
				if(strlen(base.sat.sat_imei) == 0 || strlen(base.sat.sat_imsi) == 0)
				{
					//satfi_log("AT+CGSN");
					sendto(sock_udp, "AT+CGSN\n", 8, 0, (struct sockaddr*)&viasataddr, sizeof(viasataddr));
				}
				else
				{
					/*get viasat csq*/
					sendto(sock_udp, "AT+CSQ\n", 7, 0, (struct sockaddr*)&viasataddr, sizeof(viasataddr));
					sendto(sock_udp, "AT_VGPS\n", 8, 0, (struct sockaddr*)&viasataddr, sizeof(viasataddr));
					sendto(sock_udp, "AT_VZSTAT\n", 10, 0, (struct sockaddr*)&viasataddr, sizeof(viasataddr));
				}
				break;
			}
	    default:
			{
				if(FD_ISSET(sock_udp, &fds))
				{
					n = recvfrom(sock_udp, buf, 2048, 0, (struct sockaddr *)&cliAppAddr, &len);
					//satfi_log("%s:%u n=%d length=%d,mclass=0X%04x\n", inet_ntoa(cliAppAddr.sin_addr), ntohs(cliAppAddr.sin_port) ,n, pHeader->length, pHeader->mclass);
					if (n>0)
					{
						if(n == pHeader->length)
						{
							//satfi_log("recv %d bytes,length=%d,mclass=%x\n", n, pHeader->length, pHeader->mclass);
							switch(pHeader->mclass)
							{
								case APP_MT_VOICE:
								{
									Msg_Mt_Voice *rsp = (Msg_Mt_Voice*)pHeader;
									int datalen = rsp->header.length - ((int)rsp->data - (int)rsp);
									
									MsgMtVoice *req = (MsgMtVoice *)tmp;
									req->header.length = sizeof(MsgMtVoice) + datalen;
									req->header.mclass = MT_VOICE;
									memcpy(req->MsID, FixMsID, 9);
									BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
									memcpy(req->data, rsp->data, datalen);
									satfi_log("APP_MT_VOICE datalen=%d", datalen);
									Data_To_MsID(req->MsID, (Header *)tmp); 
								}
								break;

								case APP_MMT_VOICE:
								{
									int i = 0;
									Msg_Mmt_Voice *rsp = (Msg_Mmt_Voice*)pHeader;
									int datalen = rsp->header.length - sizeof(rsp->header) - sizeof(rsp->MsNum) - rsp->MsNum * sizeof(rsp->MsID);
									char *voicedata = (char *)rsp + sizeof(rsp->header) + sizeof(rsp->MsNum) + rsp->MsNum * sizeof(rsp->MsID);
									//satfi_log("APP_MMT_VOICE voicedatalen=%d MsNum=%d", datalen, rsp->MsNum);
									for(i=0; i<rsp->MsNum; i++)
									{
										MsgMtVoice *req = (MsgMtVoice *)tmp;
										req->header.length = sizeof(MsgMtVoice) + datalen;
										req->header.mclass = MT_VOICE;
										memcpy(req->MsID, FixMsID, 9);
										BcdToStr(&(req->MsID[9]), &(rsp->MsID[6*i]), 6);
										satfi_log("APP_MMT_VOICE MsID=%.21s", req->MsID);
										memcpy(req->data, voicedata, datalen);
										Data_To_MsID(req->MsID, (Header *)tmp); 
									}
								}
								break;
								
								default :
									satfi_log("udp unrecognized message from tsc: %04x\n", pHeader->mclass);
									break;
							}
						}
						else
						{
							buf[n] = 0;
							if(strncmp(buf, "+CSQ:", 5) == 0)
							{
								base.sat.sat_csq_value = atoi(&(buf[6]));
								base.sat.sat_csq_ltime = time(0);
								//satfi_log("%d\n",atoi(&(buf[6])));						
							}
							else if(strncmp(buf, "_VGPS:", 6) == 0)
							{
								char *p = NULL;
								bzero(base.sat.sat_gps, 256);
								p = strtok(&buf[7], ",");
								int i=0;
								while(p != NULL)
								{
									//satfi_log("%d _VGPS=%s", i, p);
									strcat(base.sat.sat_gps, p);
									strcat(base.sat.sat_gps, ",");
									p = strtok(NULL,",");
									//i++;
									//if(i==4)break;
								}
							}
							else if(strncmp(buf, "_VZSTAT:", 8) == 0)
							{
								char *p = NULL;
								int i=0;
								//bzero(base.gps.gps_bd, 256);
								p = strtok(&buf[9], ",");
								
								while(p != NULL)
								{
									if(i<11)
									{
										if(i > 1)
										{
											//strcat(base.gps.gps_bd, p);
											//if(i != 10)strcat(base.gps.gps_bd, ",");											
										}
										else if(i == 1)
										{
											if(atoi(p) == 0)strcat(base.sat.sat_gps, "Acquired");
											else if(atoi(p) == 1)strcat(base.sat.sat_gps, "Searching");
											else if(atoi(p) == 2)strcat(base.sat.sat_gps, "Registering");
											else if(atoi(p) == 3)strcat(base.sat.sat_gps, "Re-Registering");
											else if(atoi(p) == 4)strcat(base.sat.sat_gps, "Online");
											else if(atoi(p) == 5)strcat(base.sat.sat_gps, "Offline");
											break;
										}
									}
									
									//satfi_log("%d _VZSTAT=%s", i, p);
									p = strtok(NULL,",");
									i++;
								}
								//satfi_log("%s %d", base.gps.gps_bd, strlen(base.gps.gps_bd));
							}
							else if(strncmp(buf, "ERROR", 5) == 0)
							{
								sendto(sock_udp, "AT+CLCK=\"AB\",0,\"vsat_m2m\"\n", strlen("AT+CLCK=\"AB\",0,\"vsat_m2m\"\n"), 0, (struct sockaddr*)&viasataddr, sizeof(viasataddr));								
							}
							else if(strncmp(buf, "OK", 2) == 0)
							{
							}
							else
							{
								if(atoi(buf))
								{
									char sat_imei[16]={0};
									sprintf(sat_imei ,"80000086%07u", atoi(buf));
									if(strcmp(base.sat.sat_imei, sat_imei) != 0)
									{
										satfi_log("sat_imei=%s", base.sat.sat_imei);
										strncpy(base.sat.sat_imei, sat_imei, sizeof(sat_imei));
										SetKeyString("satellite", "SAT_IMEI", SAT_IMEI_FILE, NULL, base.sat.sat_imei);
									}

									int maxline = 1;
									char rbuf[64];
									myexec("ifconfig eth0 | grep HWaddr | cut -c 51- | sed 's/://g'", rbuf, &maxline);
									rbuf[4] = 0;
									sprintf(base.sat.sat_imsi ,"8000%07u%s", atoi(buf), rbuf);
								}
							}
						}
					}
					else if (n == 0)
					{
						satfi_log("recv a empty udp pack n=%d\n", n);
					}
					else if (n < 0)
					{
						satfi_log("udp error: errno=%d (%s) nread=%d\n", errno, strerror(errno), n);
					}
				}
			}
			break;
		}
		
	}
	
	return NULL;
}


static void *check_route(void *p)
{
	BASE *base = (BASE *)p;
	int socket = -1;
	int usbroutecanuse = 0;
	int viasatroutecanuse = 0;

	char *UsbRoudev = "usb0";
	char *UsbRndisGw = "192.168.42.129";

	char *BrlanRoudev = "br-lan";
	char *Br_LanGw = "192.168.100.1";

	char cmdUsbRouDel[128]={0};
	char cmdUsbRouAdd[128]={0};
	
	char cmdBrlanRouDel[128]={0};
	char cmdBrlanRouAdd[128]={0};

	sprintf(cmdUsbRouDel, "route del %s gw %s %s", base->tsc.tsc_addr, UsbRndisGw, UsbRoudev);
	sprintf(cmdUsbRouAdd, "route add %s gw %s %s", base->tsc.tsc_addr, UsbRndisGw, UsbRoudev);

	sprintf(cmdBrlanRouDel, "route del %s gw %s %s", base->tsc.tsc_addr, Br_LanGw, BrlanRoudev);
	sprintf(cmdBrlanRouAdd, "route add %s gw %s %s", base->tsc.tsc_addr, Br_LanGw, BrlanRoudev);
	
	while(1)
	{
		if(usbroutecanuse == 0)
		{
			if(checkroute(UsbRoudev, base->tsc.tsc_addr, 1) == 0)
			{
				socket = ConnectTSC(UsbRoudev, base->tsc.tsc_addr, base->tsc.tsc_port, NULL, 10);
				if(socket > 0)
				{
					usbroutecanuse = 1;
					int optlen = sizeof(base->tsc.mss);
					getsockopt(socket, IPPROTO_TCP, TCP_MAXSEG, &(base->tsc.mss), &optlen);
					satfi_log("usbroute base->tsc.mss=%d",base->tsc.mss);
					if(sock_tsc > 0)
					{
						satfi_log("usbroute close(sock_tsc)");
						Close_Tsc_Socket();
						seconds_sleep(5);
					}
					strcpy((char *)base->tsc.route, UsbRoudev);
					sock_tsc = socket;
					base->n3g.n3g_status = 1;
					udpvoicetimeout = 5;

					myexec(cmdUsbRouDel, NULL, NULL);
					myexec(cmdBrlanRouDel, NULL, NULL);

					myexec(cmdBrlanRouAdd, NULL, NULL);
					myexec(cmdUsbRouAdd, NULL, NULL);				
				}
				else
				{
					usbroutecanuse = 0;
					base->n3g.n3g_status = 0;
				}
			}
			else
			{
				if(checkroute(UsbRoudev, base->tsc.tsc_addr, 0) == 0)
				{
					satfi_log("%s", cmdUsbRouAdd);
					myexec(cmdUsbRouAdd, NULL, NULL);
					if(viasatroutecanuse)
					{
						myexec(cmdBrlanRouDel, NULL, NULL);
						myexec(cmdBrlanRouAdd, NULL, NULL);
					}
					usbroutecanuse = 0;
					base->n3g.n3g_status = 0;
					continue;
				}
			}
		}

		if(usbroutecanuse == 0 && viasatroutecanuse == 0)
		{
			if(checkroute(BrlanRoudev, base->tsc.tsc_addr, 1) == 0)
			{
				socket = ConnectTSC(BrlanRoudev, base->tsc.tsc_addr, base->tsc.tsc_port, NULL, 10);
				if(socket > 0)
				{
					viasatroutecanuse = 1;
					int optlen = sizeof(base->tsc.mss);
					getsockopt(socket, IPPROTO_TCP, TCP_MAXSEG, &(base->tsc.mss), &optlen);
					satfi_log("viasatroute base->tsc.mss=%d",base->tsc.mss);
					if(sock_tsc > 0)
					{
						satfi_log("viasatroute close(sock_tsc)");
						Close_Tsc_Socket();
						seconds_sleep(5);
					}
					strcpy((char *)base->tsc.route, BrlanRoudev);
					sock_tsc = socket;
					base->sat.sat_status = 1;
					udpvoicetimeout = 60;

					myexec(cmdUsbRouDel, NULL, NULL);
					myexec(cmdBrlanRouDel, NULL, NULL);				

					myexec(cmdUsbRouAdd, NULL, NULL);
					myexec(cmdBrlanRouAdd, NULL, NULL);
				}
				else
				{
					viasatroutecanuse = 0;
					base->sat.sat_status = 0;
				}
			}
			else
			{
				if(checkroute(BrlanRoudev, base->tsc.tsc_addr, 0) == 0)
				{
					satfi_log("%s", cmdBrlanRouAdd);
					myexec(cmdBrlanRouAdd, NULL, NULL);
					if(usbroutecanuse)
					{
						myexec(cmdUsbRouDel, NULL, NULL);
						myexec(cmdUsbRouAdd, NULL, NULL);
					}
					viasatroutecanuse = 0;
					base->sat.sat_status = 0;
					continue;
				}
			}
		}

		if(sock_tsc < 0)
		{
			usbroutecanuse = 0;
			viasatroutecanuse = 0;
		}
		else if(viasatroutecanuse)
		{
			base->sat.sat_status = 1;
		}
		else if(usbroutecanuse)
		{
			base->n3g.n3g_status = 1;
		}

		//satfi_log("viasatroutecanuse=%d, usbroutecanuse=%d", viasatroutecanuse, usbroutecanuse);
		seconds_sleep(10);
	}
	return 0;
}

static void *select_tsc(void *p)
{
	BASE *base = (BASE *)p;
	fd_set fds;
	char buf[8192];
	char tmp[8192];
	int maxfd = 0;
	int nread;
	int offset = 0;
	Header *pHeader;
	struct timeval timeout={3,0};
	int time1=0,time2=0;

	while(1)
	{
		if(sock_tsc < 0)
		{
			//satfi_log("sat_available=%d n3g_status=%d %s %d\n", base->sat.sat_available, base->n3g.n3g_status, base->tsc.tsc_addr, base->tsc.tsc_port);
			offset = 0;
			bTscConnected = 0;
			base->tsc.tsc_hb_req_ltime = 0;
			base->tsc.tsc_hb_rsp_ltime = 0;
			base->tsc.SizeTmp = 0;
			base->tsc.SendBufSize = 0;
			seconds_sleep(1);
			continue;
		}
		
		FD_ZERO(&fds);/* 每次循环都需要清空 */
		FD_SET(sock_tsc, &fds); /* 添加描述符 */
		timeout.tv_sec = 1;		
	    switch(select(sock_tsc+1, &fds, NULL, NULL, &timeout))
		{
			case -1: break;
			case  0: 
			{
				ioctl(sock_tsc, SIOCOUTQ, &(base->tsc.SendBufSize));//获取tcp未发送缓冲区数据大小	
				//if(base->tsc.tsc_hb_req_ltime > base->tsc.tsc_hb_rsp_ltime)
				//{
				//	satfi_log("base->tsc.SendBufSize = %d %d", base->tsc.SendBufSize, base->tsc.SizeTmp);
				//}
				if(base->tsc.SendBufSize > 0)
				{
					time2 = time(0);
					if(base->tsc.SizeTmp == 0)
					{
						base->tsc.SizeTmp = base->tsc.SendBufSize;
						time1 = time2;
					}
					else
					{
						if(base->tsc.SendBufSize == base->tsc.SizeTmp)
						{
							if(time2 - time1 > 5)satfi_log("bufsize no change %d %d", time2 - time1, base->tsc.SizeTmp);
							//close(ConnectTSC(base->sat.sat_ifname, base->tsc.tsc_addr, base->tsc.tsc_port, &err, 10));
							base->tsc.tsc_hb_req_ltime = time(0);
							if(time2 > time1 + 90)
							{
								Close_Tsc_Socket();
							}
						}
						else
						{
							//satfi_log("SendBufSize change %d %d %d", base->tsc.SizeTmp, base->tsc.SendBufSize, base->tsc.SizeTmp - base->tsc.SendBufSize);
							base->tsc.SizeTmp = base->tsc.SendBufSize;
							base->tsc.tsc_hb_rsp_ltime = time(0);
							time1 = time2;
						}
					}
				}
				else
				{
					base->tsc.SizeTmp = 0;
					//base->tsc.tsc_hb_rsp_ltime = time(0);
				}
			}
			break;
			
			default:
			{
				if(sock_tsc > 0 && FD_ISSET(sock_tsc, &fds))
				{
					nread = read(sock_tsc, buf+offset, 4096);
					if(nread>0)
					{
						base->tsc.tsc_hb_rsp_ltime = time(0);
						nread += offset;
						offset = 0;
						int c=0;
						while(1)
						{
							pHeader = (Header *)(&buf[offset]);
							//++c;if(c>10){satfi_log("tsc data error mclass=%x length=%d", pHeader->mclass, pHeader->length);offset=0;break;}
							satfi_log("read from TSC nread=%d,length=%d,mclass=0X%04X offset=%d\n",nread,pHeader->length,pHeader->mclass,offset);
							if((sizeof(pHeader->length) + offset > nread) || (nread < pHeader->length + offset))
							{
								satfi_log("offset=%d,nread=%d\n", offset, nread);
								if(offset > 0)memcpy(buf, pHeader, nread - offset);
								offset = nread - offset;
								satfi_log("leftsize=%d\n", offset);
								break;
							}
							
							if(handle_tsc_msg((char *)pHeader, tmp) < 0)
							{
								offset = 0;
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
					else
					{
						satfi_log("sock_tsc error %d %s\n",errno, strerror(errno));
						Close_Tsc_Socket();
					}
				}
			}
			break;
		}
	}
}

static int CheckProgramUpdate(void)
{
	char md5sum[256] = {0};
	char satfimd5sum[256] = {0};
	char version[256] = {0};
	char satfiurl[256] = {0};
	char cmd[256] = {0};
	char tmpBuf[256] = {0};
	int maxline = 3;

	myexec("rm -f /tmp/satfi_1_ramips_24kec.ipk", NULL, NULL);
	myexec("rm -f /tmp/update.ini", NULL, NULL);
	
	//bzero(cmd, sizeof(cmd));
	//sprintf(cmd,"wget -c -P /tmp/ %s", config_url);
	//myexec(cmd, NULL, NULL);

	if(!isFileExists("/tmp/update.ini"))
	{
		satfi_log("wget /tmp/update.ini not exit\n");
		bzero(cmd, sizeof(cmd));
		sprintf(cmd,"httpdown %s /tmp/update.ini %s", base.tsc.route, config_url);
		satfi_log("%s",cmd);
		myexec(cmd, NULL, NULL);
	}

	if(!isFileExists("/tmp/update.ini"))
	{
		satfi_log("httpdown /tmp/update.ini not exit\n");
		bzero(cmd, sizeof(cmd));
		sprintf(cmd,"curl -o /tmp/update.ini %s", config_url);
		myexec(cmd, NULL, NULL);
	}

	if(!isFileExists("/tmp/update.ini"))
	{
		satfi_log("/tmp/update.ini not exit %d\n",__LINE__);
		return 2;
	}

	GetIniKeyString("update","VERSION","/tmp/update.ini",version);
	GetIniKeyString("update","MD5SUM","/tmp/update.ini",md5sum);
	GetIniKeyString("update","SATFIMD5SUM","/tmp/update.ini",satfimd5sum);
	GetIniKeyString("update","URL","/tmp/update.ini",satfiurl);

	satfi_log("version=%s satfi_version=%s\n",version, satfi_version);
	satfi_log("md5sum=%s\n",md5sum);
	satfi_log("satfimd5sum=%s\n",satfimd5sum);
	satfi_log("satfiurl=%s\n",satfiurl);
	if((strlen(version) != 0) && (strlen(md5sum) != 0)
		&& (strlen(satfiurl) != 0) && (strlen(satfi_version) != 0))
	{
		if(strcmp(satfi_version,version) != 0)
		{
			//download
			if(!isFileExists("/tmp/satfi_1_ramips_24kec.ipk"))
			{
				satfi_log("wget /tmp/satfi_1_ramips_24kec.ipk not exit\n");
				bzero(cmd, sizeof(cmd));
				sprintf(cmd,"httpdown %s /tmp/satfi_1_ramips_24kec.ipk %s", base.tsc.route, satfiurl);
				satfi_log("%s",cmd);
				myexec(cmd, NULL, NULL); 
			}

			if(!isFileExists("/tmp/satfi_1_ramips_24kec.ipk"))
			{
				satfi_log("httpdown /tmp/satfi_1_ramips_24kec.ipk not exit\n");
				bzero(cmd, sizeof(cmd));
				sprintf(cmd,"curl -o /tmp/satfi_1_ramips_24kec.ipk %s", satfiurl);
				myexec(cmd, NULL, NULL); 
			}

			if(!isFileExists("/tmp/satfi_1_ramips_24kec.ipk"))
			{
				satfi_log("/tmp/satfi_1_ramips_24kec.ipk not exit\n");
				return 2;
			}

			myexec("md5sum /tmp/satfi_1_ramips_24kec.ipk", tmpBuf, &maxline);

			if(strstr(tmpBuf, md5sum) != NULL)
			{
				myexec("opkg install /tmp/satfi_1_ramips_24kec.ipk --force-overwrite --force-downgrade", tmpBuf, &maxline);
				myexec("opkg install /tmp/satfi_1_ramips_24kec.ipk",tmpBuf, &maxline);

				bzero(tmpBuf,sizeof(tmpBuf));
				myexec("md5sum /bin/satfi", tmpBuf, &maxline);
				if(strstr(tmpBuf, satfimd5sum) != NULL)
				{
					satfi_log("update success\n");
					myexec("rm -f /tmp/satfi_1_ramips_24kec.ipk", NULL, NULL);
					myexec("rm -f /tmp/update.ini", NULL, NULL);
					return 0;//update success
				}
				else
				{
					satfi_log("1 %s %s %d\n",tmpBuf,satfimd5sum,maxline);
				}

				seconds_sleep(1);
			}
			else
			{
				satfi_log("2 %s %s\n",tmpBuf,md5sum);
			}
		}
		else
		{
			satfi_log("not need update\n");
			myexec("rm -f /tmp/update.ini", NULL, NULL);
			return 1;//not need update
		}
	}	

	satfi_log("update error\n");
	return 2;
}

void udpVoiceHeartBeat(BASE *base)
{
	static int UDPheartbeatCnt = 0;
	
	if(bTscConnected && AppCnt)//udp端口对讲心跳包，60s
	{
		if(UDPheartbeatCnt % udpvoicetimeout == 0)
		{
			char tmp[16] = {0};
			Msg_Ptt_Voice *req1 = (Msg_Ptt_Voice *)tmp;
			req1->header.length = sizeof(Msg_Ptt_Voice);
			req1->header.mclass = PPT_VOICE;
			//memcpy(req1->FromMsID, rsp->FromMsID, 6);
			
			struct sockaddr_in tsc_addr;
			bzero(&tsc_addr, sizeof(tsc_addr));
			tsc_addr.sin_family = AF_INET;
			tsc_addr.sin_port = htons(UDP_VOICE_DSTPORT);
			tsc_addr.sin_addr.s_addr =	inet_addr(base->tsc.tsc_addr);
			/*PPT_VOICE*/
			//satfi_log("udp heartbeat for ppt voice %d\n", AppCnt);
			sendto(sock_udp, tmp, req1->header.length, 0, (struct sockaddr*)&tsc_addr, sizeof(tsc_addr));
		
			UDPheartbeatCnt = 1;
		}
		else
		{
			UDPheartbeatCnt++;
		}
	}
	else
	{
		UDPheartbeatCnt = 0;
	}	
}

void HeartBeat(BASE *base)
{
  	static int loop1 = 0;
  	static int loop2 = 0;
	if(sock_tsc > 0)
	{
		if(bTscConnected == 0)
		{
			sendConnect();
			loop1 = 0;
			loop2 = 0;
			seconds_sleep(10);
			if(bTscConnected == 0)
			{
				satfi_log("ConnectCmd no rsp");
				Close_Tsc_Socket();
				return;
			}
		}

		if(base->tsc.tsc_timeout > 0)
		{
			if(loop2 >= base->tsc.tsc_timeout || loop2 == 0)
			{
				//SaveGpsToFile(GPS_DATA_FILE);
				updateMSList();
				sendHeartBeat();				
				loop2 = 0;
			}
			loop2++;
		}
		else
		{
			base->tsc.tsc_timeout = tsc_hb_timeout;
		}

		return;
	}
	else
	{
		if(base->tsc.tsc_timeout > 0)
		{
			if(loop2 >= base->tsc.tsc_timeout || loop2 == 0)
			{
				if(bGetGpsData)
				{
					SaveGpsToFile(GPS_DATA_FILE);
				}
				loop2 = 0;
			}
			if(bGetGpsData)loop2++;
		}
		else
		{
			base->tsc.tsc_timeout = tsc_hb_timeout;
		}
	}
}


void *CheckProgramUpdateServer(void *p)
{
	BASE *base = (BASE *)p;
	int CheckProgramUpdateCnt = 0;
	int NeedCheckProgramUpdate = 1;
	
	while(1)
	{
		if(bTscConnected == 1)
		{
			if(NeedCheckProgramUpdate && CheckProgramUpdate() == 0)//版本升级，升级成功并重启
			{
				satfi_log("update success reatart\n");
				exit(8);
			}
			
			NeedCheckProgramUpdate = 0;
			CheckProgramUpdateCnt++;
			if(CheckProgramUpdateCnt >= base->tsc.update_interval)
			{
				CheckProgramUpdateCnt = 0;
				NeedCheckProgramUpdate = 1;
			}
		}

		seconds_sleep(1);
	}
	
}

int CreateGpsMessage(void)
{ 
	if(bGetGpsData == 1 && base.gps.Lg != 0 && base.gps.Lt != 0)
	{
		char indata[1024] = {0};
		char outdata[1024] = {0};
		sprintf(indata, "%s,%d,%d,%d",	\
			base.sat.sat_imei, base.gps.Lg, \
			base.gps.Lt, base.gps.Speed);
		
		CreateMsgData("13112121509", indata, outdata);//1069078255208
		//MessageADD(NULL, "13112121509", 0, outdata, strlen(outdata));
		return 1;
	}

	return 0;
}

void CreateSOSMessage(char *Msid, int Lg, int Lt, int AlarmType)
{
	char indata[1024]= {0};
	char outdata[1024]= {0};

	if(bGetGpsData == 1)
	{
		sprintf(indata, "%s,%d,%d,%d,%d",	\
			base.sat.sat_imei, base.gps.Lg, \
			base.gps.Lt, base.gps.Speed, AlarmType);
	}
	else
	{
		sprintf(indata, "%s,%d,%d,%d,%d",		\
			base.sat.sat_imei, Lg,				\
			Lt, base.gps.Speed, AlarmType);
	}
	CreateMsgData("13112121509", indata, outdata);
	MessageADD(Msid, "13112121509", 0, outdata, strlen(outdata));	
}

void CreateMessage(char *Msid, char *toPhone, int ID, char *messagedata)
{
	char outdata[1024]= {0};
	CreateMsgData(toPhone, messagedata, outdata);
	MessageADD(Msid, toPhone, ID, outdata, strlen(outdata));		
}

void *SystemServer(void *p)
{
#define GPS_INTERVAL	1800
	BASE *base = (BASE *)p;

	time_t now;
	int sm2700ledstaton = 0;//0表示灭
	int gpsledstaton = 0;	//0表示灭
	int stat = 0;

	int TimeOutCnt = 0;
	int HeartBeatTimeOutCnt = 0;
	int GpsSendInterval = GPS_INTERVAL;

	int msg_send_timeout=0;

	while(1)
	{
		now = time(0);
		if(base->tsc.tsc_hb_req_ltime == 0 && base->tsc.tsc_hb_rsp_ltime == 0)
		{
			stat &= (~(1<<0));
			TimeOutCnt = 0;
			HeartBeatTimeOutCnt = 0;
		}
		else
		{
			if(base->tsc.tsc_hb_rsp_ltime < base->tsc.tsc_hb_req_ltime)
			{
				//satfi_log("TimeOutCnt = %d %d %d\n", TimeOutCnt, base->tsc.tsc_hb_rsp_ltime, base->tsc.tsc_hb_req_ltime);
				stat &= (~(1<<0));
				if(PackHead == NULL)TimeOutCnt++;
			}
			else
			{
				stat |= (1<<0);
				TimeOutCnt = 0;
				HeartBeatTimeOutCnt = 0;
			}
			
			if(TimeOutCnt >= 60)
			{
				HeartBeatTimeOutCnt++;
				TimeOutCnt = 0;
				if(HeartBeatTimeOutCnt > 2)
				{
					HeartBeatTimeOutCnt = 0;
					stat &= (~(1<<0));
					//Close_Tsc_Socket();
				}
				else
				{
					sendHeartbeatSP();
				}
			}
		}
		
		seconds_sleep(1);
		HeartBeat(base);
		udpVoiceHeartBeat(base);
		SendPackToTSC();//处理消息发送到服务器的优先级别，优先发送指令，对讲数据，消息，语音，图片

		if(GpsSendInterval == GPS_INTERVAL)
		{
			if(CreateGpsMessage() == 1)
			{
				GpsSendInterval--;
			}
		}
		else if((GpsSendInterval--) <= 0)
		{
			GpsSendInterval=GPS_INTERVAL;
			if(CreateGpsMessage() == 1)
			{
				GpsSendInterval--;
			}
		}

		//if(base->sat.sat_state_phone != SAT_STATE_PHONE_DIALING)
		if(base->sat.sat_calling == 0)
		{
			if(base->sat.sat_msg_sending == 0)
			{
				//没有短信正在发送
				CheckisHaveMessageAndTriggerSend(base->sat.sat_fd_message);
				msg_send_timeout=0;
			}
			else
			{
				//短信发送中，超时删除
				msg_send_timeout++;
				if(msg_send_timeout >= 120)
				{
					if(MessagesHead)
					{
						MsgSendMsgRsp rsp;
						rsp.header.length = sizeof(MsgSendMsgRsp);
						rsp.header.mclass = SEND_MESSAGE_RESP;
						strncpy(rsp.MsID, MessagesHead->MsID, 21);
						rsp.Result = 1;//短信发送失败
						rsp.ID = MessagesHead->ID;
						Data_To_MsID(rsp.MsID, &rsp);
					}

					satfi_log("MessageDel timeout");
					MessageDel();
					base->sat.sat_msg_sending = 0;
				}
			}
		}
	}
	
	return NULL;
}

/* 初始化
 *
 */
void init()
{
	base.sat.sat_fd = -1;
	base.sat.sat_fd_message = -1;
	base.sat.sat_status = 0;
	base.sat.sat_state = -1;
	base.sat.sat_calling = 0;
	base.sat.sat_dialing = 0;
	base.sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
	base.sat.captain_socket = -1;;
	base.sat.start_time = 0;
	base.sat.end_time = 0;
	
	base.n3g.n3g_fd = -1;
	base.n3g.n3g_status = 0;
	base.n3g.n3g_state = -1;
	base.n3g.n3g_dialing = 0;

	base.gps.gps_fd = -1;

	char ucTmp[256];
	GetIniKeyString("server","TSCDOMAIN","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.tsc.tsc_domain, ucTmp);
		satfi_log("TSC_DOMAIN:%s\n", ucTmp);
	}

	GetIniKeyString("server","TSCADDR","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.tsc.tsc_addr, ucTmp);
		satfi_log("TSC_ADDR:%s\n", ucTmp);
	}

	GetIniKeyString("server","APPADDR","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.app.app_addr, ucTmp);
		satfi_log("APP_ADDR:%s\n", ucTmp);
	}

	base.tsc.tsc_port = GetIniKeyInt("server","TSCPORT","/etc/config.ini");
	satfi_log("TSC_PORT:%d\n", base.tsc.tsc_port);
	base.tsc.update_interval = GetIniKeyInt("server","UPDATEINTERVAL","/etc/config.ini");
	satfi_log("UPDATEINTERVAL:%d\n", base.tsc.update_interval);
	base.app.app_port = GetIniKeyInt("server","APPPORT","/etc/config.ini");
	satfi_log("APP_PORT:%d\n", base.app.app_port);
	base.tsc.tsc_timeout = GetIniKeyInt("server","TSCTIMEOUT","/etc/config.ini");
	satfi_log("TSC_TIMEOUT:%d\n", base.tsc.tsc_timeout);
	base.tsc.keepalive_interval = GetIniKeyInt("server","KEEPALIVE","/etc/config.ini");
	satfi_log("KEEPALIVE:%d\n", base.tsc.keepalive_interval);
	base.app.app_timeout = GetIniKeyInt("server","APPTIMEOUT","/etc/config.ini");
	satfi_log("APP_TIMEOUT:%d\n", base.app.app_timeout);

	GetIniKeyString("omc","OMCDOMAIN","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.omc.omc_domain, ucTmp);
		satfi_log("OMC_DOMAIN:%s\n", ucTmp);
	}

	GetIniKeyString("omc","OMCADDR","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.omc.omc_addr, ucTmp);
		satfi_log("OMC_ADDR:%s\n", ucTmp);
	}

	base.omc.omc_port = GetIniKeyInt("omc","OMCPORT","/etc/config.ini");
	satfi_log("OMC_PORT:%d\n", base.omc.omc_port);
	base.omc.omc_timeout = GetIniKeyInt("omc","OMCTIMEOUT","/etc/config.ini");
	satfi_log("OMC_TIMEOUT:%d\n", base.omc.omc_timeout);

	GetIniKeyString("satellite","IFNAME","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.sat.sat_ifname, ucTmp);
		satfi_log("SAT_IFNAME:%s\n", ucTmp);
	}

	GetIniKeyString("satellite","IFNAMEA","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.sat.sat_ifname_a, ucTmp);
		satfi_log("SAT_IFNAMEA:%s\n", ucTmp);
	}

	GetIniKeyString("satellite","DEVNAME","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.sat.sat_dev_name, ucTmp);
		satfi_log("SAT_DEV_NAME:%s\n", ucTmp);
	}
	
	GetIniKeyString("satellite","SAT_IMEI","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strncpy(base.sat.sat_imei, ucTmp, 16);
		satfi_log("SAT_IMEI:%s\n", ucTmp);
	}

	base.sat.sat_baud_rate = GetIniKeyInt("satellite","BAUDRATE","/etc/config.ini");
	satfi_log("SAT_BAUD_RATE:%d\n", base.sat.sat_baud_rate);

	base.sat.charge = GetIniKeyInt("satellite","CHARGE","/etc/config.ini");
	satfi_log("CHARGE:%d\n", base.sat.charge);

	base.sat.sat_csq_limit_value = GetIniKeyInt("satellite","CSQLIMIT","/etc/config.ini");
	satfi_log("CSQLIMIT:%d\n", base.sat.sat_csq_limit_value);

	GetIniKeyString("gprs","IFNAME","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.n3g.n3g_ifname, ucTmp);
		satfi_log("GPRS_IFNAME:%s\n", ucTmp);
	}

	GetIniKeyString("gprs","IFNAMEA","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.n3g.n3g_ifname_a, ucTmp);
		satfi_log("GPRS_IFNAMEA:%s\n", ucTmp);
	}

	GetIniKeyString("gprs","DEVNAME","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.n3g.n3g_dev_name, ucTmp);
		satfi_log("GPRS_DEV_NAME:%s\n", ucTmp);
	}

	base.n3g.n3g_baud_rate = GetIniKeyInt("gprs","BAUDRATE","/etc/config.ini");
	satfi_log("GPRS_BAUD_RATE:%d\n", base.n3g.n3g_baud_rate);

	base.n3g.n3g_csq_limit_value = GetIniKeyInt("gprs","CSQLIMIT","/etc/config.ini");
	satfi_log("GPRS_CSQ_LIMIT:%d\n", base.n3g.n3g_csq_limit_value);

	GetIniKeyString("gps","DEVNAME","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.gps.gps_dev_name, ucTmp);
		satfi_log("GPS_DEV_NAME:%s\n", ucTmp);
	}
	base.gps.gps_baud_rate = GetIniKeyInt("gps","BAUDRATE","/etc/config.ini");
	satfi_log("GPS_BAUD_RATE:%d\n", base.gps.gps_baud_rate);

	GetIniKeyString("version","VERSION","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(satfi_version, ucTmp);
		satfi_log("satfi_version:%s\n", ucTmp);
	}

	GetIniKeyString("version","URL","/etc/config.ini",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(config_url, ucTmp);
		satfi_log("url:%s\n", ucTmp);
	}

	version_num = GetIniKeyInt("version","VERSIONNUM","/etc/config.ini");
	satfi_log("version_num:%d\n", version_num);
	
	GetIniKeyString("satellite", "SAT_IMEI", SAT_IMEI_FILE, base.sat.sat_imei);
	if(strlen(base.sat.sat_imei)==0)
	{
		sprintf(base.sat.sat_imei ,"80000086%07u", time(0)%1000000);
		SetKeyString("satellite", "SAT_IMEI", SAT_IMEI_FILE, NULL, base.sat.sat_imei);
	}
}
 
int AppCallUpRsp(int socket, short sat_state_phone)
{	
	static short stat = -1;
	if(stat != sat_state_phone)
	{
		satfi_log("socket=%d sat_state_phone=%d\n",socket,sat_state_phone);
		MsgAppCallUpRsp rsp;
		rsp.header.length = sizeof(MsgAppCallUpRsp);
		rsp.header.mclass = CALLUP_RSP;
		rsp.result = sat_state_phone;
		if(socket>0)
		{
			int n = write(socket, &rsp, rsp.header.length);
			if(n<0) satfi_log("write return error: errno=%d (%s) %d %d %d\n", errno, strerror(errno),socket,__LINE__,sat_state_phone);
		}
	}
	stat = sat_state_phone;
	if(sat_state_phone >= 4)
	{
		stat = -1;//拨号退出 清除
	}
	return 0;
}

int AppRingUpRsp(int socket, char* called_number)
{
	if(socket <= 0 || called_number == NULL)
	{
		return -1;
	}
	MsgAppRingUpRsp rsp;
	rsp.header.length = sizeof(MsgAppRingUpRsp);
	rsp.header.mclass = RINGUP_RSP;
	strncpy(rsp.called_number, base.sat.called_number, sizeof(rsp.called_number));
	int n = write(socket, &rsp, rsp.header.length);
    if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),socket,__LINE__);
	return 0;
}

int AppHangingUpRsp(int socket, int sat_state_phone)
{
	if(socket <= 0)
	{
		return -1;
	}
	MsgAppCallUpRsp rsp;
	rsp.header.length = sizeof(MsgAppCallUpRsp);
	rsp.header.mclass = HANGINGUP_RSP;
	rsp.result = sat_state_phone;
	int n = write(socket, &rsp, rsp.header.length);
    if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),socket,__LINE__);
	return 0;
}

int AppAnsweringPhoneRsp(int socket, int sat_state_phone)
{
	if(socket <= 0)
	{
		return -1;
	}
	MsgAppCallUpRsp rsp;
	rsp.header.length = sizeof(MsgAppCallUpRsp);
	rsp.header.mclass = ANSWERINGPHONE_RSP;
	rsp.result = sat_state_phone;
	int n = write(socket, &rsp, rsp.header.length);
    if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),socket,__LINE__);
	return 0;
}

/* 构造字符串格式的CSQ
 *
 */
char *make_csq(char *buf, time_t *timep, int csqval)
{
  if(*(int *)timep == 0)
  {
    buf[0] = 0;
  }
  else
  {
    struct tm *tmp = localtime(timep);
    sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d %d",
            tmp->tm_year+1900,tmp->tm_mon+1,tmp->tm_mday,
            tmp->tm_hour,tmp->tm_min,tmp->tm_sec,
            csqval);
  }
  return buf;
}

static void *CallUpThread(void *p)
{
	satfi_log("CallUpThread Create\n");
	BASE *base = (BASE*)p;
	char buf[256] = {0};
	
	base->sat.sat_state_phone = SAT_STATE_PHONE_CLCC;
	int atdwaitcnt=0,clcccnt=0,ringcnt=0,dialcnt=0,dialfailecnt=0;

	while(base->sat.sat_calling)
	{
		if(base->sat.sat_fd == -1 && base->sat.sat_state_phone != SAT_STATE_PHONE_ONLINE)
	    {
			if(init_serial(&base->sat.sat_fd, base->sat.sat_dev_name, base->sat.sat_baud_rate) < 0)
			{
				base->sat.sat_calling = 0;
				base->sat.sat_state_phone = SAT_STATE_PHONE_DIALINGFAILE;
				AppCallUpRsp(base->sat.socket, get_sat_dailstatus());
			}
	    }
		
		if(base->sat.sat_state_phone == SAT_STATE_PHONE_ATH_W)
		{
			base->sat.EndTime = time(0);
			satfi_log("SAT_STATE_PHONE_ATH_W BREAK\n");
			break;
		}
		
		//satfi_log("base->sat.sat_state_phone = %d\n", base->sat.sat_state_phone);
		switch(base->sat.sat_state_phone)
		{
			case SAT_STATE_PHONE_CLCC:
				satfi_log("SAT_STATE_PHONE_CLCC\n");
		        uart_send(base->sat.sat_fd, "AT^PCMMODE=0,0,0,0,0,0,0\r\n", 26);
		        break;
			case SAT_STATE_PHONE_CLCC_OK:
				satfi_log("SAT_STATE_PHONE_CLCC_OK\n");
				sprintf(buf,"ATD%s;\r\n",base->sat.calling_number);
				//satfi_log("ATD = %s\n", buf);
				uart_send(base->sat.sat_fd, buf, 6+strlen(base->sat.calling_number));
				base->sat.sat_state_phone = SAT_STATE_PHONE_DIALING_CLCC;
				break;
			case SAT_STATE_PHONE_ATD_W:
				satfi_log("SAT_STATE_PHONE_ATD_W\n");
				AppCallUpRsp(base->sat.socket, get_sat_dailstatus());
				//atdwaitcnt++;
				if(atdwaitcnt == 10)
				{
					base->sat.sat_state_phone = SAT_STATE_PHONE_DIALING_ATH_W;
					uart_send(base->sat.sat_fd, "ATH\r\n", 5);
					atdwaitcnt = 0;
				}
				break;
			case SAT_STATE_PHONE_DIALING_CLCC:
				satfi_log("SAT_STATE_PHONE_DIALING_CLCC\n");
				uart_send(base->sat.sat_fd, "AT+CLCC\r\n", 9);
				break;
			case SAT_STATE_PHONE_DIALING_ATH_W:
				satfi_log("SAT_STATE_PHONE_DIALING_ATH_W\n");
				base->sat.sat_state_phone = SAT_STATE_PHONE_CLCC;
				break;
			case SAT_STATE_PHONE_DIALING:
			case SAT_STATE_PHONE_DIALING_RING:
				uart_send(base->sat.sat_fd, "AT+CLCC\r\n", 9);
				AppCallUpRsp(base->sat.socket, get_sat_dailstatus());
				break;
			case SAT_STATE_PHONE_DIALING_ERROR:	
			case SAT_STATE_PHONE_DIALINGFAILE:
				dialfailecnt++;
				if(dialfailecnt >= 3)
				{
					base->sat.sat_state_phone = SAT_STATE_PHONE_DIALING_FAILE_AND_ERROR;
				}
				else
				{
					base->sat.sat_state_phone = SAT_STATE_PHONE_CLCC;
				}
				break;
			case SAT_STATE_PHONE_IDLE:
			case SAT_STATE_PHONE_NOANSWER:
			case SAT_STATE_PHONE_HANGUP:
			case SAT_STATE_PHONE_DIALING_FAILE_AND_ERROR:
				AppCallUpRsp(base->sat.socket, get_sat_dailstatus());
				base->sat.sat_calling = 0;
				break;
			case SAT_STATE_PHONE_ONLINE:
				//satfi_log("SAT_STATE_PHONE_ONLINE\n");
				AppCallUpRsp(base->sat.socket, get_sat_dailstatus());
				if(base->sat.StartTime == 0)
				{
					base->sat.StartTime = time(0);
					satfi_log("StartTime=%lld\n", base->sat.StartTime);
				}
				break;
		}

		if(base->sat.sat_state_phone == SAT_STATE_PHONE_CLCC)
		{
			++clcccnt;
			if(clcccnt >= 10)
			{
				clcccnt = 0;
				base->sat.sat_state_phone = SAT_STATE_PHONE_DIALINGFAILE;
			}

		}
		
		if(base->sat.sat_state_phone == SAT_STATE_PHONE_DIALING_RING)
		{
			//satfi_log("ringcnt %d",ringcnt);
			++ringcnt;
			if(ringcnt >= 50)
			{
				base->sat.sat_state_phone = SAT_STATE_PHONE_NOANSWER;
				uart_send(base->sat.sat_fd, "ATH\r\n", 5);
				//ringcnt = 0;
			}
		}
		
		if(base->sat.sat_state_phone == SAT_STATE_PHONE_DIALING)
		{
			//++dialcnt;
			if(dialcnt >= 50)
			{
				base->sat.sat_state_phone = SAT_STATE_PHONE_DIALING_ATH_W;
				satfi_log("SAT_STATE_PHONE_DIALING too much ATH %d\n",base->sat.sat_state_phone);
				uart_send(base->sat.sat_fd, "ATH\r\n", 5);
				dialcnt = 0;
			}
		}
		
		seconds_sleep(1);
	}

	int cnt = 10;
	while(cnt--)
	{
		if(base->sat.sat_fd == -1)
	    {
			if(init_serial(&base->sat.sat_fd, base->sat.sat_dev_name, base->sat.sat_baud_rate) < 0)
			{
				break;
			}
	    }

		satfi_log("ATH\n");
		base->sat.sat_state_phone = SAT_STATE_PHONE_ATH_W;
		uart_send(base->sat.sat_fd, "ATH\r\n", 5);
		AppCallUpRsp(base->sat.socket, get_sat_dailstatus());
		seconds_sleep(1);
		if(base->sat.sat_state_phone == SAT_STATE_PHONE_HANGUP ||
			base->sat.sat_state_phone == SAT_STATE_PHONE_COMING_HANGUP || 
			base->sat.sat_state_phone == SAT_STATE_PHONE_IDLE)
		{
			base->sat.EndTime = time(0);
			base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
			break;
		}
	}

	if(base->sat.StartTime > 0 && 0 < base->sat.EndTime && base->sat.EndTime > base->sat.StartTime)
	{
		unsigned short CallTime = base->sat.EndTime - base->sat.StartTime;
		base->sat.CallTime = (CallTime%60) ? ((CallTime/60)*60 + 60) : CallTime;
		base->sat.Money = base->sat.CallTime * base->sat.charge;
		satfi_log("StartTime=%lld,CallTime=%d,Money=%d\n",base->sat.StartTime, base->sat.CallTime, base->sat.Money);
		satfi_log("MsID=%.21s,MsPhoneNum=%.11s,DesPhoneNum=%.11s\n",base->sat.MsID, base->sat.MsPhoneNum, base->sat.DesPhoneNum);

		char tmp[128] = {0};
		Msg_Phone_Money_Req *rsp = (Msg_Phone_Money_Req*)tmp;
		rsp->header.length = sizeof(Msg_Phone_Money_Req);
		rsp->header.mclass = APP_PHONE_MONEY_CMD;
		StrToBcd(rsp->MsID, &(base->sat.MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
		StrToBcd(rsp->MsPhoneNum, base->sat.MsPhoneNum, 11);
		StrToBcd(rsp->DesPhoneNum, base->sat.DesPhoneNum, 11);
		rsp->StartTime = base->sat.StartTime * 1000;
		rsp->CallTime = base->sat.CallTime;
		rsp->Money = base->sat.Money;

		SaveDataToFile(CALL_RECORDS_FILE ,(char *)rsp, rsp->header.length);
	}
	
	base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
	base->sat.socket = 0;
	base->sat.sat_calling = 0;
	satfi_log("CallUpThread Exit\n");
}

void StartCallUp(char *calling_number)
{
	pthread_t thread;
	strncpy(base.sat.calling_number, calling_number,sizeof(base.sat.called_number));
	pthread_create(&thread,NULL,CallUpThread,&base);
}

void AnsweringPhone()
{	
	satfi_log("AnsweringPhone ATA %d\n",base.sat.sat_fd);	
	base.sat.sat_state_phone = SAT_STATE_PHONE_ATA_W;
	uart_send(base.sat.sat_fd, "ATA\r\n", 5);
}

int StrToBcd(unsigned char *bcd, const char *str, int strlen)
{
        int i;
		char tmp[100] = {0};
		char *p = tmp;
        unsigned char hbit,lbit;
  		//printf("str:%s\n",str);
  		//printf("strTobcd:");
		if(strlen%2 != 0)
		{
			tmp[0] = '0';
			strncpy(p+1,str,strlen);
		}
		else
		{
			strncpy(tmp,str,strlen);	
		}
		
        for(i = 0; i < strlen; i+=2)
        {
            hbit = (tmp[i] > '9') ? ((tmp[i] & 0x0F) + 9) : (tmp[i] & 0x0F);
            lbit = (tmp[i+1] > '9') ? ((tmp[i+1] & 0x0F) + 9) : (tmp[i+1] & 0x0F);
            bcd[i/2] = (hbit << 4) | lbit;
			//printf("%02x ",bcd[i/2]);
        }
		//printf("%d\n",__LINE__);
        return 0;
}

int BcdToStr(unsigned char *str, const char *bcd, int bcdlen)
{
		int i;
		char tmp[100] = {0};
		for(i=0;i<bcdlen;i++)
		{
			tmp[2*i]	=	((bcd[i]>>4) & 0x0F) + '0';
			tmp[2*i+1]	=	(bcd[i] & 0x0F) + '0';
		}
		tmp[i*2] = 0;
		
		if(bcdlen%2 != 0)
		{
			strncpy(str,&tmp[1], i*2-1);
		}
		else
		{
			strncpy(str, tmp, i*2);
		}
		//printf("bcdTostr:%s\n",str);
        return 0;
}

void printsocketbufsize(int socket, char *prefix)
{
	int size1 = 0;
	int size2 = 0;
	int size3 = 0;
	int optlen = sizeof(int);
	getsockopt(socket, SOL_SOCKET, SO_SNDBUF,&size1,&optlen);
	getsockopt(socket, SOL_SOCKET, SO_RCVBUF,&size2,&optlen);
	ioctl(socket, SIOCOUTQ, &size3);
	satfi_log("%s=%d SO_SNDBUF=%d SO_RCVBUF=%d SIOCOUTQ=%d\n",prefix,socket,size1,size2,size3);
}

void set_block(int socket,int on) 
{
    int flags;
    flags = fcntl(socket,F_GETFL,0);
    if (on==0) {
        fcntl(socket, F_SETFL, flags | O_NONBLOCK);
    }else{
        flags &= ~ O_NONBLOCK;
        fcntl(socket, F_SETFL, flags);
    }
}

int ConnectTSC(char* routename, char* ip, int port, int *err, int timeout)
{
	int sockfd;
    struct sockaddr_in server_addr;   
	struct timeval timeo = {timeout, 0};
	int on = 1;
	int size_send = 1024*1024;
	int optlen = sizeof(int);
    struct ifreq struIR;
	
	struct linger so_linger;
	//当l_onoff值设置为非0值，而l_linger也设置为0，强制关闭socket如果此时缓冲区中有未发送数据，所有未发数据都将丢失
	so_linger.l_onoff = 1;
	so_linger.l_linger = 0;
	
    if(inet_aton(ip, &server_addr.sin_addr) == 0) 
	{
        satfi_log("the hostip is not right!");   
        return -1;  
    }

    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
	{
        satfi_log("Socket Error:%s", strerror(errno));   
        return -1; 
    }

	setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *)&on, sizeof(on));  
	setsockopt(sockfd, SOL_SOCKET, SO_LINGER, (void *)&so_linger, sizeof(so_linger));
	setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, (void *)&size_send, sizeof(size_send));
	//setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));        
	if(timeout > 0) setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeo, sizeof(timeo));

	if(routename)
	{
		strncpy(struIR.ifr_name, routename, IFNAMSIZ);
		if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, &struIR, sizeof(struIR)) < 0)
		{
			satfi_log("setsockopt Error:%s\n", strerror(errno));  
			//close(sockfd);
			//return -1; 
		}
	}

    server_addr.sin_family = AF_INET;   
    server_addr.sin_port = htons(port);   

    if(connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) 
	{
		if(err != NULL) *err = errno;
        //if(routename && sock_tsc < 0)satfi_log("Connect Error:%s errno=%d sockfd=%d %s %d %s\n", strerror(errno), errno, sockfd, ip, port, routename);   
		close(sockfd);
        return -1;
    }
	else
	{
		satfi_log("ConnectTSC success sockfd=%d\n", sockfd);
	}

	return sockfd;
}

int appsocket_init(int port)
{	
	int ret = 0;
	struct sockaddr_in server_addr;		// 服务器地址结构体
		
	int socket_tcp = socket(AF_INET, SOCK_STREAM, 0);   // 创建TCP套接字
	if(socket_tcp < 0)
	{
		satfi_log("socket error");
		exit(1);
	}
	
    unsigned int value = 1;  
    if (setsockopt(socket_tcp, SOL_SOCKET, SO_REUSEADDR,  
                (void *)&value, sizeof(value)) < 0)  
    {  
        satfi_log("fail to setsockopt");  
		close(socket_tcp);		
        exit(1);  
    }
	
	bzero(&server_addr, sizeof(server_addr));	   // 初始化服务器地址
	server_addr.sin_family = AF_INET;
	server_addr.sin_port   = htons(port);
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int sizer = 256*1024;
	int sizes = 256*1024;
	int optlen = sizeof(int);
	setsockopt(socket_tcp, SOL_SOCKET, SO_RCVBUF, (void *)&sizer, optlen);
	setsockopt(socket_tcp, SOL_SOCKET, SO_SNDBUF, (void *)&sizes, optlen);

	//int on=1;
	//setsockopt(socket_tcp, IPPROTO_TCP, TCP_NODELAY,&on,sizeof(int));        

	ret = bind(socket_tcp, (struct sockaddr*)&server_addr, sizeof(server_addr));
	if(ret != 0)
	{
		satfi_log("app_tcp_bind port=%d", port);
		satfi_log("PID=%d PPID=%d\n",getpid(), getppid());
		close(socket_tcp);		
		exit(1);
	}
	
	ret = listen(socket_tcp, 10);
	if( ret != 0)
	{
		satfi_log("app_tcp_listen");
		close(socket_tcp);		
		exit(1);
	}

	return socket_tcp;
}

void printfhex(unsigned char *buf, int len)
{
	int i = 0;
	//printf("len=%d\n",len);
	for(i=0;i<len;i++)
	{
		printf("%02X ",buf[i]);
	}
	printf("\n");
}

void handle_gps_data(int gpsfd)
{
#define HEADER_SIZE	7
#define TAIL_SIZE	2
	static int idx = 0;
	char buf[1024]={0};
	static char gpsbuf[1024];
	char *tmp;

	int n = read(gpsfd, buf, 1024);
	if(n-HEADER_SIZE-TAIL_SIZE>0)
	{
		if(idx+n-HEADER_SIZE-TAIL_SIZE<1024)
		{
			buf[n] = 0;
			//satfi_log("%d %s\n",n, gpsbuf);
			if(tmp = strstr(buf, "AT+ADCVOL="))
			{
				VoltageVal = atoi(&tmp[10]);//2.7V VOLTAGE_WARNING_ADC_VAL
				//satfi_log("VoltageVal=%d", VoltageVal);
				return;
			}
			
			memcpy(&gpsbuf[idx], &buf[HEADER_SIZE], n-HEADER_SIZE-TAIL_SIZE);//AT+U=3,DATA\r\n去掉协议头7字节 2个尾部包括逗号 ,剩下中间数据
			idx += n-HEADER_SIZE-TAIL_SIZE;

			while(idx>0)
			{
				int k=0;
				while(k<idx && gpsbuf[k]!='$') k++;
				if(k>=idx)
				{
					idx=0;
					break;
				}
				int kk=k+1;
				while(kk<idx && kk+1<idx)
				{
					if(gpsbuf[kk]==0xa && gpsbuf[kk+1] == 0xa)
					{
						if((strncmp(&gpsbuf[k],"$GPRMC",6)==0) || (strncmp(&gpsbuf[k],"$GNRMC",6)==0))
						{
							gpsbuf[kk]='\0';
							//printf("gpsbuf:%s\n", &gpsbuf[k]);
							//strncpy(GpsData, &gpsbuf[k], kk-k);
							//GpsData[kk-k] = 0;
							strncpy(base.gps.gps_bd, &gpsbuf[k], kk-k);
							base.gps.gps_bd[kk-k] = 0;
							parseGpsData(&gpsbuf[k],kk-k);
						}
						kk+=2;
						int kkk;
						for(kkk=kk;kkk<idx;kkk++)
						{
							gpsbuf[kkk-kk] = gpsbuf[kkk];
						}
						idx -= kk;
						kk = -1;
						break;
					}
					else
					{
						kk++;
					}
				}
				if(kk>=idx-1)
				{
					int kkk;
					for(kkk=k;kkk<idx;kkk++)
					{
						gpsbuf[kkk-k] = gpsbuf[kkk];
					}
					idx -= k;
					break;
				}
			}
		}
		else
		{
			satfi_log("GPS data is too long to read idx=%d n=%d\n",idx,n);
		}
	}
	else if(n-HEADER_SIZE-TAIL_SIZE < 0)
	{
		satfi_log("close(base.gps.gps_fd)\n");	
		close(base.gps.gps_fd);
		base.gps.gps_fd = -1;
	}
	else if(n-HEADER_SIZE-TAIL_SIZE == 0)
	{
		satfi_log("no GPS data\n");		
	}
}

static void *ring_detect(void *p)
{
	BASE *base = (BASE *)p;
	int ring = 0;
	int ringnocarriercnt = 0;
	int ringcnt = 0;

	int ringsocket = -1;
	
	while(1)
	{
		if(base->sat.sat_state_phone == SAT_STATE_PHONE_RING_COMING)
		{
			satfi_log("ring SAT_STATE_PHONE_RING_COMING\n");
			ring = 1;
		}
		
		if(ring == 1)
		{
			net_lock();
			base->sat.sat_calling = 1;
			base->sat.EndTime = 0;
			base->sat.StartTime = 0;
			
			base->sat.sat_state_phone = SAT_STATE_PHONE_RING_COMING;

			ringnocarriercnt = 0;
			ringcnt = 0;

			int clcccnt = 0;
			while(base->sat.sat_calling)
			{
				if(base->sat.sat_state_phone == SAT_STATE_PHONE_ATH_W)
				{
					satfi_log("SAT_STATE_PHONE_ATH_W BREAK\n");
					break;
				}
				
				if(strlen(base->sat.called_number) == 0)
				{
					clcccnt++;
					uart_send(base->sat.sat_fd, "AT+CLCC\r\n", 9);//查询来电电话号码
					if(clcccnt >= 10)
					{
						base->sat.sat_calling = 0;
						break;
					}
				}
				else
				{
					ringsocket = base->sat.captain_socket;
					if(ringsocket < 0)
					{
						int i=0;
						USER * t = gp_users;
						while(t)
						{
							for(i=0; i<t->famNumConut && i<10; i++)
							{
								if(strstr(t->FamiliarityNumber[i], base->sat.called_number))
								{
									satfi_log("%s %s %.21s", t->FamiliarityNumber[i], base->sat.called_number, t->userid);
									ringsocket = t->socketfd;
									break;
								}
							}

							if(ringsocket>0)
							{
								break;
							}
							
							t=t->next;
						}
					}

					if(ringsocket < 0)
					{
						USER * t = gp_users;
						while(t)
						{
							ringsocket = t->socketfd;
							if(ringsocket>0)
							{
								satfi_log("ringsocket=%d %.21s %.21s", ringsocket, t->userid, base->sat.called_number);
								break;
							}
							t=t->next;
						}
					}

					satfi_log("SAT_STATE_PHONE_RING_COMING ringsocket=%d captain_socket=%d\n", ringsocket, base->sat.captain_socket);
					AppRingUpRsp(ringsocket,base->sat.called_number);
					AppCallUpRsp(ringsocket, get_sat_dailstatus());

					int i=0;
					int len = strlen(base->sat.called_number);
					for(i=0;i<11 && len>=11;i++)
					{
						base->sat.DesPhoneNum[10-i] = base->sat.called_number[len-i-1];
					}

					USER *pUser = gp_users;
					while(pUser)
					{
						if(pUser->socketfd == ringsocket)
						{
							memcpy(base->sat.MsID, pUser->userid, USERID_LLEN);
							memcpy(base->sat.MsPhoneNum, &(pUser->userid[USERID_LLEN - 11]), 11);
						}
						pUser = pUser->next;
					}
					satfi_log("ring MsID=%.21s MsPhoneNum=%.11s DesPhoneNum=%.11s\n", base->sat.MsID, base->sat.MsPhoneNum, base->sat.DesPhoneNum);
					break;
				}

				seconds_sleep(1);
			}
			
			while(base->sat.sat_calling)
			{
				if(ringsocket < 0)
				{
					break;
				}
				
				if(base->sat.sat_state_phone == SAT_STATE_PHONE_ATH_W)
				{
					satfi_log("SAT_STATE_PHONE_ATH_W BREAK\n");
					break;
				}
				
				if(base->sat.sat_state_phone == SAT_STATE_PHONE_COMING_HANGUP)
				{
					ringnocarriercnt++;
					if(ringnocarriercnt == 1)
					{
						satfi_log("SAT_STATE_PHONE_COMING_HANGUP captain_socket=%d\n", base->sat.captain_socket);
						AppCallUpRsp(ringsocket, get_sat_dailstatus());
						break;
					}
					else
					{
						base->sat.sat_state_phone = SAT_STATE_PHONE_RING_COMING;
					}
				}

				if(base->sat.sat_state_phone == SAT_STATE_PHONE_RING_COMING)
				{
					uart_send(base->sat.sat_fd, "AT+CLCC\r\n", 9);
					ringcnt++;
					if(ringcnt >= 60)
					{
						base->sat.sat_state_phone = SAT_STATE_PHONE_COMING_HANGUP;
						satfi_log("SAT_STATE_PHONE_COMING_HANGUP captain_socket1=%d\n", base->sat.captain_socket);
						AppCallUpRsp(ringsocket, get_sat_dailstatus());
						break;
					}
				}

				if(base->sat.sat_state_phone == SAT_STATE_PHONE_ONLINE)
				{
					//satfi_log("SAT_STATE_PHONE_ONLINE captain_socket=%d\n", base->sat.captain_socket);
					//uart_send(base->sat.sat_fd, "AT+CLCC\r\n", 9);
					AppCallUpRsp(ringsocket, get_sat_dailstatus());
					if(base->sat.StartTime == 0)
					{
						base->sat.StartTime = time(0);
						satfi_log("ring StartTime=%lld\n", base->sat.StartTime);
					}

				}
				
				if(base->sat.sat_state_phone == SAT_STATE_PHONE_HANGUP)
				{
					satfi_log("SAT_STATE_PHONE_HANGUP captain_socket=%d\n", base->sat.captain_socket);
					AppCallUpRsp(ringsocket, get_sat_dailstatus());
					break;
				}
	
				seconds_sleep(1);
			}

			base->sat.EndTime = time(0);
			
			int cnt = 10;
			while(cnt--)
			{
				if(base->sat.sat_fd == -1)
			    {
					if(init_serial(&base->sat.sat_fd, base->sat.sat_dev_name, base->sat.sat_baud_rate) < 0)
					{
						break;
					}
			    }

				satfi_log("ATH\n");
				base->sat.sat_state_phone = SAT_STATE_PHONE_ATH_W;
				uart_send(base->sat.sat_fd, "ATH\r\n", 5);
				seconds_sleep(1);
				if(base->sat.sat_state_phone == SAT_STATE_PHONE_HANGUP ||
					base->sat.sat_state_phone == SAT_STATE_PHONE_COMING_HANGUP || 
					base->sat.sat_state_phone == SAT_STATE_PHONE_IDLE)
				{
					base->sat.EndTime = time(0);
					base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
					break;
				}
				
			}

			satfi_log("ring EndTime=%lld\n", base->sat.EndTime);

			if(base->sat.StartTime > 0 && 0 < base->sat.EndTime && base->sat.EndTime > base->sat.StartTime)
			{
				unsigned short CallTime = base->sat.EndTime - base->sat.StartTime;
				base->sat.CallTime = (CallTime%60) ? ((CallTime/60)*60 + 60) : CallTime;
				base->sat.Money = base->sat.CallTime * base->sat.charge / 5;
				satfi_log("ring StartTime=%lld,CallTime=%d,Money=%d\n",base->sat.StartTime, base->sat.CallTime, base->sat.Money);
				satfi_log("ring MsID=%.21s,MsPhoneNum=%.11s,DesPhoneNum=%.11s\n",base->sat.MsID, base->sat.MsPhoneNum, base->sat.DesPhoneNum);

				char tmp[128] = {0};
				Msg_Phone_Money_Req *rsp = (Msg_Phone_Money_Req*)tmp;
				rsp->header.length = sizeof(Msg_Phone_Money_Req);
				rsp->header.mclass = APP_PHONE_MONEY_CMD;
				StrToBcd(rsp->MsID, &(base->sat.MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
				StrToBcd(rsp->MsPhoneNum, base->sat.MsPhoneNum, 11);
				StrToBcd(rsp->DesPhoneNum, base->sat.DesPhoneNum, 11);
				rsp->StartTime = base->sat.StartTime * 1000;
				rsp->CallTime = base->sat.CallTime;
				rsp->Money = base->sat.Money;

				//CallRecordsADD((char *)rsp, rsp->header.length);
				SaveDataToFile(CALL_RECORDS_FILE ,(char *)rsp, rsp->header.length);
			}
			
			satfi_log("SAT_STATE_PHONE_HANGUP %d\n", base->sat.sat_state_phone);
			bzero(base->sat.called_number,15);
			ring = 0;			
			ringsocket = -1;
			base->sat.sat_calling = 0;
			base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
			net_unlock();
		}

		seconds_sleep(1);
	}
	return NULL;
}

static void SignalHandler(int nSigno)  
{
    switch(nSigno)  
    {
	    case SIGPIPE:  
	        satfi_log("SIGPIPE\n");  
	        break;
	    default:
	        break;  
    }
}

#define PCM_SET_RECORD			0
#define PCM_SET_UNRECORD		1	
#define PCM_READ_PCM			2
#define PCM_START				3
#define PCM_STOP				4
#define PCM_SET_PLAYBACK		5
#define PCM_SET_UNPLAYBACK		6
#define PCM_WRITE_PCM			7
#define PCM_OPEN				13
#define PCM_CLOSE				14

#define NN 160
static int pcmfd=-1;
static void *recvfrom_app_voice_udp(void *p)
{
	BASE *base = (BASE *)p;
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(12070);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
	
    int sock;
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket udp");
        exit(1);
    }
	
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind udp");
        exit(1);
    }

	char tmp[320];
	char playbackbuf[3200];
	int plybackbufofs=0;
	struct sockaddr_in clientAddr;
	struct sockaddr_in *clientAddr1 = &(base->sat.clientAddr1);
	struct sockaddr_in *clientAddr2 = &(base->sat.clientAddr2);

	base->sat.voice_socket_udp = sock;

	int len = sizeof(struct sockaddr_in);
	
	bzero(&clientAddr, len);
	bzero(clientAddr1, len);
	bzero(clientAddr2, len);
	
	int n;
	struct timeval tout = {10,0};	
	fd_set fds;
	int maxfd;
	int ret;

	satfi_log("select_voice_udp %d", sock);

	SpeexPreprocessState *st;
	st = speex_preprocess_state_init(NN, 8000);

	int vad = 1;
	int vadProbStart = 90;
	int vadProbContinue = 100;
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_VAD, &vad); //静音检测
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_START , &vadProbStart); //Set probability required for the VAD to go from silence to voice
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_CONTINUE, &vadProbContinue); //Set probability required for the VAD to stay in the voice state (integer percent)
	
	while (1)
    {
		FD_ZERO(&fds);/* 每次循环都需要清空 */
		FD_SET(sock, &fds); /* 添加描述符 */
		maxfd = sock;
		tout.tv_sec = 30;
		tout.tv_usec = 0;
		
		switch(select(maxfd+1,&fds,NULL,NULL,&tout))
		{
			case -1: break;
			case  0:
				if(ntohs(clientAddr1->sin_port) != 0 && ntohs(clientAddr2->sin_port) != 0)
				{
					base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
				}
				else if(ntohs(clientAddr1->sin_port) != 0)
				{
					base->sat.sat_calling = 0;
					base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
				}
				
				if(ntohs(clientAddr1->sin_port) != 0)
				{
					satfi_log("bzero clientAddr1");
					bzero(clientAddr1, len);
				}
				
				if(ntohs(clientAddr2->sin_port) != 0)
				{
					satfi_log("bzero clientAddr2");
					bzero(clientAddr2, len);
				}

				plybackbufofs = 0;
				break;
			default:
								
				if(FD_ISSET(sock, &fds))
				{
			        n = recvfrom(sock, tmp, 320, 0, (struct sockaddr*)&clientAddr, &len);
			        if (n>0 && base->sat.sat_calling == 1)
			        {
						if(memcmp(clientAddr1 ,&clientAddr, len) == 0)
						{
							if(ntohs(clientAddr2->sin_port) != 0)
							{
								if(base->sat.sat_state_phone == SAT_STATE_PHONE_ONLINE)
								{
									n = sendto(sock, tmp, n, 0, (struct sockaddr *)clientAddr2, len);
									if (n < 0)
									{
										perror("sendto clientAddr2");
									}
								}
							}
							else
							{
								if(base->sat.sat_state_phone == SAT_STATE_PHONE_ONLINE)
								{
									vad = speex_preprocess_run(st, (spx_int16_t *)tmp);
									if(vad)
									{
										memcpy(playbackbuf+plybackbufofs, tmp, n);
										plybackbufofs += n;
										if(plybackbufofs >= sizeof(playbackbuf))
										{
											//satfi_log("plybackbufofs=%d", plybackbufofs);
											plybackbufofs = 0;
											if(pcmfd > 0)
											{
												ioctl(pcmfd, PCM_WRITE_PCM, playbackbuf);
											}						
										}
									}
								}
								else
								{
									plybackbufofs = 0;
								}
							}
						}
						else if(memcmp(clientAddr2 ,&clientAddr, len) == 0)
						{
							if(ntohs(clientAddr1->sin_port) != 0 && base->sat.sat_state_phone == SAT_STATE_PHONE_ONLINE)
							{
								//gettimeofday(&tv, NULL);
								//satfi_log(" 								   %s %d %06d %d",inet_ntoa(clientAddr2->sin_addr) ,tv.tv_sec, tv.tv_usec, n);
								n = sendto(sock, tmp, n, 0, (struct sockaddr *)clientAddr1, len);
								if (n < 0)
								{
									perror("sendto clientAddr1");
								}
							}
						}
						else
						{
							if(ntohs(clientAddr1->sin_port) == 0)
							{
								satfi_log("clientAddr1 %s %d\n", inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));
								memcpy(clientAddr1, &clientAddr, len);
							}
							else if(ntohs(clientAddr2->sin_port) == 0)
							{
								satfi_log("clientAddr2 %s %d\n", inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));
								memcpy(clientAddr2, &clientAddr, len);
							}
							else
							{
								satfi_log("%s %d\n", inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));
							}
						}
			        }					
				}
		}
    }	
}

static void *sendto_app_voice_udp(void *p)
{
	BASE *base = (BASE *)p;
	int packsize = 320;
	int ret;
	
	char voicebuf[4800];
	struct sockaddr_in *clientAddr1 = &(base->sat.clientAddr1);
	int len = sizeof(struct sockaddr_in);

	SpeexPreprocessState *st;
	st = speex_preprocess_state_init(NN, 8000);

	int vad = 1;
	int vadProbStart = 90;
	int vadProbContinue = 100;
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_VAD, &vad); //静音检测
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_START , &vadProbStart); //Set probability required for the VAD to go from silence to voice
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_CONTINUE, &vadProbContinue); //Set probability required for the VAD to stay in the voice state (integer percent)

	while(1)
	{
		if(base->sat.sat_calling == 1)
		{
			if(pcmfd<0)
			{
				int iRecordCH=0;
				pcmfd = open("/dev/pcm0", O_RDWR);
				ioctl(pcmfd, PCM_OPEN, &ret);
				satfi_log("PCM_OPEN=%d\n", ret);
				if (ret < 0)
				{
					ioctl(pcmfd, PCM_CLOSE);
					return NULL;
				}
				ioctl(pcmfd, PCM_SET_PLAYBACK, &iRecordCH);
				ioctl(pcmfd, PCM_SET_RECORD, &iRecordCH);
				ioctl(pcmfd, PCM_START, 0);
				
				if(pcmfd>0)
				{
					satfi_log("AudioRecord Init pcmfd=%d\n", pcmfd);
				}
				else
				{
					seconds_sleep(1);
				}				
			}

			if(ntohs(clientAddr1->sin_port) != 0)
			{	
				if(pcmfd>0)
				{
					ioctl(pcmfd, PCM_READ_PCM, voicebuf);
					if(ntohs(clientAddr1->sin_port) != 0)
					{
						int offset = 0;
						while(offset != sizeof(voicebuf))
						{
							if(base->sat.sat_state_phone == SAT_STATE_PHONE_ATH_W)
							{
								satfi_log("SAT_STATE_PHONE_ATH_W BREAK\n");
								break;
							}

							vad = speex_preprocess_run(st, (spx_int16_t *)&(voicebuf[offset]));
							if(vad)
							{
								ret = sendto(base->sat.voice_socket_udp, &(voicebuf[offset]), packsize, 0, (struct sockaddr *)clientAddr1, len);
								if (ret < 0)
								{
									satfi_log("sendto clientAddr11 %s", strerror(errno));
								}
							}

							offset += packsize;
						}
					}
				}			
			}
		}
		else
		{
			if(pcmfd>0)
			{
				satfi_log("AudioRecord Exit pcmfd=%d %d\n", pcmfd, base->sat.sat_calling);	
				ioctl(pcmfd, PCM_STOP, 0);
				ioctl(pcmfd, PCM_SET_UNPLAYBACK);
				ioctl(pcmfd, PCM_SET_UNRECORD);
				
				ioctl(pcmfd, PCM_CLOSE);
				close(pcmfd);
				pcmfd = -1;
				
				if(ntohs(clientAddr1->sin_port) != 0)
				{
					satfi_log("send_app_voice_udp bzero clientAddr1");
					bzero(clientAddr1, len);
				}
			}
			seconds_sleep(1);
		}
	}
}

void main_thread_loop(void)
{
	fd_set fds;
	int maxfd = 0;
	struct timeval timeout;

	int SatDataOfs[2]={0};
	char SatDataBuf[2][1024];
	
	while(1)
	{
		FD_ZERO(&fds);
		maxfd=0;

		if(base.gps.gps_fd > 0) {
			FD_SET(base.gps.gps_fd, &fds);
			if(base.gps.gps_fd > maxfd) {
				maxfd = base.gps.gps_fd;
			}
		} else {
			if (isFileExists(base.gps.gps_dev_name)) {
				init_serial(&base.gps.gps_fd, base.gps.gps_dev_name, base.gps.gps_baud_rate);
			}
		}

		if(base.sat.sat_fd > 0) {
			FD_SET(base.sat.sat_fd, &fds);
			if(base.sat.sat_fd > maxfd) {
				maxfd = base.sat.sat_fd;
			}
		}

		if(base.n3g.n3g_fd > 0) {
			FD_SET(base.n3g.n3g_fd, &fds);
			if(base.n3g.n3g_fd > maxfd) {
				maxfd = base.n3g.n3g_fd;
			}
		}

		if(maxfd == 0) {
			seconds_sleep(3);
			continue;
		}

		timeout.tv_sec = 3;
		switch(select(maxfd+1,&fds,NULL,NULL,&timeout))
		{
			case -1: break;
			case  0: break;
			default:
				if(base.gps.gps_fd > 0 && FD_ISSET(base.gps.gps_fd, &fds)) {
					handle_gps_data(base.gps.gps_fd);//定位数据
				}

				if(base.sat.sat_fd > 0 && FD_ISSET(base.sat.sat_fd, &fds)) {
					handle_sat_data(&base.sat.sat_fd, SatDataBuf[0], &SatDataOfs[0]);//卫星模块数据
				}

				if(base.n3g.n3g_fd > 0 && FD_ISSET(base.n3g.n3g_fd, &fds)) {
					handle_gprs_data(base.n3g.n3g_fd);//gprs串口数据
				}
				break;
		}
	}
	
}

int main_fork(void)
{
	pthread_t id_1,id_2;
	pthread_t id_3,id_4;
	pthread_t id_5,id_6;
	pthread_t id_7,id_8;

	prctl(PR_SET_PDEATHSIG, SIGKILL);//父进程退出发送SIGKILL 给子进程
	signal(SIGPIPE,SignalHandler);
	
	init();
	
	//处理服务器,APP数据
	if(pthread_create(&id_1, NULL, select_app, (void *)&base) == -1) exit(1);
	if(pthread_create(&id_2, NULL, select_tsc, (void *)&base) == -1) exit(1);

	//切换路由
	if(pthread_create(&id_3, NULL, check_route, (void *)&base) == -1) exit(1);

	//使用udp，接收服务器对讲语音数据
	//if(pthread_create(&id_4, NULL, select_tsc_udp, (void *)&base) == -1) exit(1);
	
	//System检测，心跳包
	if(pthread_create(&id_5, NULL, SystemServer, (void *)&base) == -1) exit(1);
	//if(pthread_create(&id_6, NULL, CheckProgramUpdateServer, (void *)&base) == -1) exit(1);

	//sat拨号线程,ring检测
	if(pthread_create(&id_4, NULL, func_y, (void *)&base) == -1) exit(1);
	if(pthread_create(&id_5, NULL, ring_detect, (void *)&base) == -1) exit(1);
	//gprs拨号线程
	//if(pthread_create(&id_6, NULL, func_z, (void *)&base) == -1) exit(1);

	//电话音频处理
	if(pthread_create(&id_7, NULL, recvfrom_app_voice_udp, (void *)&base) == -1) exit(1);
	if(pthread_create(&id_8, NULL, sendto_app_voice_udp, (void *)&base) == -1) exit(1);

	main_thread_loop();
	return 0;
}

#define RECORD				0
#define PLAYBACK			1

#define ROUTE				"br-lan2"

#define CARD1				"/dev/dsp"
#define MCAST_ADDR_CARD1	"224.0.0.56"
#define MCAST_PORT_CARD1	6789

#define CARD2				"/dev/dsp1"
#define MCAST_ADDR_CARD2	"224.0.0.78"
#define MCAST_PORT_CARD2	9876

#define MCAST_LOOP			0		//广播回环 0禁用

#define RATE				8000
#define BITS				16
#define CHANNELS			1

int broadcast_init(char *broadcastip, int broadcastport)
{
	int s; 								/*套接字文件描述符*/
	struct sockaddr_in local_addr; 		/*本地地址*/
	int err = -1;
	s = socket(AF_INET, SOCK_DGRAM, 0); /*建立套接字*/
	if (s == -1)
	{
		satfi_log("socket() error");
		return -1;
	}
	
	/*初始化地址*/
	memset(&local_addr, 0, sizeof(local_addr));
	local_addr.sin_family 		= AF_INET;
	local_addr.sin_addr.s_addr 	= htonl(INADDR_ANY);
	local_addr.sin_port 		= htons(broadcastport);
	/*绑定socket*/
	err = bind(s, (struct sockaddr*)&local_addr, sizeof(local_addr));
	if(err < 0)
	{
		satfi_log("binderror %s %d", broadcastip, broadcastport);
		return -1;
	}
	
	/*设置回环许可*/
	int loop = MCAST_LOOP;
	err = setsockopt(s,IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));
	if(err < 0)
	{
		satfi_log("setsockopt():IP_MULTICAST_LOOP error");
		return -1;
	}
	
	struct ip_mreq mreq; 									/*加入广播组*/
	mreq.imr_multiaddr.s_addr = inet_addr(broadcastip); 	/*广播地址*/
	mreq.imr_interface.s_addr = htonl(INADDR_ANY); 		/*网络接口为默认*/

	/*将本机加入广播组*/
	err = setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
	if (err < 0)
	{
		satfi_log("setsockopt():IP_ADD_MEMBERSHIP error");
		return -1;
	}

	satfi_log("broadcast success Socket=%d ip=%s port=%d", s, broadcastip, broadcastport);
	return s;
}

int card_init(char *devname, int type)
{
	int devfd;
	int rate = RATE;	//采样率
	int bits = BITS;		
	int channels = CHANNELS;	//单通道
	int flag;

	if(type == RECORD)
	{
		flag = O_RDONLY;	//record
	}
	else
	{
		flag = O_WRONLY;	//playback		
	}
	
	devfd = open(devname, flag);
	if (devfd < 0) 
	{
		satfi_log("open of %s failed", devname);
		return -1;
	}
	
	ioctl(devfd, SOUND_PCM_WRITE_BITS, &bits);
	ioctl(devfd, SOUND_PCM_WRITE_CHANNELS, &channels);
	ioctl(devfd, SOUND_PCM_WRITE_RATE, &rate);
	satfi_log("devname:%s type:%d devfd:%d", devname, type, devfd);
	return devfd;
}

SpeexPreprocessState * SpeexPreprocessStateInit(void)
{
	SpeexPreprocessState *st;
	st = speex_preprocess_state_init(NN, RATE);
	
	int vad = 1;
	int vadProbStart = 90;
	int vadProbContinue = 100;
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_VAD, &vad); //静音检测
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_START , &vadProbStart); //Set probability required for the VAD to go from silence to voice
	speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_CONTINUE, &vadProbContinue); //Set probability required for the VAD to stay in the voice state (integer percent)
	return st;
}

void SpeexPreprocessStateDestroy(SpeexPreprocessState *st)
{
	speex_preprocess_state_destroy(st);	
}

typedef struct _card_playback_info
{
	char *DevName;
	int *BroadSocket;
	int CmdPttOn;
	int CmdPttOff;
}Card_PlayBack_Info;

typedef struct _card_record_info
{
	char *DevName;
	int *BroadSocket;
	char *BroadAddr;
	int BroadPort;
}Card_Record_Info;

static void *card_record_thread(void *arg)
{
	int i=0;
	int ret;
	int vad;
	char buf[320];
	char enc_byte[320];
	Card_Record_Info *card_record_info = (Card_Record_Info *)arg;

	int *BroadSocket = card_record_info->BroadSocket;
	
	void *enc_state;
	int nbBytes;
	SpeexBits enc_bits;
	
	int recordfd = card_init(card_record_info->DevName, RECORD);

	speex_bits_init(&enc_bits);
	enc_state = speex_encoder_init(&speex_nb_mode);
	int quality = 8;
	speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY, &quality);
	
	SpeexPreprocessState *st = SpeexPreprocessStateInit();

	struct sockaddr_in mcast_addr;
	memset(&mcast_addr, 0, sizeof(mcast_addr));								/*初始化IP多播地址为0*/
	mcast_addr.sin_family 	   = AF_INET; 									/*设置协议族类行为AF*/
	mcast_addr.sin_addr.s_addr = inet_addr(card_record_info->BroadAddr);	/*设置多播IP地址*/
	mcast_addr.sin_port 	   = htons(card_record_info->BroadPort); 		/*设置多播端口*/
	int len = sizeof(mcast_addr);

	while(1)
	{
		if(recordfd > 0)
		{
			if((ret = read(recordfd, buf, 320)) <= 0)
			{
				satfi_log("read of %s failed", card_record_info->DevName);
				close(recordfd);
				recordfd = -1;
				continue;
			}
		}
		else
		{
			seconds_sleep(1);
			recordfd = card_init(card_record_info->DevName, RECORD);
			continue;
		}

		if(*BroadSocket > 0)
		{
			vad = speex_preprocess_run(st, (spx_int16_t *)buf);
			if(vad)
			{
				//speex_bits_reset(&enc_bits);
				//speex_encode_int(enc_state, (short *)buf, &enc_bits);
				//nbBytes = speex_bits_write(&enc_bits, enc_byte, ret);
				//if((i++)%50 == 0)satfi_log("sendto %s %s %d\n", card_record_info->DevName, card_record_info->BroadAddr, card_record_info->BroadPort);
				if(sendto(*BroadSocket, buf, ret, 0, (struct sockaddr*)&mcast_addr, len) < 0)
				{
					satfi_log("card_record_thread sendto error %d\n", *BroadSocket);
					close(*BroadSocket);
					*BroadSocket = -1;
				}
			}
		}
		else
		{
			satfi_log("%s %s BroadSocket=%d", __FUNCTION__, card_record_info->DevName, *BroadSocket);
			seconds_sleep(1);			
		}
	}

	SpeexPreprocessStateDestroy(st);
	return NULL;
}

static void *card_playback_thread(void *arg)
{
	int i=0;
	int ret;
	fd_set fds;
	struct timeval timeout={3,0};
	int pptstat=0;

	char buf[320];
	char dec[320];
	Card_PlayBack_Info *card_playback_info = (Card_PlayBack_Info *)arg;

	int *BroadSocket = card_playback_info->BroadSocket;
	
	struct sockaddr_in cli_addr; 		/*本地地址*/
	int addr_len = sizeof(cli_addr);

	void *dec_state;
	SpeexBits dec_bits;
	
	speex_bits_init(&dec_bits);
	dec_state = speex_decoder_init(&speex_nb_mode);
	int playback = card_init(card_playback_info->DevName, PLAYBACK);

	led_ctl(card_playback_info->CmdPttOff);
	
	while(1)
	{
		if(*BroadSocket > 0)
		{
			FD_ZERO(&fds);/* 每次循环都需要清空 */
			FD_SET(*BroadSocket, &fds); /* 添加描述符 */
			timeout.tv_sec = 2; 	
			
			switch(select(*BroadSocket+1, &fds, NULL, NULL, &timeout))
			{
				case -1: break;
				case  0: 
				{
					if(pptstat)
					{
						satfi_log("%s PttOff=%d",card_playback_info->DevName, card_playback_info->CmdPttOff);
						led_ctl(card_playback_info->CmdPttOff);
						pptstat = 0;
					}
				}
				break;
				
				default:
				{
					ret = recvfrom(*BroadSocket, buf, 320, 0, (struct sockaddr*)&cli_addr, &addr_len);
					if( ret <= 0)
					{
						satfi_log("%s recvfrom() error %d", __FUNCTION__, *BroadSocket);
						close(*BroadSocket);
						*BroadSocket = -1;
					}
					else
					{
						//speex_bits_reset(&dec_bits);
						//speex_bits_read_from(&dec_bits, buf, ret);
						//speex_decode_int(dec_state, &dec_bits, (short *)dec);
			
						if(!pptstat)
						{
							satfi_log("%s PttOn=%d", card_playback_info->DevName, card_playback_info->CmdPttOn);
							led_ctl(card_playback_info->CmdPttOn);
							pptstat = 1;
						}
						//if((i++)%50 == 0)satfi_log("recvfrom %s %s %d", card_playback_info->DevName, inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));					
						if((ret = write(playback, buf, 320)) <= 0)
						{
							satfi_log("write of %s failed", card_playback_info->DevName);
							close(playback);
							playback = card_init(card_playback_info->DevName, PLAYBACK);
							seconds_sleep(1);
						}
					}
				}
				break;
			}
		}
		else
		{
			satfi_log("%s %s BroadSocket=%d", __FUNCTION__, card_playback_info->DevName, *BroadSocket);
			seconds_sleep(1);
		}
	}
	
	return NULL;
}

void CardBroadCast_fork(void)
{
	pthread_t tID1;
	pthread_t tID2;
	pthread_t tID3;
	pthread_t tID4;
	
	prctl(PR_SET_PDEATHSIG, SIGKILL);//父进程退出发送SIGKILL 给子进程
	signal(SIGPIPE,SignalHandler);

	int broadSocket1 = -1;
	int broadSocket2 = -1;

	Card_PlayBack_Info card1_playback_info;
	card1_playback_info.DevName 	= CARD1;
	card1_playback_info.BroadSocket = &broadSocket1;
	card1_playback_info.CmdPttOn 	= PTT1_ON;
	card1_playback_info.CmdPttOff 	= PTT1_OFF;
	
	Card_Record_Info card1_record_info;
	card1_record_info.DevName = CARD1;
	card1_record_info.BroadSocket = &broadSocket1;
	card1_record_info.BroadAddr = MCAST_ADDR_CARD1;
	card1_record_info.BroadPort = MCAST_PORT_CARD1;

	Card_PlayBack_Info card2_playback_info;
	card2_playback_info.DevName 	= CARD2;
	card2_playback_info.BroadSocket = &broadSocket2;
	card2_playback_info.CmdPttOn 	= PTT2_ON;
	card2_playback_info.CmdPttOff 	= PTT2_OFF;

	Card_Record_Info card2_record_info;
	card2_record_info.DevName = CARD2;
	card2_record_info.BroadSocket = &broadSocket2;
	card2_record_info.BroadAddr = MCAST_ADDR_CARD2;
	card2_record_info.BroadPort = MCAST_PORT_CARD2;

	if(pthread_create(&tID1, NULL, card_record_thread, (void *)&card1_record_info) == -1) exit(1);
	if(pthread_create(&tID2, NULL, card_playback_thread, (void *)&card1_playback_info) == -1) exit(1);
	if(pthread_create(&tID3, NULL, card_record_thread, (void *)&card2_record_info) == -1) exit(1);
	if(pthread_create(&tID4, NULL, card_playback_thread, (void *)&card2_playback_info) == -1) exit(1);

	while(1)
	{
		if(checkroute(ROUTE, MCAST_ADDR_CARD2, 1) == 0)
		{
			if(broadSocket2 < 0)
			{
				broadSocket2 = broadcast_init(MCAST_ADDR_CARD2, MCAST_PORT_CARD2);	
			}
		}
		else
		{
			char cmd[128] = {0};
			sprintf(cmd, "route add -net %s netmask 255.255.255.255 %s", MCAST_ADDR_CARD2, ROUTE);
			satfi_log("%s", cmd);
			myexec(cmd, NULL, NULL);
			if(broadSocket2 > 0)
			{
				close(broadSocket2);
				broadSocket2 = -1;
			}
		}

		if(checkroute(ROUTE, MCAST_ADDR_CARD1, 1) == 0)
		{
			if(broadSocket1 < 0)
			{
				broadSocket1 = broadcast_init(MCAST_ADDR_CARD1, MCAST_PORT_CARD1);
			}
		}
		else
		{
			char cmd[128] = {0};
			sprintf(cmd, "route add -net %s netmask 255.255.255.255 %s", MCAST_ADDR_CARD1, ROUTE);
			satfi_log("%s", cmd);
			myexec(cmd, NULL, NULL);
			if(broadSocket1 > 0)
			{
				close(broadSocket1);
				broadSocket1 = -1;
			}
		}
		
		seconds_sleep(1);
	}
}

int main(int argc, char *argv[])
{
	int status;
	pid_t fpid;
	
	//signal(SIGCHLD,SIG_IGN);
	//等效于ulimit -c unlimited,为了程序发生错误时产生coredump文件
    struct rlimit coreFileSize;
    bzero(&coreFileSize, sizeof(coreFileSize));
    coreFileSize.rlim_cur = 1000*1024;
    coreFileSize.rlim_max = 4294967295UL;
    setrlimit(RLIMIT_CORE, &coreFileSize);

	while(1)
	{
		fpid = fork();
		if(fpid < 0)
		{
			satfi_log("fork error");
			exit(0);
		}
		else if(fpid == 0)
		{
			main_fork();
			//CardBroadCast_fork();
		}
		
		wait(&status);
		
		if(WIFEXITED(status))
		{
			satfi_log("WIFEXITED %d\n",WEXITSTATUS(status));//EXIT
			if(WEXITSTATUS(status) == 8)
			{
				//程序版本更新完成，重启
				if(isFileExists("/etc/init.d/satfi"))
				{
					satfi_log("/etc/init.d/satfi restart %d", __LINE__);
					myexec("/etc/init.d/satfi restart", NULL, NULL);
				}
				exit(0);
			}
			else
			{
				satfi_log("SATFI EXIT %d\n",__LINE__);//EXIT
				exit(0);
			}
		}
		else if(WIFSIGNALED(status))
		{
			satfi_log("WIFSIGNALED %d\n",WTERMSIG(status));//SIG
		}
		else if(WIFSTOPPED(status))
		{
			satfi_log("WIFSTOPPED %d\n",WSTOPSIG(status));
		}
		else
		{
			satfi_log("SATFI EXIT %d\n",__LINE__);
		}
	}
	
	return 0;
}

