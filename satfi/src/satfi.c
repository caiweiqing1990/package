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

#include"log.h"
#include "timer.h"
#include "msg.h"
#include "led_control.h"
#include "server.h"

#define GPS_DATA_FILE		"/GpsData.txt"
#define CALL_RECORDS_FILE	"/CallRecords.txt"

#define USERID_LLEN 21  //完整的用户ID长度
#define USERID_LEN  12  //上传TSC服务器时，只需要传送用户ID的后12位，以节约流量
#define IMSI_LEN    15  //IMEI和IMSI的长度

pthread_mutex_t sat_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t n3g_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t net_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t tsc_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t pack_mutex = PTHREAD_MUTEX_INITIALIZER;

//void printsocketbufsize(int socket, char *prefix);

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
  SAT_SIM_NOT_INSERTED,
  SAT_SIM_ARREARAGE,//欠费
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
  int forbid_dial;           //是否禁止拨号 0准许 1禁止
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
  int sat_available;
  int sat_status;            //0：command 1：online data
  enum SAT_STATE sat_state;
  enum SAT_STATE_PHONE sat_state_phone;
  int sat_wait_gps;			//是否等待获取到卫星GPS再拨号 0:1 是:否
  int sat_hb_seconds;
  int sat_baud_rate;         //波特率
  int sat_csq_limit_value;   //信号阈值
  int sat_csq_value;         //信号强度
  int sat_csq_ltime;         //得到信号强的时间
  int sat_calling;           //是否正在进行呼叫
  int sat_dialing;           //是否正在尝试拨号
  int forbid_dial;           //是否禁止拨号 0准许 1禁止
  short sat_mode;			 //0:1 工作模式 调试模式
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
  
  int usbSerialStm32;
  int serverFd;
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

typedef struct _user {
  int socketfd;
  char userid[USERID_LLEN];           		//用户ID
  pthread_mutex_t msg_mutex;
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
	struct Pack *next;
}PACK;

typedef  struct CallRecords{
	char* Data;
	struct CallRecords *next;
}CALLRECORDS;

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
static CALLRECORDS *CallRecordsHead = NULL;	//通话记录
static LOG *gp_log = NULL;
static USER *gp_users = NULL;				//用户列表
static int iCntUserSave = 0;

//发送给TSC的GPS数据，非标准格式
char GpsData[256] = "$GPRMC,140039.000,A,2300.00000,N,11300.0000,E,0.0,2000.0,A*60,00,00,00000000,";
//发送给TSC的心跳间隔时间
static int tsc_hb_timeout = 600;

int bSatNetWorkActive = 1;//默认激活
int bTscConnected = 0;
int bGetGpsData = 0;
//int sock_app_udp = -1;
int sock_tsc = -1;
int sock_udp = -1;
int app_socket_voice = -1;
int isNeedReset = 1;
char FixMsID[10] = {0};

/* 同步锁
 *
 */
void n3g_lock() { pthread_mutex_lock(&n3g_mutex); }
void n3g_unlock() { pthread_mutex_unlock(&n3g_mutex); }
void sat_lock() { pthread_mutex_lock(&sat_mutex); }
void sat_unlock() { pthread_mutex_unlock(&sat_mutex); }
void net_lock() { pthread_mutex_lock(&net_mutex); }
void net_unlock() { pthread_mutex_unlock(&net_mutex); }

void HangingUp(void);
void AnsweringPhone(void);
void StartCallUp(char calling_number[15]);

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
			if(n != p->length || n < 0)
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

	satfi_log("SaveGpsToFile imsi:%.15s,Lg:%d,Lg_D:%c,Lt:%d,Lt_D:%c,Date:%llu,Speed:%d\n",
		req->Sat_IMSI, req->Lg, req->Lg_D, 
		req->Lt, req->Lt_D, req->Date, 
		req->Speed);

	SaveDataToFile(FileName , buf, req->header.length);
	
	if(base.gps.Lg == 0)
	{
		satfi_log("NOT GPS DATA %d %d %c %c", base.gps.Lg, base.gps.Lt, base.gps.Lg_D, base.gps.Lt_D);
	}

	return 0;
}

int ReadCallRecordsAndSendToTSC(char *FileName)
{
	Header *pHeader = NULL;
	int ret;
	int fd;
	int filesize;
	unsigned char buf[1024] = {0};
	static int offset = 0;
	
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
	
	pHeader = (Header *)&(data[offset]);
	satfi_log("ReadCallRecordsAndSendToTSC 0x%04x %d %d\n",pHeader->mclass, pHeader->length, offset);
	write(sock_tsc, pHeader, pHeader->length);
	offset += pHeader->length;
	
	munmap(data, filesize);
	close(fd);

	if(offset == filesize)
	{
		satfi_log("remove %s\n", FileName);
		offset = 0;
		remove(FileName);
	}

	return 0;
}

int DelCallRecordsFile(char *FileName)
{
	satfi_log("remove %s\n", FileName);
	return remove(FileName);
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
	write(sock_tsc, pHeader, pHeader->length);
	satfi_log("ReadGpsDataAndSendToTSC 0x%04x %d filesize=%d\n",pHeader->mclass, pHeader->length, filesize);
	
	munmap(data, filesize);
	close(fd);

	return 0;
}

int SendALLGpsDataToTSCNoWaitAndDel(void)
{
	GPSDATA *p = GpsDataHead;
	GPSDATA *q = NULL;
	Header *pHeader = NULL;

	int cnt = 0;

	if(GpsDataHead == NULL)
	{
		satfi_log("No GpsData");
	}
	
	while(p)
	{
		pHeader = (Header *)p->Data;
		//satfi_log("SendGpsDataToTSC%d 0x%04x %d",cnt, pHeader->mclass, pHeader->length);
		write(sock_tsc, pHeader, pHeader->length);
		p = p->next;
		++cnt;
	}

	GpsDataDel();

	return 0;
}

int SendOneGpsDataToTSCAndDel(void)
{
	GPSDATA *p = GpsDataHead;
	Header *pHeader = NULL;

	if(GpsDataHead == NULL)
	{
		satfi_log("No GpsData");
	}
	else
	{
		pHeader = (Header *)p->Data;
		satfi_log("SendGpsDataToTSC 0x%04x %d",pHeader->mclass, pHeader->length);
		write(sock_tsc, pHeader, pHeader->length);
		GpsDataHead = p->next;
		
		free(p->Data);
		free(p);
	}

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

int GpsDataDel(void)
{
	GPSDATA *tmp = NULL;
	GPSDATA *p = GpsDataHead;
	GPSDATA *q = NULL;

	int cnt=0;

	while(p)
	{
		++cnt;
		satfi_log("GpsDataDel %d\n",cnt);
		q = p->next;
		free(p->Data);
		free(p);
		p = q;
	}

	GpsDataHead = NULL;
}


int SendCallRecordsToTSCAndDel(void)
{
	CALLRECORDS *p = CallRecordsHead;
	CALLRECORDS *q = NULL;
	Header *pHeader = NULL;

	int cnt = 0;

	if(CallRecordsHead == NULL)
	{
		satfi_log("No CallRecords");
	}
	
	while(p)
	{
		satfi_log("SendCallRecordsToTSC%d",cnt);
		pHeader = (Header *)p->Data;
		write(sock_tsc, pHeader, pHeader->length);
		p = p->next;
		++cnt;
	}

	CallRecordsDel();

	return 0;
}

int CallRecordsADD(char *packdata, unsigned short packsize)
{
	CALLRECORDS *tmp = NULL;
	CALLRECORDS *p = CallRecordsHead;
	CALLRECORDS *q = NULL;

	while(p)
	{
		q = p;
		p = p->next;
	}

	if(p == NULL)
	{
		tmp = calloc(1, sizeof(CALLRECORDS));
		if(tmp == NULL)
		{
			satfi_log("CallRecordsADD error %d\n",__LINE__);
			return -1;
		}
		
		tmp->Data = calloc(1, packsize);
		if(tmp->Data == NULL)
		{
			satfi_log("CallRecordsADD error %d\n",__LINE__);
			return -1;
		}
				
		memcpy(tmp->Data, packdata, packsize);
		tmp->next = NULL;

		if(CallRecordsHead == NULL)
		{
			satfi_log("CallRecordsADD packsize1=%d\n",packsize);
			CallRecordsHead = tmp;
		}
		else
		{
			satfi_log("CallRecordsADD packsize2=%d\n",packsize);
			q->next = tmp;
		}
	}

	return 0;
}

int CallRecordsDel(void)
{
	CALLRECORDS *tmp = NULL;
	CALLRECORDS *p = CallRecordsHead;
	CALLRECORDS *q = NULL;

	int cnt=0;

	while(p)
	{
		satfi_log("CallRecordsDel%d\n",cnt);
		q = p->next;
		free(p->Data);
		free(p);
		p = q;
		++cnt;
	}

	CallRecordsHead = NULL;
}

int Message_Pack_Add(char *packdata, unsigned short packsize, unsigned int Name)
{
	pthread_mutex_lock(&pack_mutex);
	PACK *tmp = NULL;
	PACK *p = PackHead;
	PACK *q = NULL;
	int cnt=0;

	while(p)
	{
		if(p->Name == Name)
		{
			satfi_log("Message_Pack Exist Name=%d\n", Name);
			break;
		}
		cnt++;
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
		tmp->next = NULL;

		if(PackHead == NULL)
		{
			satfi_log("Message_Pack_Add1 packsize=%d Name=%d %d\n",packsize, Name, cnt);
			PackHead = tmp;
		}
		else
		{
			satfi_log("Message_Pack_Add2 packsize=%d Name=%d %d\n",packsize, Name, cnt);
			q->next = tmp;
		}
	}
	pthread_mutex_unlock(&pack_mutex);
	return 0;
}

int Voice_Pic_Pack_Add(char *packdata, int packsize, unsigned short PackSeq, unsigned short Packtotal, unsigned int Name)
{
	PACK *tmp = NULL;
	PACK *p = NULL;
	PACK *q = NULL;
	int cnt=0;
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

		satfi_log("Voice_Pic_Pack_Add1 PackSeq=%d Packtotal=%d Name=%d\n",PackSeq,Packtotal,Name);
		
		tmp->Name = Name;
		tmp->PackSeq = PackSeq;
		tmp->Packtotal = Packtotal;
		memcpy(tmp->Data, packdata, packsize);
		tmp->offset = packsize;
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
			cnt++;
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

			satfi_log("Voice_Pic_Pack_Add2 PackSeq=%d Packtotal=%d Name=%d %d\n",PackSeq,Packtotal,Name,cnt);
			
			tmp->Name = Name;
			tmp->PackSeq = PackSeq;
			tmp->Packtotal = Packtotal;
			memcpy(tmp->Data, packdata, packsize);
			tmp->offset = packsize;
			tmp->next = NULL;
			
			q->next = tmp;
		}
		else if(PackSeq > p->PackSeq)
		{
			if(PackSeq == p->Packtotal)satfi_log("PackSeq=%d Packtotal=%d Name=%d\n",PackSeq, p->Packtotal, Name);
			//satfi_log("PackSeq=%d Name=%d Packtotal=%d packsize=%d\n",PackSeq, Name, p->Packtotal, packsize);
			int dataoffset = p->offset;
			memcpy(&(p->Data[dataoffset]), packdata, packsize);
			p->PackSeq = PackSeq;
			p->offset += packsize;
		}
		else
		{
			//satfi_log("Voice_Pic_Exist\n");
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

int SendPackToTSC(void)
{
	Header *pHeader = NULL;
	static unsigned int NameIDOld = 0;
	static int SendPackTime = 0;
	static int timeout = 0;
	static int PackLostTimeOut = 0;
	static short OldPackSeq = 1;
	unsigned int NameID = 0;
	
	if(PackHead != NULL)
	{
		unsigned int TimeNow = time(0);
		NameID = PackHead->Name;
		if(NameIDOld != NameID)
		{
			if(PackHead->PackSeq == 0 && PackHead->Packtotal == 0)
			{
				pHeader = (Header *)PackHead->Data;
				satfi_log("SendPackToTSC MESSAGE %d\n", NameID);
				timeout = 30;
				write(sock_tsc, pHeader, pHeader->length);
				NameIDOld = NameID;
				SendPackTime = time(0);
			}
			else if(PackHead->PackSeq == PackHead->Packtotal)
			{
				timeout = 180;
				int offset = 0;
				while(1)
				{
					pHeader = (Header *)&(PackHead->Data[offset]);
					//satfi_log("pic_voi %04x %d %d %d %d\n",pHeader->mclass, pHeader->length, NameID, offset, PackHead->offset);
					pthread_mutex_lock(&tsc_mutex);
					write(sock_tsc, pHeader, pHeader->length);
					pthread_mutex_unlock(&tsc_mutex);
					offset += pHeader->length;
					if(offset == PackHead->offset)
					{
						satfi_log("SendPackToTSC PIC_VOICE SEND END %d %d %d\n", NameID, PackHead->Packtotal, PackHead->offset);
						break;
					}
				}
				NameIDOld = NameID;
				SendPackTime = time(0);
			}
			else
			{
				satfi_log("PackSeq=%d Packtotal=%d PackLostTimeOut=%d NameID=%d",PackHead->PackSeq, PackHead->Packtotal, PackLostTimeOut, NameID);
				SendPackTime = 0;
				if(PackHead->PackSeq == OldPackSeq)
				{
					PackLostTimeOut++;
					if(PackLostTimeOut >= 30)
					{
						PackLostTimeOut = 0;
						satfi_log("PackLost %d", NameID);
						Pack_Del(NameID);
					}
				}
				else
				{
					PackLostTimeOut = 0;
					OldPackSeq = PackHead->PackSeq;
				}
			}
		}

		if(SendPackTime != 0 && TimeNow >= SendPackTime && TimeNow - SendPackTime > timeout)
		{
			satfi_log("SendPackTimeOut=%d %d %d\n", TimeNow, SendPackTime, timeout);
			SendPackTime = 0;
			NameIDOld = 0;
			close(sock_tsc);
			sock_tsc = -1;	
		}
	}
}

int log_insert(char *MsID, char *data, int datalen)
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
			satfi_log("get_log_data %.21s\n",MsID);
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
  //satfi_log("open serial port : %s ...\n", device);
  int fd_serial = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_serial < 0)
  {
    satfi_log("open %s faile",device);
    return -1;
  }

  *fd = fd_serial;

  /* 串口主要设置结构体termios <termios.h> */
  struct termios options;

  /* 1.tcgetattr()用于获取与终端相关的参数
   *   参数fd为终端的文件描述符，返回的结果保存在termios结构体中
   */
  tcgetattr(fd_serial, &options);

  /* 2.修改获得的参数 */
  options.c_cflag |= CLOCAL | CREAD; /* 设置控制模块状态：本地连接，接收使能 */
  options.c_cflag &= ~CSIZE;         /* 字符长度，设置数据位之前，一定要屏蔽这一位 */
  options.c_cflag &= ~CRTSCTS;       /* 无硬件流控 */
  options.c_cflag |= CS8;            /* 8位数据长度 */
  options.c_cflag &= ~CSTOPB;        /* 1位停止位 */
  options.c_iflag |= IGNPAR;         /* 无奇偶校验 */
  options.c_oflag = 0;               /* 输出模式 */
  options.c_lflag = 0;               /* 不激活终端模式 */
  cfsetospeed(&options, baud_rate);    /* 设置波特率 */

  /* 3.设置新属性: TCSANOW，所有改变立即生效 */
  tcflush(fd_serial, TCIFLUSH);      /* 溢出数据可以接收，但不读 */
  tcsetattr(fd_serial, TCSANOW, &options);

  //satfi_log("open serial port : %s successfully!!!\n", device);
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
		satfi_log("Set system datatime error!");  
		return -1;  
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
		satfi_log("sat_csq_ltime=%d t=%d tN=%d %d", base.sat.sat_csq_ltime, t, tN, t - tN);
		if(base.sat.sat_csq_ltime > 0) base.sat.sat_csq_ltime += (t - tN);

		APPSOCKET *pApp = gp_appsocket;
		while(pApp)
		{
			pApp->Update += (t - tN);
			pApp = pApp->next;
		}

		if(SetSystemTime(t) == 0) isSetTime = 1;
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
  if(strlen(base.n3g.n3g_imei)==0) status = 2;       //没有读到IMEI
  else if(strlen(base.n3g.n3g_imsi)==0 || base.n3g.n3g_state == N3G_SIM_NOT_INSERTED) status = 3;  //没有读到IMSI
  else if(base.n3g.n3g_dialing == 1 || 
  	base.n3g.n3g_state == N3G_STATE_DIALING) status = 1;     //正在拨号
  else if(base.n3g.n3g_state == N3G_STATE_CSQ ||
          base.n3g.n3g_state == N3G_STATE_CSQ_W)
  {
    status = 4; //信号强度不够
  }
  else if(base.n3g.n3g_status == 1) status = 0;
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
  else if(base.sat.sat_mode == 1) status = 7;			  //调试模式
  else if(base.sat.sat_status == 1) status = 0;			  //工作正常
  else if(strlen(base.sat.sat_imei)==0) status = 2;       //没有读到IMEI
  else if(strlen(base.sat.sat_imsi)==0 || 
  	base.sat.sat_state == SAT_SIM_NOT_INSERTED) status = 3;  //没有读到IMSI
  else if(base.sat.sat_calling == 1) status = 6;     //正在进行电路域的呼叫
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
  //if(base.sat.sat_calling == 1 && base.sat.sat_state_phone == SAT_STATE_PHONE_IDLE) status = 2;
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

int Check3GNetLink(char* ip, int port)
{
	int Sock;
	Sock = ConnectTSC(base.n3g.n3g_ifname, ip, port, NULL, 10);
	if(Sock > 0)
	{
		close(Sock);
	}
	return Sock;
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

	while(1)
	{
		n3g_lock();
		//判断3g module是否上电正常
		if (!isFileExists(base->n3g.n3g_dev_name) || !isFileExists("/dev/ttyGPRS"))
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

			if(base->n3g.n3g_fd > 0)
			{
				close(base->n3g.n3g_fd);
				base->n3g.n3g_fd = -1;
			}
			
			satfi_log("power_mode gprs off\n");
			myexec("power_mode gprs off", NULL, NULL);
			satfi_log("power_mode gprs on\n");
			myexec("power_mode gprs on", NULL, NULL);			
			seconds_sleep(30);
			continue;
		}

		if (detect_interface(base->n3g.n3g_ifname) == 0)
		{
			uart_send(base->n3g.n3g_fd, "AT+CSQ\r\n", 8);
			if(base->n3g.n3g_status == 0)
			{
				char buf[256];
				sprintf(buf, "route add default dev %s", base->n3g.n3g_ifname);
				satfi_log("%s %d\n",buf, __LINE__);
				myexec(buf, NULL, NULL);
				
				base->n3g.n3g_status = 1;
				base->n3g.n3g_state = N3G_STATE_IDLE;
				base->n3g.n3g_dialing = 0;
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
			init_serial(&base->n3g.n3g_fd, base->n3g.n3g_dev_name, base->n3g.n3g_baud_rate);
			
			if(base->n3g.n3g_fd>0)
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
	int dialingcount = 0;

	int checkaddr = 0;
	int iscalled = 0;

	while(1)
	{
		sat_lock();
		if(bSatNetWorkActive == 0)
		{
			if(checkroute(base->sat.sat_ifname,base->tsc.tsc_addr, 0) == 0)
			{
				char ucbuf[256];
				sprintf(ucbuf, "ifdown %s", base->sat.sat_ifname_a);
				satfi_log("%s %d\n",ucbuf, __LINE__);
				myexec(ucbuf, NULL, NULL);
			}
			sat_unlock();
			seconds_sleep(60);
			continue;
		}

		if(base->sat.sat_calling || base->sat.forbid_dial)
		{
			//satfi_log("sat_calling waiting 10 seconds...\n");
			if(base->sat.sat_calling)
			{
				iscalled = 1;
			}
			else
			{
				if(base->sat.sat_fd>=0)
				{
					close(base->sat.sat_fd);
					base->sat.sat_fd = -1;
				}
			}

			if(base->sat.sat_dialing)
			{
				char ucbuf[256];
				sprintf(ucbuf, "ifdown %s", base->sat.sat_ifname_a);
				satfi_log("%s %d\n",ucbuf, __LINE__);
				myexec(ucbuf, NULL, NULL);

				base->sat.sat_dialing = 0;
			}
			sat_unlock();
			seconds_sleep(10);
			continue;
		}
		else
		{
			if(iscalled && base->sat.sat_dialing)
			{
				char ucbuf[256];
				bzero(ucbuf,sizeof(ucbuf));
				sprintf(ucbuf, "ifup %s", base->sat.sat_ifname_a);
				satfi_log("%s %d\n",ucbuf, __LINE__);
				myexec(ucbuf, NULL, NULL);
				//base->sat.sat_dialing = 0;
				iscalled = 0;
				seconds_sleep(10);
			}
		}

		//判断sat module是否上电正常
		if (!isFileExists(base->sat.sat_dev_name))
		{
			satfi_log("%s is not exists,restarting sat module\n", base->sat.sat_dev_name);
			if(base->sat.sat_fd>=0)
			{
				close(base->sat.sat_fd);
				base->sat.sat_fd = -1;
			}
			base->sat.sat_state = SAT_STATE_IDLE;
			sat_unlock();

			char ucbuf[256];
			sprintf(ucbuf, "ifdown %s", base->sat.sat_ifname_a);
			satfi_log("%s %d\n",ucbuf, __LINE__);
			myexec(ucbuf, NULL, NULL);
			seconds_sleep(10);

			satfi_log("power_mode sat_sm2500 reset %d\n",__LINE__);
			myexec("power_mode sat_sm2500 reset", NULL, NULL);
			seconds_sleep(60);
			continue;
		}

		if (base->sat.sat_dialing && isFileExists("/var/log/pppsatlog"))
		{
			if(base->sat.sat_state != SAT_SIM_ARREARAGE)
			{
				if(isFileHaveString("/var/log/pppsatlog", "QoS not accepted"))
				{
					satfi_log("SAT_SIM_ARREARAGE\n");
					//base->sat.forbid_dial = 1;
					base->sat.sat_state = SAT_SIM_ARREARAGE;
					base->sat.forbid_dial = 1;				
				}
			}
			
			if(isFileHaveString("/var/log/pppsatlog", "Connect script failed"))
			{
				//Connect script failed
				base->sat.sat_dialing = 0;
			}
		}

		if(detect_interface(base->sat.sat_ifname) == 0)
		{
			if(checkroute(base->sat.sat_ifname,base->tsc.tsc_addr, checkaddr) == 0)
			{
				if(base->sat.sat_available == 0)
				{
					char buf[256] = {0};           
					sprintf(buf, "route add %s dev %s", base->tsc.tsc_addr, base->sat.sat_ifname);
					satfi_log("%s %d\n",buf, __LINE__);
					myexec(buf, NULL, NULL);
					checkaddr = 1;
					base->sat.sat_available = 1;

					if(GetIniKeyInt("route","OPEN","/etc/rc.local") == 1)
					{
						sprintf(buf, "route add default dev %s", base->sat.sat_ifname);
						satfi_log("%s %d\n",buf, __LINE__);
						myexec(buf, NULL, NULL);
					}
					else
					{
						int i;
						for(i=0;;i++)
						{
							char ucTmp[64]={0};
							char ucTmp1[64]={0};
							sprintf(ucTmp1, "IP%d", i+1);
							GetIniKeyString("iplist", ucTmp1, "/etc/rc.local", ucTmp);
							if(strlen(ucTmp) == 0) break;
							bzero(buf, sizeof(buf));
							sprintf(buf, "route add %s dev %s", ucTmp, base->sat.sat_ifname);
							satfi_log("%s %d\n",buf, __LINE__);
							myexec(buf, NULL, NULL);
						}
					}
			
					satfi_log("3g-wan8 add\n");
				}

				if(base->sat.sat_fd>=0)
				{
					close(base->sat.sat_fd);
					base->sat.sat_fd = -1;
				}

				base->sat.sat_wait_gps = 0;
				base->sat.sat_status = 1;
				base->sat.sat_dialing = 0;
				base->sat.sat_calling = 0;
				dialingcount = 0;
			}
			else
			{
				checkaddr = 0;
				base->sat.sat_available = 0;
				//base->sat.sat_status = 0;
			}

			if(base->sat.sat_mode == 1)
			{
				char ucbuf[256];
				sprintf(ucbuf, "ifdown %s", base->sat.sat_ifname_a);
				satfi_log("%s %d\n",ucbuf, __LINE__);
				myexec(ucbuf, NULL, NULL);
			}

			sat_unlock();
			seconds_sleep(10);
			continue;
		}

		//没有拨号成功或者没有路由
		base->sat.sat_available = 0;
		if(base->sat.sat_dialing)
		{
			sat_unlock();

			if(base->sat.sat_mode == 1)
			{
				base->sat.sat_dialing = 0;
				char ucbuf[256];
				sprintf(ucbuf, "ifdown %s", base->sat.sat_ifname_a);
				satfi_log("%s %d\n",ucbuf, __LINE__);
				myexec(ucbuf, NULL, NULL);
				continue;
			}

			if(++dialingcount>=30)
			{
				satfi_log("sat module dialing too much time, trying reset ...\n");
				net_lock();

				char ucbuf[256];
				sprintf(ucbuf, "ifdown %s", base->sat.sat_ifname_a);
				satfi_log("%s %d\n",ucbuf, __LINE__);
				myexec(ucbuf, NULL, NULL);
				seconds_sleep(10);

				satfi_log("power_mode sat_sm2500 reset %d\n",__LINE__);
				myexec("power_mode sat_sm2500 reset", NULL, NULL);
				seconds_sleep(10);
				net_unlock();

				if(base->sat.sat_state != SAT_SIM_ARREARAGE)
				{
					//base->sat.sat_wait_gps = 1;
				}

				base->sat.sat_status = 0;
				base->sat.sat_state = SAT_STATE_AT;
				base->sat.sat_dialing = 0;
				dialingcount = 0;
			}
			else
			{
				seconds_sleep(10);
				if (!isFileExists("/var/lock/LCK..ttySAT0"))
				{
					satfi_log("/var/lock/LCK..ttySAT0 Not Exists %d dialingcount=%d", counter,dialingcount);
					counter++;
					if(counter > 5)
					{
						counter=0;
						dialingcount=0;
						satfi_log("/etc/init.d/network restart %d", __LINE__);
						myexec("/etc/init.d/network restart", NULL, NULL);
						seconds_sleep(10);

						char ucbuf[256];
						sprintf(ucbuf, "ifup %s", base->sat.sat_ifname_a);
						satfi_log("%s %d\n",ucbuf, __LINE__);
						myexec(ucbuf, NULL, NULL);

						if(base->n3g.n3g_dialing || base->n3g.n3g_status)
						{
							char ucbuf[256];
							sprintf(ucbuf, "ifup %s", base->n3g.n3g_ifname_a);
							myexec(ucbuf, NULL, NULL);
						}
					}
				}
				else
				{
					counter=0;
					//satfi_log("/var/lock/LCK..ttySAT0 Exists dialingcount=%d", dialingcount);
				}
			}
			continue;
		}

		if(base->sat.sat_fd == -1)
		{
			char ucbuf[256];
			sprintf(ucbuf, "ifdown %s", base->sat.sat_ifname_a);
			satfi_log("%s %d\n",ucbuf, __LINE__);
			myexec(ucbuf, NULL, NULL);
			myexec("killall chat", NULL, NULL);
			seconds_sleep(5);

			satfi_log("init_serial(%s,%d)\n", base->sat.sat_dev_name, base->sat.sat_baud_rate);
			init_serial(&base->sat.sat_fd, base->sat.sat_dev_name, base->sat.sat_baud_rate);
			if(base->sat.sat_fd>=0)
			{
				base->sat.sat_state = 0;
				base->sat.sat_status = 0;
			}
			else
			{
				sat_unlock();
				seconds_sleep(1);
				continue;
			}
		}

		//satfi_log("sat module state = %d\n", base->sat.sat_state);
		switch(base->sat.sat_state)
		{
			case SAT_STATE_AT:
				satfi_log("func_y1:send AT to SAT Module\n");
				uart_send(base->sat.sat_fd, "AT\r\n", 4);
				base->sat.sat_state = SAT_STATE_AT_W;
				bzero(base->sat.sat_imei, sizeof(base->sat.sat_imei));
				bzero(base->sat.sat_imsi, sizeof(base->sat.sat_imsi));
				counter=0;
			break;
			case SAT_STATE_AT_W:
				satfi_log("func_y2:send AT to SAT Module\n");
				uart_send(base->sat.sat_fd, "AT\r\n", 4);
				base->sat.sat_state = SAT_STATE_AT_W;
				//counter=0;
			break;
			case SAT_STATE_IMEI:
				satfi_log("func_y:send AT+CGSN to SAT Module\n");
				uart_send(base->sat.sat_fd, "AT+CGSN\r\n", 9);
				base->sat.sat_state = SAT_STATE_IMEI_W;
				counter=0;
			break;
			case SAT_STATE_IMSI:
				satfi_log("func_y:send AT+CIMI to SAT Module\n");
				uart_send(base->sat.sat_fd, "AT+CIMI\r\n", 9);
				base->sat.sat_state = SAT_STATE_IMSI_W;
				counter=0;
			break;
			case SAT_STATE_GPSTRACK_START:
				satfi_log("func_y:send AT+GPSTRACK=1,0,16 to SAT Module\n");
				uart_send(base->sat.sat_fd, "AT+GPSTRACK=1,0,16\r\n",20);
				base->sat.sat_state = SAT_STATE_GPSTRACK_START_W;
				counter=0;
			break;
			case SAT_STATE_GPSTRACK_STOP:
				satfi_log("func_y:send AT+GPSTRACK=0,0,16 to SAT Module\n");
				uart_send(base->sat.sat_fd, "AT+GPSTRACK=0,0,16\r\n", 20);
				base->sat.sat_state = SAT_STATE_GPSTRACK_STOP_W;
				counter=0;
			break;
			case SAT_STATE_CSQ:
				//satfi_log("func_y:send AT+CSQ to SAT Module\n");
				uart_send(base->sat.sat_fd, "AT+CSQ\r\n", 8);
				base->sat.sat_state = SAT_STATE_CSQ_W;
				counter=0;
			break;
			case SAT_STATE_DIALING:
				satfi_log("func_y:SAT Module Trying to connect to GmPRS\n");
				if(base->sat.sat_mode == 1)
				{
					//satfi_log("sat_debug_mode\n");
					base->sat.sat_state = SAT_STATE_AT;
				}
				else
				{
					if(base->sat.sat_fd>=0)
					{
						close(base->sat.sat_fd);
						base->sat.sat_fd = -1;
					}
					base->sat.sat_state = SAT_STATE_IDLE;
					base->sat.sat_dialing = 1;
					dialingcount = 0;
					net_lock();
					char ucbuf[256];
					sprintf(ucbuf, "ifdown %s", base->sat.sat_ifname_a);
					satfi_log("%s %d isNeedReset=%d\n",ucbuf, __LINE__, isNeedReset);
					myexec(ucbuf, NULL, NULL);
					seconds_sleep(5);

					if(isNeedReset)
					{
						//satfi_log("power_mode sat_sm2500 reset %d\n",__LINE__);
						//myexec("power_mode sat_sm2500 reset", NULL, NULL);
						//seconds_sleep(10);
						isNeedReset = 1;
					}

					satfi_log("remove /var/log/pppsatlog %d\n",__LINE__);
					remove("/var/log/pppsatlog");

					bzero(ucbuf,sizeof(ucbuf));
					sprintf(ucbuf, "ifup %s", base->sat.sat_ifname_a);
					satfi_log("%s %d\n",ucbuf,__LINE__);
					myexec(ucbuf, NULL, NULL);
					net_unlock();
				}

				counter=0;
			break;

			case SAT_SIM_NOT_INSERTED:
			case SAT_SIM_ARREARAGE:
				counter=0;
			break;
		}

		if(++counter>=30)
		{
			if(base->sat.sat_fd>=0)
			{
				satfi_log("close(base->sat.sat_fd)\n");
				close(base->sat.sat_fd);
				base->sat.sat_fd=-1;
			}

			//base->sat.sat_state = SAT_STATE_IDLE;
			sat_unlock();

			net_lock();

			if(base->sat.sat_dialing)
			{
				char ucbuf[256];
				sprintf(ucbuf, "ifdown %s", base->sat.sat_ifname_a);
				satfi_log("%s %d\n",ucbuf, __LINE__);
				myexec(ucbuf, NULL, NULL);
				seconds_sleep(10);
			}

			if(base->sat.sat_state == SAT_STATE_GPSTRACK_START_W)
			{
				satfi_log("power_mode sat_sm2500 off %d\n",__LINE__);
				myexec("power_mode sat_sm2500 off", NULL, NULL);
				seconds_sleep(10);

				satfi_log("power_mode sat_sm2500 on %d\n",__LINE__);
				myexec("power_mode sat_sm2500 on", NULL, NULL);
				seconds_sleep(10);

				base->sat.sat_wait_gps = 0;
			}
			else
			{
				satfi_log("power_mode sat_sm2500 reset %d\n",__LINE__);
				myexec("power_mode sat_sm2500 reset", NULL, NULL);
				seconds_sleep(10);
			}

			if(base->sat.sat_dialing)
			{
				char ucbuf[256];
				sprintf(ucbuf, "ifup %s", base->sat.sat_ifname_a);
				satfi_log("%s %d\n",ucbuf, __LINE__);
				myexec(ucbuf, NULL, NULL);
			}

			net_unlock();
			counter=0;
			continue;
		}

		sat_unlock();
		if(base->sat.sat_mode == 1)
		{
			milliseconds_sleep(500);
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
						satfi_log("N3G_SIM_NOT_INSERTED\n");
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

void handle_sat_data(int satfd)
{
	static char data[1024]={0};
	static int idx=0,flg=0,ncnt=0;
	int n;

	while(1)
	{
		if(idx==0)
		{
			memset(data,0,1024);
			flg=0;
			ncnt=0;
		}
		
		n = read(satfd, &data[idx], 1);
		if(n>0)
		{
			if(data[idx]=='\r') data[idx]='\n';

			if(data[idx]=='\n') ncnt++;
			else ncnt = 0;

			if(ncnt==4)
			{
				idx=2;
				flg=0;
				ncnt=0;
				continue;
			}

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
			else if(idx>4)
			{
				//printf("%s",data);
				if(strstr(data,"+RCIPH"))
				{
					if(base.sat.sat_state_phone == SAT_STATE_PHONE_ATD_W)
					{
						satfi_log("+RCIPH = %d\n", base.sat.sat_state_phone);
						base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING_CLCC;
					}
				}
				else if(strstr(data,"+GPSUTC"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						gps_parse(data);
						idx = 0;
						flg = 0;
						ncnt = 0;
					}
				}
				else if(strstr(data,"+GPSTRACKD"))
				{
					if(data[idx-1] == '\"')
					{
						flg++;
					}
					else if(data[idx-2]=='\n' && data[idx-1] == '\n' && flg==2)
					{
						if(parsegps(data, idx))
						{
							sat_lock();
							if(base.sat.sat_state==SAT_STATE_GPSTRACK_START_W)
							{
								int sat_gps_len = strlen(base.sat.sat_gps);
								if(sat_gps_len > sizeof(base.sat.sat_gps))
								{
									sat_gps_len = 256;
									base.sat.sat_gps[sat_gps_len] = 0;
								}
								else
								{
									base.sat.sat_gps[sat_gps_len - 2] = 0;
								}
								base.sat.sat_state = SAT_STATE_GPSTRACK_STOP;
							}
							sat_unlock();
						}
						
						if(base.sat.sat_wait_gps == 0 && base.sat.sat_state==SAT_STATE_GPSTRACK_START_W)
						{
							//base.sat.sat_state = SAT_STATE_CSQ;
							base.sat.sat_state = SAT_STATE_GPSTRACK_STOP;
						}
						idx = 0;
						flg = 0;
						ncnt=0;
					}
				}
				else if(strstr(data,"+GPSTRACKS"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						sat_lock();
						if(base.sat.sat_state==SAT_STATE_GPSTRACK_STOP_W)
						{
							base.sat.sat_state = SAT_STATE_CSQ;
						}
						sat_unlock();
						idx=0;flg=0;ncnt=0;
					}
				}
				else if(strstr(data,"+CSQ"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						static int sat_csq_good_cnt = 0;
						static int sat_csq_bad = 0;
						sat_lock();
						base.sat.sat_csq_value = parsecsq(data, idx);
						base.sat.sat_csq_ltime = time(0);
						satfi_log("sat_csq_value = %d\n", base.sat.sat_csq_value);
						if(base.sat.sat_csq_value != 99 && base.sat.sat_csq_value>=base.sat.sat_csq_limit_value)
						{
							//satfi_log("sat_csq_good_cnt %d %d\n", sat_csq_good_cnt, base.sat.sat_csq_value);
							sat_csq_good_cnt++;
							if(base.sat.sat_state==SAT_STATE_CSQ_W && sat_csq_good_cnt >= 1)
							{
								base.sat.sat_state = SAT_STATE_DIALING;
								sat_csq_good_cnt = 0;
							}
							else
							{
								base.sat.sat_state = SAT_STATE_CSQ;
							}

							NotifyAllUserSatState(SAT_LINK_NORMAL);
						}
						else
						{
							sat_csq_good_cnt = 0;
							
							if(base.sat.sat_state==SAT_STATE_CSQ_W)
							{
								base.sat.sat_state = SAT_STATE_CSQ;
							}

							if(base.sat.sat_csq_value == 99) base.sat.sat_csq_value = 0;
							
							NotifyAllUserSatState(SAT_LINK_DISCONNECT);
						}
												
						sat_unlock();
						idx=0;flg=0;ncnt=0;
					}
				}
				else if(strstr(data,"+CINFO"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						idx=0;ncnt=0;
					}
				}
				else if(strstr(data,"+RCIPH"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						idx=0;ncnt=0;
					}
				}
				else if(strstr(data,"RING"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						//sat_lock();
						//base.sat.sat_state_phone = SAT_STATE_PHONE_RING_COMING;
						//sat_unlock();	
						idx=0;ncnt=0;
					}
				}
				else if(strstr(data,"+CME ERROR"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						satfi_log("%s\n",data);
						//printf("%s\n",data);
						sat_lock();
						if(strstr(data,"SIM not inserted"))
						{
							base.sat.sat_state = SAT_SIM_NOT_INSERTED;
						}

						if(base.sat.sat_calling == 1)
						{
							base.sat.sat_state_phone = SAT_STATE_PHONE_DIALINGFAILE;
						}

						sat_unlock();	
						idx=0;ncnt=0;
					}
				}
				else if(strstr(data,"+CLCC"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						sat_lock();
						if(data[15] == '0')//通话中
						{
							//base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING_SUCCESS;
							base.sat.sat_state_phone = SAT_STATE_PHONE_ONLINE;
						}
						else if(data[15] == '2')//拨号中
						{
							if(base.sat.sat_state_phone != SAT_STATE_PHONE_DIALING_ATH_W)base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING;
						}
						else if(data[15] == '3')//振铃中
						{
							if(base.sat.sat_state_phone != SAT_STATE_PHONE_DIALING_ATH_W)base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING_RING;
						}
						else if(data[15] == '4')//有来电,提取来电电话号码
						{
							bzero(base.sat.called_number,15);
							base.sat.sat_state_phone = SAT_STATE_PHONE_RING_COMING;

							int i=0;
							for(i;i<15;i++)
							{
								if(data[i+25] == '"')
								{
									break;
								}
								
								base.sat.called_number[i] = data[i+25];
							}
							satfi_log("+CLCC called_number=%s %d\n", base.sat.called_number, strlen(base.sat.called_number));
						}
						sat_unlock();

						idx=0;ncnt=0;
					}
				}
				else if(strstr(data,"+CLIP"))
				{
					if(data[idx-2]=='\n' && data[idx-1]=='\n')
					{
						//strncpy(base.sat.called_number, &data[10],11);
						idx=0;ncnt=0;
					}
				}
				else if(strstr(data,"\n\nOK\n\n"))
				{
					sat_lock();
					if(base.sat.sat_state==SAT_STATE_AT_W)
					{
						base.sat.sat_state = SAT_STATE_IMEI;
					}

					if(base.sat.sat_state_phone==SAT_STATE_PHONE_CLCC) 
					{
						base.sat.sat_state_phone = SAT_STATE_PHONE_CLCC_OK;//可进行拨号
					}
					else if(base.sat.sat_state_phone==SAT_STATE_PHONE_ATH_W) 
					{
						base.sat.sat_state_phone = SAT_STATE_PHONE_HANGUP;//已挂断
					}
					else if(base.sat.sat_state_phone==SAT_STATE_PHONE_DIALING_ATH_W) 
					{
						satfi_log("SAT_STATE_PHONE_DIALING_ATH_W\n");
						base.sat.sat_state_phone = SAT_STATE_PHONE_DIALINGFAILE;
					}
					else if(base.sat.sat_state_phone==SAT_STATE_PHONE_ATA_W) 
					{
						base.sat.sat_state_phone = SAT_STATE_PHONE_ONLINE;//已接通
					}
					else if(base.sat.sat_state_phone==SAT_STATE_PHONE_CLIP) 
					{
						base.sat.sat_state_phone = SAT_STATE_PHONE_CLIP_OK;
					}
					else 
					{
						//printf("none %d\n",base->sat.sat_state_phone);
						//base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
					}
					sat_unlock();
					idx=0;ncnt=0;
				}
				else if(strstr(data,"\n\nERROR\n\n"))
				{
					satfi_log("ERROR %d %d\n",base.sat.sat_state_phone,base.sat.sat_state);
					idx=0;ncnt=0;	

					if(base.sat.sat_calling == 1)
					{
						base.sat.sat_state_phone = SAT_STATE_PHONE_DIALING_ERROR;
					}

					if(base.sat.sat_state == SAT_STATE_IMSI_W)
					{
						base.sat.sat_state = SAT_SIM_NOT_INSERTED;
					}
						
				}
				else if(strstr(data,"\n\nNO CARRIER\n\n"))
				{
					satfi_log("NO CARRIER %d\n",base.sat.sat_state_phone);
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
					idx=0;ncnt=0;
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
					idx=0;ncnt=0;
				}
				else if(data[idx-2]=='\n' && data[idx-1]=='\n')
				{
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
							strncpy(base.sat.sat_imei, &data[2], 15);
							satfi_log("sat_imei:%s\n", base.sat.sat_imei);
						}
						else if(base.sat.sat_state == SAT_STATE_IMSI_W)
						{
							strncpy(base.sat.sat_imsi, &data[2], 15);
							satfi_log("sat_imsi:%s\n", base.sat.sat_imsi);
							if(base.sat.sat_imsi[0] == '4')
							{
								base.sat.forbid_dial = 1;
							}
							else
							{
								base.sat.sat_state = SAT_STATE_GPSTRACK_START;								
							}
						}
						sat_unlock();
					}
					idx=0;ncnt=0;
				}
			}
		}
		else
		{
			break;
		}
	}	

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
		satfi_log("sat_imei empty %s", base.n3g.n3g_imei);
		strncpy(&buf[4], base.n3g.n3g_imei, IMSI_LEN);
		//return -1;
	}

	pthread_mutex_lock(&tsc_mutex);
	int n = write(sock_tsc,buf,p->length);
	pthread_mutex_unlock(&tsc_mutex);
	if (n<0)
	{
		satfi_log("sendto tsc return error: errno=%d %s %d\n", errno, strerror(errno),__LINE__);
		close(sock_tsc);
		sock_tsc = -1;
		bTscConnected = 0;
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
	iCntUserSave = iCntUser;
	p->length = offset;

	satfi_log("Send HeartBeat Message to TSC Server iCntUser=%d users=%s length=%d %d\n", iCntUser, users, p->length,__LINE__);
	pthread_mutex_lock(&tsc_mutex);
	//printfhex(buf, p->length);
	int n = write(sock_tsc, buf, p->length);
	pthread_mutex_unlock(&tsc_mutex);
	if (n<=0)
	{
		satfi_log("sendto tsc return error: errno=%d %s %d\n", errno, strerror(errno),__LINE__);
		close(sock_tsc);
		sock_tsc = -1;
		bTscConnected = 0;
		return -1;
	}
	else
	{
		base.tsc.tsc_hb_req_ltime = time(0);	
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
	//printf("Send heartbeatSP Message to TSC Server\n");
	pthread_mutex_lock(&tsc_mutex);
	int n = write(sock_tsc,buf,p->length);
	pthread_mutex_unlock(&tsc_mutex);
	if (n<0)
	{
		satfi_log("sendto tsc return error: errno=%d %s %d\n", errno, strerror(errno),__LINE__);
		close(sock_tsc);
		sock_tsc = -1;
		bTscConnected = 0;
		return -1;
	}
	else
	{
		//base.tsc.tsc_hb_req_ltime = time(0);
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

	if(iCntUser == iCntUserSave && bTscConnected == 1)
	{
		//satfi_log("no updateMSList %d %d %d", iCntUser, iCntUserSave, bTscConnected);
		return;
	}
	else
	{
		satfi_log("SATFI_MS_LIST_CMD useridlist=%s\n", userlist);
	}

	if(sock_tsc < 0)
	{
		iCntUserSave = 0;
		return;
	}

	pthread_mutex_lock(&tsc_mutex);
	int n = write(sock_tsc,tmp,rep->header.length);
	pthread_mutex_unlock(&tsc_mutex);
	if (n<=0)
	{
		satfi_log("write tsc return error: errno=%d %s %d\n", errno, strerror(errno),__LINE__);
		close(sock_tsc);
		sock_tsc = -1;
		bTscConnected = 0;
		return;
	}

	iCntUserSave = iCntUser;
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

void add_user(char *msid, int socketfd, pthread_mutex_t *mutex)
{
	USER *pUser = gp_users;

	while(pUser)
	{
		if(strncmp(pUser->userid, msid, USERID_LLEN) == 0)
		{
			satfi_log("UPDATE FIND USER %.21s %d\n",msid,pUser->socketfd);
			pUser->socketfd = socketfd;
			strncpy(pUser->userid, msid, USERID_LLEN);
			pUser->msg_mutex = *mutex;
			break;
		}
		pUser = pUser->next;
	}

	if(pUser == NULL)
	{
		satfi_log("MALLOC ADD USER %.21s\n",msid);
		pUser = (USER *)malloc(sizeof(USER));
		if(pUser)
		{
			bzero(pUser,sizeof(USER));
			pUser->socketfd = socketfd;
			strncpy(pUser->userid, msid, USERID_LLEN);
			pUser->msg_mutex = *mutex;
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

int handle_app_msg_tcp(int socket, char *pack, pthread_mutex_t *app_msg_mutex)
{
	char tmp[4096] = {0};
	char userid[24] = {0};
	int n = 0;
	MsgHeader *p = (MsgHeader *)pack;
	//satfi_log("p->length=%d p->mclass=%04x\n",p->length, p->mclass);	

	switch(p->mclass)
	{
		case CONNECT_CMD:
		{
			if(p->length != sizeof(MsgAppConnectReq) || socket < 0)
			{
				//satfi_log("CONNECT_CMD DATA ERROR %d %d", p->length, sizeof(MsgAppConnectReq), socket);
				break;
			}

			int result = CheckProjectAppIsConnect(&pack[4]);

			satfi_log("CheckProjectAppIsConnect %d %d", result, socket);

			if(result == 0)
			{
				add_user(&pack[4],socket,app_msg_mutex);
			}

			if(base.sat.sat_state_phone != SAT_STATE_PHONE_IDLE && strncmp(&pack[4], base.sat.MsID, USERID_LLEN) == 0)
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
				pthread_mutex_lock(app_msg_mutex);
				n = write(socket, tmp, rsp->header.length);
				pthread_mutex_unlock(app_msg_mutex);
				if(n != rsp->header.length)
				{
					satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,rsp->header.length,__LINE__);
				}
				if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),__LINE__,socket);
			}
		}
		break;
		case HEART_BEAT_CMD:
		{
			if(p->length != sizeof(MsgAppHeartbeatReq))
			{
				satfi_log("HEART_BEAT_CMD DATA ERROR");
				break;
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
				pthread_mutex_lock(app_msg_mutex);
				n = write(socket, tmp, rsp->header.length);
				pthread_mutex_unlock(app_msg_mutex);
				if(n != rsp->header.length)
				{
					satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,rsp->header.length,__LINE__);
				}
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
			pthread_mutex_lock(app_msg_mutex);
			n = write(socket, tmp, rsp->header.length);
			pthread_mutex_unlock(app_msg_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,rsp->header.length,__LINE__);
			}
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
					pthread_mutex_lock(&(pUser->msg_mutex));
					n = write(pUser->socketfd, pack, p->length);
					pthread_mutex_unlock(&(pUser->msg_mutex));
					if(n != p->length)
					{
						satfi_log("SENDLANMESSAGE1_CMD DATA ERROR n=%d p->length=%d %d\n",n,p->length,__LINE__);
					}
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
						pthread_mutex_lock(&(pUser->msg_mutex));
						n = write(pUser->socketfd, pack, p->length);
						pthread_mutex_unlock(&(pUser->msg_mutex));
						if(n != p->length)
						{
							satfi_log("SENDLANMESSAGE2_CMD DATA ERROR n=%d p->length=%d %d\n",n,p->length,__LINE__);
						}
						if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),__LINE__,pUser->socketfd);
					}
				}

				pUser = pUser->next;
			}
		}
		break;

		case SAT_MODE_SWITCH_CMD:
		{
			short sat_mode = *((short*)(&pack[4]));
			base.sat.sat_mode = sat_mode;
			base.sat.sat_wait_gps = sat_mode;
			satfi_log("SAT_MODE_SWITCH_CMD %d\n", sat_mode);
			MsgHeader *rsp = (MsgHeader*)tmp;
			rsp->length = 6;
			rsp->mclass = SAT_MODE_SWITCH_CMD_RSP;
			*((short*)(&tmp[4])) = sat_mode;

			pthread_mutex_lock(app_msg_mutex);
			n = write(socket, tmp, rsp->length);
			pthread_mutex_unlock(app_msg_mutex);
			if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),__LINE__,socket);
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
			
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("GRP_CHAT_CMD %s %d %d\n",userid, req->Name,req->header.length);

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
				satfi_log("SEND GRP_CHAT_CMD %s %d %d\n",userid, req->Name, GroupMessageIDNum);
				break;
			}
			else
			{
				GroupMessageID[GroupMessageIDNum] = req->Name;
				GroupMessageIDNum++;
				satfi_log("ADD GRP_CHAT_CMD %s %d %d\n",userid, req->Name, GroupMessageIDNum);
				if(GroupMessageIDNum == GROUP_MESSAGE_ID_NUM)
				{
					GroupMessageIDNum = 0;
				}
			}

			pthread_mutex_lock(app_msg_mutex);
			n = write(socket, tmp, rsp->header.length);
			pthread_mutex_unlock(app_msg_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,rsp->header.length,__LINE__);
			}
			if(n<0) satfi_log("write return error: errno=%d (%s) %d %d\n", errno, strerror(errno),__LINE__,socket);
		}
		break;

		case CALLUP_CMD:
		{
			short sat_state_phone = get_sat_dailstatus();

			MsgAppCallUpRsp rsp;
			rsp.header.length = sizeof(MsgAppCallUpRsp);
			rsp.header.mclass = CALLUP_RSP;
			rsp.result = sat_state_phone;
			pthread_mutex_lock(app_msg_mutex);
			n = write(socket, &rsp, rsp.header.length);
			pthread_mutex_unlock(app_msg_mutex);
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
				pthread_mutex_lock(app_msg_mutex);
				n = write(socket, &rsp, rsp.header.length);
				pthread_mutex_unlock(app_msg_mutex);
				
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
			satfi_log("ANSWERINGPHONE_CMD\n");
			//printf("ANSWERINGPHONE_CMD\n");
			if(base.sat.sat_calling == 1)
			{
				AnsweringPhone();
				
				MsgAppHangUpRsp rsp;
				rsp.header.length = sizeof(MsgAppHangUpRsp);
				rsp.header.mclass = ANSWERINGPHONE_RSP;
				rsp.result = 0;
				pthread_mutex_lock(app_msg_mutex);
				n = write(socket, &rsp, rsp.header.length);
				pthread_mutex_unlock(app_msg_mutex);
			}
		}
		break;
		
		case GET_LOG_CMD:
		{
			if(p = get_log_data(&pack[4]))
			{
				satfi_log("GET_LOG_CMD %.21s %d %x\n",&pack[4],p->length,p->mclass);
				pthread_mutex_lock(app_msg_mutex);
				n = write(socket, p, p->length);
				pthread_mutex_unlock(app_msg_mutex);
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

			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("APP_QUERY_GROUP_CMD %s %d %d\n",userid, socket,rsp->header.length);
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc, tmp, rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
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

			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("APP_QUERY_MS_CMD %s %d %d\n",userid, socket,rsp->header.length);
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc, tmp, rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
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

			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("APP_SET_BLOCK_CMD %s %d %d\n",userid, socket,rsp->header.length);
			//intf("APP_SET_BLOCK_CMD %s %d\n",userid,socket);

			//printf("APP_SET_BLOCK_CMD to TSC Server\n");
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc, tmp, rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case UPLOAD_MESSAGE_CMD:
		{
			int msglen;
			MsgUploadMessageReq *req = (MsgUploadMessageReq *)pack;
			msglen = req->header.length - ((int)req->Message - (int)req);
			//printf("MESSAGE msglen=%d %s %s\n",msglen, req->MsID, req->TargetGrpID);

			Msg_Upload_Message_Req* rsp = (Msg_Upload_Message_Req*)tmp;
			rsp->header.length = sizeof(Msg_Upload_Message_Req) + msglen;
			rsp->header.mclass = APP_UPLOAD_MESSAGE_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TargetGrpID, &(req->TargetGrpID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->ID = req->ID;
			if(msglen > 0)memcpy(rsp->Message, req->Message, msglen);

			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			//satfi_log("===================APP_UPLOAD_MESSAGE_CMD %s %d\n",userid, rsp->ID);
			//intf("APP_UPLOAD_MESSAGE_CMD %s %d\n",userid,socket);

			Message_Pack_Add(tmp, rsp->header.length, rsp->ID);
			break;
			
			if(sock_tsc < 0)
			{
				Message_Pack_Add(tmp, rsp->header.length, rsp->ID);
				break;
			}
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc, tmp, rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);		

		}
		break;

		case UPLOAD_VOICE_CMD:
		{				
			int datalen;
			MsgUploadVoiceReq *req = (MsgUploadVoiceReq *)pack;
			datalen = req->header.length - ((int)req->data - (int)req);
			//printf("VOICE datalen=%d\n",datalen);

			Msg_Upload_Voice_Req* rsp = (Msg_Upload_Voice_Req*)tmp;
			rsp->header.length = sizeof(Msg_Upload_Voice_Req) + datalen;
			rsp->header.mclass = APP_UPLOAD_VOICE_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TargetGrpID, &(req->TargetGrpID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->name = req->name;
			rsp->lengthtotal= req->lengthtotal;
			rsp->packseq = req->packseq;
			rsp->packtotal = req->packtotal;
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);

			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			if(rsp->packseq == 1)satfi_log("APP_UPLOAD_VOICE_CMD %s rsp->name=%d packtotal=%d\n",userid, rsp->name, rsp->packtotal);
			if(rsp->packseq == rsp->packtotal)satfi_log("APP_UPLOAD_VOICE_CMD %s rsp->name=%d packtotal=%d\n",userid, rsp->name, rsp->packtotal);

			Voice_Pic_Pack_Add(tmp, rsp->header.length, rsp->packseq, rsp->packtotal, rsp->name);
			break;

			if(sock_tsc < 0)
			{
				Voice_Pic_Pack_Add(tmp, rsp->header.length, rsp->packseq, rsp->packtotal, rsp->name);
				break;
			}
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc, tmp, rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);		
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
			rsp->name = req->name;
			rsp->lengthtotal= req->lengthtotal;
			rsp->packseq = req->packseq;
			rsp->packtotal = req->packtotal;
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);

			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			if(rsp->packseq == 1)satfi_log("APP_UPLOAD_PICTURE_CMD %s rsp->name=%d packtotal=%d %d\n", userid, rsp->name, rsp->packtotal, req->packseq);
			if(rsp->packseq == rsp->packtotal)satfi_log("APP_UPLOAD_PICTURE_CMD %s rsp->name=%d packtotal=%d %d\n", userid, rsp->name, rsp->packtotal, req->packseq);
			//if(rsp->packseq == 5){satfi_log("LOST pack rsp->packseq=%d", rsp->packseq);break;}
			//if(rsp->packseq == 5){satfi_log("LOST pack rsp->packseq=%d", rsp->packseq);break;}
			Voice_Pic_Pack_Add(tmp, rsp->header.length, rsp->packseq, rsp->packtotal, rsp->name);
			break;

			if(sock_tsc < 0)
			{
				//Voice_Pic_Pack_Add(tmp, rsp->header.length, rsp->packseq, rsp->packtotal, rsp->name);
				//break;
			}
			//printfhex(tmp, rsp->header.length);
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc, tmp, rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);

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

			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("APP_READ_OK_CMD %s %d %d\n",userid, socket,rsp->header.length);
			//intf("APP_READ_OK_CMD %s %d\n",userid, socket);

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc, tmp, rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
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

			if(sock_tsc < 0)
			{
				break;
			}
			
			//printf("get from app GET_MESSAGE_CMD\n");
			Msg_Get_Message_Req* rsp = (Msg_Get_Message_Req*)tmp;
			rsp->header.length = sizeof(Msg_Get_Message_Req);
			rsp->header.mclass = APP_GET_MESSAGE_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("get from app APP_GET_MESSAGE_CMD %s %d\n",userid, socket);
			//intf("get from app APP_GET_MESSAGE_CMD %s %d\n",userid, socket);

			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case ZF_MESSAGE_CMD:
		{
			int datalen;
			MsgZfMessageReq *req = (MsgZfMessageReq *)pack;
			datalen = req->header.length - ((int)req->Path - (int)req);
			//printf("MESSAGE datalen=%d\n",datalen);

			Msg_Zf_Message1_Req* rsp1 = (Msg_Zf_Message1_Req*)tmp;
			Msg_Zf_Message2_Req* rsp2 = (Msg_Zf_Message2_Req*)tmp;

			//printf("APP_ZF_MESSAGE_CMD req->Type=%d to TSC Server\n",req->Type);
			pthread_mutex_lock(&tsc_mutex);
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
				n= write(sock_tsc,tmp,rsp1->header.length);
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
				n= write(sock_tsc,tmp,rsp2->header.length);
				if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);	
			}
			pthread_mutex_unlock(&tsc_mutex);

			//strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			//printf("APP_ZF_MESSAGE_CMD %s\n",userid);
		}
		break;

		case EMERGENCY_ALARM_CMD:
		{
			//printf("get from app EMERGENCY_ALARM_CMD\n");
			MsgEmergencyAlarmReq *req = (MsgEmergencyAlarmReq *)pack;

			Msg_Emergency_Alarm_Req* rsp = (Msg_Emergency_Alarm_Req*)tmp;
			rsp->header.length = sizeof(Msg_Emergency_Alarm_Req);
			rsp->header.mclass = APP_EMERGENCY_ALARM_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Lg 		= req->Lg;
			rsp->Lt 		= req->Lt;
			rsp->AlarmType	= req->AlarmType;

			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("APP_EMERGENCY_ALARM_CMD %s %d %d\n",userid, socket,rsp->header.length);
			//intf("APP_EMERGENCY_ALARM_CMD %s %d\n",userid, socket);

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
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

			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("APP_CANCEL_EMERGENCY_ALARM_CMD %s %d %d\n",userid, socket,rsp->header.length);

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
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

			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("APP_OPERAT_CMD %s %d %d\n",userid, socket,rsp->header.length);

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case PHONE_MONEY_CMD:
		{
			MsgPhoneMoneyReq *req = (MsgPhoneMoneyReq *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("PHONE_MONEY_CMD %s %d %d\n",userid, socket,req->header.length);

			Msg_Phone_Money_Req *rsp = (Msg_Phone_Money_Req*)tmp;
			rsp->header.length = sizeof(Msg_Phone_Money_Req);
			rsp->header.mclass = APP_PHONE_MONEY_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->MsPhoneNum, req->MsPhoneNum, 11);
			StrToBcd(rsp->DesPhoneNum, req->DesPhoneNum, 11);
			rsp->StartTime = req->StartTime;
			rsp->CallTime = req->CallTime;
			rsp->Money = req->Money;

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc, tmp, rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case GRP_UPLOAD_MESSAGE_CMD:
		{
			MsgGrpUploadMessageReq *req = (MsgGrpUploadMessageReq *)pack;
			int msglen = req->header.length - ((int)req->Message - (int)req);
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("GRP_UPLOAD_MESSAGE_CMD %s %d %d\n",userid, socket,req->header.length);
			
			Msg_Grp_Upload_Message_Req *rsp = (Msg_Grp_Upload_Message_Req*)tmp;
			rsp->header.length = sizeof(Msg_Grp_Upload_Message_Req) + msglen;
			rsp->header.mclass = APP_GRP_UPLOAD_MESSAGE_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TargetGrpID, req->TargetGrpID, USERID_LLEN);
			rsp->ID = req->ID;
			if(msglen > 0)memcpy(rsp->Message, req->Message, msglen);

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
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
			rsp->name = req->name;
			rsp->lengthtotal= req->lengthtotal;
			rsp->packseq = req->packseq;
			rsp->packtotal = req->packtotal;
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);
			
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			if(rsp->packseq == 1 || rsp->packseq == rsp->packtotal)satfi_log("GRP_UPLOAD_VOICE_CMD %s %d %d\n",userid, socket,req->header.length);

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
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
			rsp->name = req->name;
			rsp->lengthtotal= req->lengthtotal;
			rsp->packseq = req->packseq;
			rsp->packtotal = req->packtotal;
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);
			
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			if(rsp->packseq == 1 || rsp->packseq == rsp->packtotal)satfi_log("GRP_UPLOAD_PICTURE_CMD %s %d %d %d\n",userid, datalen, rsp->header.length, req->header.length);
			
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case GRP_READ_OK_CMD:
		{
			MsgGrpReadOKReq *req = (MsgGrpReadOKReq *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("GRP_READ_OK_CMD %s %d %d %d\n",userid, socket,req->header.length,req->ID);

			Msg_Grp_Read_OK_Req *rsp = (Msg_Grp_Read_OK_Req*)tmp;
			rsp->header.length = sizeof(Msg_Grp_Read_OK_Req);
			rsp->header.mclass = APP_GRP_READ_OK_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->ID = req->ID;
			rsp->Type = req->Type;

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case SET_GRP_CHAT_CMD:
		{
			MsgSetGrpChatReq *req = (MsgSetGrpChatReq *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("SET_GRP_CHAT_CMD %s %d %d\n",userid, socket,req->header.length);

			Msg_Set_Grp_Chat_Req *rsp = (Msg_Set_Grp_Chat_Req*)tmp;
			rsp->header.length = sizeof(Msg_Set_Grp_Chat_Req);
			rsp->header.mclass = APP_SET_GRP_CAHT_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			rsp->Type = req->Type;
			rsp->MsgType = req->MsgType;

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case SET_MSG_CMD:
		{
			MsgSetMsgReq *req = (MsgSetMsgReq *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("SET_MSG_CMD %s %d %d\n",userid, socket,req->header.length);

			Msg_Set_Msg_Req *rsp = (Msg_Set_Msg_Req*)tmp;
			rsp->header.length = sizeof(Msg_Set_Msg_Req);
			rsp->header.mclass = APP_SET_MSG_CMD;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Type = req->Type;
			StrToBcd(rsp->TargetMsID, &(req->TargetMsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case WL_COMMAND2:
		{
			MsgWlCommand *req = (MsgWlCommand *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("WL_COMMAND2 %s %d %d\n",userid, socket,req->header.length);
			int Msglen = req->header.length - ((int)req->Msg - (int)req);

			Msg_Wl_Command *rsp = (Msg_Wl_Command*)tmp;
			rsp->header.length = sizeof(Msg_Wl_Command) + Msglen;
			rsp->header.mclass = APP_WL_COMMAND;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			if(Msglen > 0)memcpy(rsp->Msg, req->Msg, Msglen);
			
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case SET_ONLINE:
		{
			MsgSetOnlineReq *req = (MsgSetOnlineReq *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("SET_ONLINE %s %d %d\n",userid, socket, req->header.length);
			
			Msg_Set_Online_Req *rsp = (Msg_Set_Online_Req*)tmp;
			rsp->header.length = sizeof(Msg_Set_Online_Req);
			rsp->header.mclass = APP_SET_ONLINE;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			rsp->Type = req->Type;
			
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case GPS_POINT:
		{
			MsgGpsPointRsp *req = (MsgGpsPointRsp *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Gps_Point_Rsp *rsp = (Msg_Gps_Point_Rsp*)tmp;
			rsp->header.length = sizeof(Msg_Gps_Point_Rsp);
			rsp->header.mclass = APP_GPS_POINT_RSP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Id = req->Id;
			rsp->Result = req->Result;
			satfi_log("GPS_POINT %s %d %d\n",userid, rsp->header.length, req->header.length);

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case PTT:
		{
			MsgPtt *req = (MsgPtt *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Ptt *rsp = (Msg_Ptt*)tmp;
			rsp->header.length = sizeof(Msg_Ptt);
			rsp->header.mclass = APP_PTT;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			satfi_log("PTT %s %.21s %.21s\n",userid, req->MsID, req->GrpID);
			
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc, tmp, rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;

		case PTT_IN_RSP:
		{
			MsgPttInRsp *req = (MsgPttInRsp *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Ptt_In_Rsp *rsp = (Msg_Ptt_In_Rsp*)tmp;
			rsp->header.length = sizeof(Msg_Ptt_In_Rsp);
			rsp->header.mclass = APP_PTT_IN_RSP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Result = req->Result;
			satfi_log("PTT_IN_RSP  %s %.21s\n",userid, req->MsID);
			
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
		}
		break;
		
		case HUP:
		{
			MsgHup *req = (MsgHup *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Hup *rsp = (Msg_Hup*)tmp;
			rsp->header.length = sizeof(Msg_Hup);
			rsp->header.mclass = APP_HUP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			satfi_log("HUP %s %.21s %.21s\n",userid, req->MsID, req->GrpID);

			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);	
		}
		break;

		case SPTT:
		{
			MsgSptt *req = (MsgSptt *)pack;
			int datalen = req->header.length - ((int)req->data - (int)req);
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Sptt *rsp = (Msg_Sptt*)tmp;
			rsp->header.length = sizeof(Msg_Sptt) + datalen;
			rsp->header.mclass = APP_SPTT;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			rsp->MS_Count = req->MS_Count;
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);
			satfi_log("SPTT %s datalen=%d %d %d\n",userid, datalen, rsp->header.length, req->header.length);
			
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;

		case SPTT_HUP:
		{
			MsgSpttHup *req = (MsgSpttHup *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Sptt_Hup *rsp = (Msg_Sptt_Hup*)tmp;
			rsp->header.length = sizeof(Msg_Sptt_Hup);
			rsp->header.mclass = APP_SPTT_HUP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->GrpID, req->GrpID, USERID_LLEN);
			memcpy(rsp->Reserve, req->Reserve, sizeof(rsp->Reserve));
			satfi_log("SPTT_HUP %s %d %d\n",userid, rsp->header.length, req->header.length);
			
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;

		case MO_VOICE:
		{
			MsgMoVoice *req = (MsgMoVoice *)pack;
			int datalen = req->header.length - ((int)req->data - (int)req);
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Mo_Voice *rsp = (Msg_Mo_Voice*)tmp;
			rsp->header.length = sizeof(Msg_Mo_Voice) + datalen;
			rsp->header.mclass = APP_MO_VOICE;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			if(datalen > 0)memcpy(rsp->data, req->data, datalen);
			satfi_log("udp MO_VOICE %s datalen=%d %d %d\n",userid, datalen, rsp->header.length, req->header.length);

			struct sockaddr_in tsc_addr;
			bzero(&tsc_addr, sizeof(tsc_addr));
			tsc_addr.sin_family = AF_INET;
			tsc_addr.sin_port = htons(12056);
			tsc_addr.sin_addr.s_addr =  inet_addr(base.tsc.tsc_addr);
			sendto(sock_udp, tmp, rsp->header.length, 0, (struct sockaddr*)&tsc_addr, sizeof(tsc_addr));
			
			//if(sock_tsc<0)break;
			//pthread_mutex_lock(&tsc_mutex);
			//n= write(sock_tsc,tmp,rsp->header.length);
			//pthread_mutex_unlock(&tsc_mutex);
			//if(n != rsp->header.length)
			//{
			//	satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			//}
			//if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;

		case HUPIN_RSP:
		{
			MsgHupinRsp *req = (MsgHupinRsp *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Hupin_Rsp *rsp = (Msg_Hupin_Rsp*)tmp;
			rsp->header.length = sizeof(Msg_Hupin_Rsp);
			rsp->header.mclass = APP_HUPIN_RSP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Result = req->Result;
			
			satfi_log("HUPIN_RSP %s %d %d\n",userid, rsp->header.length, req->header.length);
			
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;
				
		case MS_HUP:
		{
			MsgMsHup *req = (MsgMsHup *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Ms_Hup *rsp = (Msg_Ms_Hup*)tmp;
			rsp->header.length = sizeof(Msg_Ms_Hup);
			rsp->header.mclass = APP_MS_HUP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			memcpy(rsp->Src_Name, req->Src_Name, sizeof(req->Src_Name));
			rsp->Operation = req->Operation;
			
			satfi_log("MS_HUP %s %.21s %.21s\n",userid, req->MsID, req->Src_Name);
			
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;

		case CLR_SPTT:
		{
			MsgClrSptt *req = (MsgClrSptt *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Clr_Sptt *rsp = (Msg_Clr_Sptt*)tmp;
			rsp->header.length = sizeof(Msg_Clr_Sptt);
			rsp->header.mclass = APP_CLR_SPTT;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			satfi_log("CLR_SPTT %s\n",userid);
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;

		case ASPTT_REQ:
		{
			MsgAsptt *req = (MsgAsptt *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Asptt *rsp = (Msg_Asptt*)tmp;
			rsp->header.length = sizeof(Msg_Asptt);
			rsp->header.mclass = APP_ASPTT_REQ;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TARGET_MS_Id, &(req->TARGET_MS_Id[USERID_LLEN - USERID_LEN]), USERID_LEN);
			memcpy(rsp->Reserve, req->Reserve, sizeof(rsp->Reserve));
			
			satfi_log("ASPTT_REQ %s\n",userid);
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;
		
		case ASPTT_REQ_RSP:
		{
			MsgAspttRsp *req = (MsgAspttRsp *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Asptt_Rsp *rsp = (Msg_Asptt_Rsp*)tmp;
			rsp->header.length = sizeof(Msg_Asptt_Rsp);
			rsp->header.mclass = APP_ASPTT_REQ_RSP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TARGET_MS_Id, &(req->TARGET_MS_Id[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Result = req->Result;
			
			satfi_log("ASPTT_REQ_RSP %s\n",userid);
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;
				
		case ASPTT_HUP:
		{
			MsgAspttHup *req = (MsgAspttHup *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
		
			Msg_Asptt_Hup *rsp = (Msg_Asptt_Hup*)tmp;
			rsp->header.length = sizeof(Msg_Asptt_Hup);
			rsp->header.mclass = APP_ASPTT_HUP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			memcpy(rsp->Reserve, req->Reserve, sizeof(rsp->Reserve));
			
			satfi_log("ASPTT_HUP %s\n",userid);
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;
		
		case ASPTT_HUP_RSP:
		{
			MsgAspttHupRsp *req = (MsgAspttHupRsp *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
		
			Msg_Asptt_Hup_Rsp *rsp = (Msg_Asptt_Hup_Rsp*)tmp;
			rsp->header.length = sizeof(Msg_Asptt_Hup_Rsp);
			rsp->header.mclass = APP_ASPTT_HUP_RSP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Result = req->Result;
			
			satfi_log("ASPTT_HUP_RSP %s\n",userid);
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;
						
		case ASPTT_CLER:
		{
			MsgAspttCler *req = (MsgAspttCler *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
		
			Msg_Asptt_Cler *rsp = (Msg_Asptt_Cler*)tmp;
			rsp->header.length = sizeof(Msg_Asptt_Cler);
			rsp->header.mclass = APP_ASPTT_CLER;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TARGET_MS_Id, &(req->TARGET_MS_Id[USERID_LLEN - USERID_LEN]), USERID_LEN);
			memcpy(rsp->Reserve, req->Reserve, sizeof(rsp->Reserve));
			
			satfi_log("ASPTT_CLER %s\n",userid);
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;
		
		case ASPTT_CLER_RSP:
		{
			MsgAspttClerRsp *req = (MsgAspttClerRsp *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
		
			Msg_Asptt_Cler_Rsp *rsp = (Msg_Asptt_Cler_Rsp*)tmp;
			rsp->header.length = sizeof(Msg_Asptt_Cler_Rsp);
			rsp->header.mclass = APP_ASPTT_CLER_RSP;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			rsp->Result = req->Result;
			
			satfi_log("ASPTT_CLER_RSP %s\n",userid);
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;
								
		case ASPTT_CANCEL:
		{
			MsgAspttCancel *req = (MsgAspttCancel *)pack;
			strncpy(userid, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);

			Msg_Asptt_Cancel *rsp = (Msg_Asptt_Cancel*)tmp;
			rsp->header.length = sizeof(Msg_Asptt_Cancel);
			rsp->header.mclass = APP_ASPTT_CANCEL;
			StrToBcd(rsp->MsID, &(req->MsID[USERID_LLEN - USERID_LEN]), USERID_LEN);
			StrToBcd(rsp->TARGET_MS_Id, &(req->TARGET_MS_Id[USERID_LLEN - USERID_LEN]), USERID_LEN);
			memcpy(rsp->Reserve, req->Reserve, sizeof(rsp->Reserve));
			
			satfi_log("ASPTT_CANCEL %s\n",userid);
			if(sock_tsc<0)break;
			pthread_mutex_lock(&tsc_mutex);
			n= write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);
			if(n != rsp->header.length)
			{
				satfi_log("DATA ERROR n=%d p->length=%d %d %d\n",n,rsp->header.length,__LINE__,sock_tsc);
			}
			if(n<0) satfi_log("write to tsc return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);				
		}
		break;
		
		default:
			satfi_log("received unrecognized message from app : mclass=0x%x length=%d\n", p->mclass,p->length);
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
				base.sat.sat_mode = 0;
				base.sat.sat_wait_gps = 0;
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
	struct timeval timeout={2,0};
	int max_fd = -1;
	int ret = -1;
	int new_conn_fd = -1;
	struct sockaddr_in cli_addr;  
	int len = sizeof(cli_addr);
	Header *pHeader = NULL;
	pthread_mutex_t app_msg_mutex = PTHREAD_MUTEX_INITIALIZER;

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
				//base->sat.captain_socket = new_conn_fd;
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
							//printf("%d %d %d\n",pApp->AppSocketFd, nread, pApp->DataSaveOffset);
							nread += pApp->DataSaveOffset;
							pApp->DataSaveOffset = 0;
							while(1)
							{
								pHeader = (Header *)&(pApp->Data[pApp->DataSaveOffset]);
								
								if((pApp->DataSaveOffset + pHeader->length) > nread)
								{
									satfi_log("%d %d %d %d %04x %d\n",pApp->AppSocketFd,nread,pApp->DataSaveOffset,pHeader->length,pHeader->mclass,nread - pApp->DataSaveOffset);
									memcpy(pApp->Data, pHeader, nread - pApp->DataSaveOffset);
									if(pHeader->mclass > APP_MAX_MCLASS)
									{
										pApp->DataSaveOffset = 0;
									}
									else
									{
										pApp->DataSaveOffset = nread - pApp->DataSaveOffset;
									}	
									break;
								}

								//printf("%04x\n",pHeader->mclass);
								if(handle_app_msg_tcp(pApp->AppSocketFd, (char*)pHeader, &app_msg_mutex) == -1)
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
							//updateMSList();
							satfi_log("FD_CLR %d\n",pApp->AppSocketFd);
							FD_CLR(pApp->AppSocketFd, &fdread);
							//close(pApp->AppSocketFd);
							//App_Remove(pApp->AppSocketFd);
						}
						//break;
					}

					pApp = pApp->next;
				}
			}
		}
	}
	return NULL;
}

void get_app_message_from_tsc(void)
{
	USER *pUser = gp_users;
	char tmp[512];
	
	while(pUser)
	{
		if(pUser->socketfd > 0)
		{
			Msg_Get_Message_Req* rsp = (Msg_Get_Message_Req*)tmp;
			rsp->header.length = sizeof(Msg_Get_Message_Req);
			rsp->header.mclass = APP_GET_MESSAGE_CMD;
			StrToBcd(rsp->MsID, &(pUser->userid[USERID_LLEN - USERID_LEN]), USERID_LEN);
			satfi_log("get_app_message_from_tsc %s %d\n",pUser->userid, pUser->socketfd);
			pthread_mutex_lock(&tsc_mutex);
			write(sock_tsc,tmp,rsp->header.length);
			pthread_mutex_unlock(&tsc_mutex);	
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

static void *select_tsc_udp(void *p)
{
	struct timeval tv = {3,0};	
	fd_set fds;
	int maxfd = 0;
	int n;
	char buf[2048];
	char tmp[2048];
	MsgHeader *pHeader = (MsgHeader *) buf;
	struct sockaddr_in cliAppAddr;	
	int len = sizeof(cliAppAddr);
	
	sock_udp = socket(AF_INET, SOCK_DGRAM, 0);
	if(sock_udp < 0)
	{
		satfi_log("socket : select_tsc_udp error");
		return NULL;
	}

	struct sockaddr_in server_addr;
	bzero(&server_addr, sizeof(server_addr));
	server_addr.sin_family	  	= AF_INET;
	server_addr.sin_port		= htons(12056);
	server_addr.sin_addr.s_addr =  htonl(INADDR_ANY);
	
	if(bind(sock_udp, (struct sockaddr*)&server_addr, sizeof(server_addr))<0)
	{
		satfi_log("bind : select_tsc_udp");
		return NULL;
	}

	while(1)
	{
		FD_ZERO(&fds);/* 每次循环都需要清空 */
		FD_SET(sock_udp, &fds); /* 添加描述符 */
		maxfd = sock_udp;
		FD_SET(sock_udp, &fds);
		tv.tv_sec = 3;
	    switch(select(maxfd+1,&fds,NULL,NULL,&tv))
	    {
	    case -1: break;
	    case  0: break;
	    default:
			{
				if(FD_ISSET(sock_udp, &fds))
				{
					n = recvfrom(sock_udp, buf, 2048, 0, (struct sockaddr *)&cliAppAddr, &len);
					if (n>0)
					{
						if(n == pHeader->length)
						{
							satfi_log("recv %d bytes,length=%d,mclass=%x\n", n, pHeader->length, pHeader->mclass);
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
									//satfi_log("APP_MT_VOICE %.21s %d\n", req->MsID, datalen);
								
									USER *pUser = gp_users;
									while(pUser)
									{
										if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
										{
											satfi_log("sendto app APP_MT_VOICE_UDP %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], datalen);
											if(pUser->socketfd < 0)break;
											pthread_mutex_lock(&(pUser->msg_mutex));
											n = write(pUser->socketfd, tmp, req->header.length);
											pthread_mutex_unlock(&(pUser->msg_mutex));
											if(n != req->header.length)
											{
												satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
												App_Remove(pUser->socketfd);
												close(pUser->socketfd);
												pUser->socketfd = -1;
												updateMSList();
											}
											if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
											break;
										}
										pUser = pUser->next;
									}
								
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

							}
						}
						else
						{
							satfi_log("bad pack recv %d bytes,length=%d,mclass=%x\n", n, pHeader->length, pHeader->mclass);
						}
					}
					else
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

static void *select_tsc(void *p)
{
	BASE *base = (BASE *)p;
	fd_set fds;
	char buf[8192];
	char tmp[8192];
	int maxfd = 0;
	int n;
	int nread;
	int len = sizeof(struct sockaddr_in);
	char userid[24] = {0};
	char grpID[24] = {0};
	int cnt = 0;
	unsigned short offset = 0;
	Header *pHeader;
	//char FixMsID[10] = "860311000";
	struct timeval timeout={3,0};
	int err;
	
	while(1)
	{
		if(base->sat.sat_available == 0 && base->n3g.n3g_status == 0)
		{
			if(base->gps.serverFd > 0)
			{
				close(base->gps.serverFd);
				base->gps.serverFd = -1;
			}
			
			if(sock_tsc > 0)
			{
				close(sock_tsc);
				sock_tsc = -1;
			}
			
			seconds_sleep(10);
			continue;
		}

		if(base->gps.serverFd <= 0)
		{
			base->gps.serverFd = ConnectTSC(NULL, base->omc.omc_addr, base->omc.omc_port, NULL, 5);
		}
		
		if(sock_tsc <= 0)
		{
			bTscConnected = 0;
			base->tsc.tsc_hb_req_ltime = 0;
			base->tsc.tsc_hb_rsp_ltime = 0;
			if((sock_tsc = ConnectTSC(NULL, base->tsc.tsc_addr, base->tsc.tsc_port, NULL, 5)) == -1)
			{
				iCntUserSave = 0;
			}
		}

		FD_ZERO(&fds);/* 每次循环都需要清空 */
		maxfd = 0;
		
		if(sock_tsc > 0)
		{
			FD_SET(sock_tsc, &fds); /* 添加描述符 */
			if(sock_tsc > maxfd) maxfd = sock_tsc;
		}

		if(base->gps.serverFd > 0)
		{
			FD_SET(base->gps.serverFd, &fds); /* 添加描述符 */
			if(base->gps.serverFd > maxfd) maxfd = base->gps.serverFd;
		}

		if(maxfd == 0)
		{
			seconds_sleep(10);
			continue;
		}
		
		timeout.tv_sec = 5;
	    switch(select(maxfd+1, &fds, NULL, NULL, &timeout))
	    {
	    case -1: break;
	    case  0: break;
	    default:
		  if(base->gps.serverFd > 0 && FD_ISSET(base->gps.serverFd, &fds))
		  {
		  	unsigned char data[1024] = {0};
			n = read(base->gps.serverFd, data, 1024);
			if(n>0)
			{
				satfi_log("base->gps.serverFd size = %d\n", n);
				//write(base->gps.usbSerialStm32, data, n);
				#if 1
				int i=0;
				while(n>0)
				{
					//satfi_log("base->gps.serverFd=%d\n", n);
					if(n>64)
					{
						write(base->gps.usbSerialStm32, &data[i], 64);
					}
					else
					{
						write(base->gps.usbSerialStm32, &data[i], n);
					}
					i+=64;
					n-=64;
					usleep(50000);
				}
				#endif
			}
			else
			{
				close(base->gps.serverFd);
				base->gps.serverFd = 0;
			}
		  }
		  
		  if(sock_tsc > 0 && FD_ISSET(sock_tsc, &fds))
		  {
			if(offset < sizeof(buf))
			{
				bzero(buf+offset,sizeof(buf)-offset);
			}
			bzero(tmp,sizeof(tmp));
			
	        nread = read(sock_tsc,buf+offset,4096) + offset;

			if(offset > 0 && nread <= offset)
			{
				satfi_log("sock_tsc error %d %s\n",errno, strerror(errno));
				close(sock_tsc);
				sock_tsc = -1;
				break;
			}

			offset  = 0;
			while(1)
			{
				if (nread > 0)
				{
				  pHeader = (Header *)(&buf[offset]);
				  satfi_log("read from TSC nread=%d,length=%d,mclass=0X%04X offset=%d\n",nread,pHeader->length,pHeader->mclass,offset);

				  if((offset + pHeader->length) > nread)
				  {
					int leftsize = nread - offset;
					memcpy(buf,pHeader,leftsize);
					offset = leftsize;
					satfi_log("leftsize=%d\n",leftsize);
					break;
				  }

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
						
						get_app_message_from_tsc();
						ReadCallRecordsAndSendToTSC(CALL_RECORDS_FILE);
						ReadGpsDataAndSendToTSC(GPS_DATA_FILE);
						updateMSList();
						break;
					}
					case SATFI_HEART_BEAT_CMD_RSP:
						{
						base->tsc.tsc_hb_rsp_ltime = time(0);
						satfi_log("from tsc heartbeat rsp\n");

						ReadCallRecordsAndSendToTSC(CALL_RECORDS_FILE);
						int size = DelGpsData(GPS_DATA_FILE);
						if(size > 0)
						{
							ReadGpsDataAndSendToTSC(GPS_DATA_FILE);
						}
						break;
					}
					case SATFI_HEART_BEAT_SP_CMD_RSP:
						{
						base->tsc.tsc_hb_rsp_ltime = time(0);
						satfi_log("from tsc heartbeatSP rsp\n");
						
						ReadCallRecordsAndSendToTSC(CALL_RECORDS_FILE);
						int size = DelGpsData(GPS_DATA_FILE);
						if(size > 0)
						{
							ReadGpsDataAndSendToTSC(GPS_DATA_FILE);
						}
						break;
					}
					case SATFI_MS_LIST_CMD_RSP:
						{
						satfi_log("from tsc SATFI_MS_LIST_CMD rsp\n");
						break;
					}
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
							req->Lg = base->gps.Lg;
							req->Lg_D = base->gps.Lg_D;
							req->Lt = base->gps.Lt;
							req->Lt_D = base->gps.Lt_D;

							satfi_log("Lg=%d Lg_D=%c Lt=%d Lt_D=%c\n",req->Lg, req->Lg_D, req->Lt, req->Lt_D);
						}
						
						break;
					}
		          	case APP_QUERY_GROUP_CMD_RSP:
						{
						Msg_Query_Group_Rsp *rsp = (Msg_Query_Group_Rsp *)pHeader;
						
						MsgQueryGroupRsp * req = (MsgQueryGroupRsp *)tmp;
						req->header.length = sizeof(MsgQueryGroupRsp) + rsp->Count * sizeof(MsgGroup);
						req->header.mclass = QUERY_GROUP_RSP;
						req->Count = rsp->Count;
						for(cnt=0;cnt<rsp->Count;cnt++)
						{
							BcdToStr(req->msggroup[cnt].GrpID, rsp->group[cnt].GrpID, 11);
							memcpy(req->msggroup[cnt].GrpName,rsp->group[cnt].GrpName, 30);
							req->msggroup[cnt].Config = rsp->group[cnt].Config;
							req->msggroup[cnt].Creator = rsp->group[cnt].Creator;
							req->msggroup[cnt].IsGrpChat = rsp->group[cnt].IsGrpChat;
						}

						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app QUERY_GROUP_RSP %s %d\n",userid, pUser->socketfd);
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}					
						
						break;
		          	}
		          	case APP_QUERY_MS_CMD_RSP:
						{						
						Msg_QueryMs_Rsp *rsp = (Msg_QueryMs_Rsp *)pHeader;
						
						MsgQueryMsRsp * req = (MsgQueryMsRsp *)tmp;
						req->header.length = sizeof(MsgQueryMsRsp) + rsp->Count * sizeof(MsgTerminal);
						req->header.mclass = QUERY_MS_RSP;					
						BcdToStr(req->GrpID,rsp->GrpID,11);
						req->Count = rsp->Count;
						for(cnt=0;cnt<rsp->Count;cnt++)
						{
							memcpy(req->msgterminal[cnt].MsID, FixMsID, 9);
							BcdToStr(&(req->msgterminal[cnt].MsID[9]), rsp->terminal[cnt].MsID, 6);
							memcpy(req->msgterminal[cnt].MsName,rsp->terminal[cnt].MsName, 30);
							req->msgterminal[cnt].Config = rsp->terminal[cnt].Config;
							req->msgterminal[cnt].Type = rsp->terminal[cnt].Type;
							req->msgterminal[cnt].OnlineStatus = rsp->terminal[cnt].OnlineStatus;
						}

						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app QUERY_MS_RSP %s %d\n",userid, pUser->socketfd);
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}	

						break;
		          	}
		          	case APP_SET_BLOCK_CMD_RSP:
						{
						Msg_SetBlock_Rsp *rsp = (Msg_SetBlock_Rsp *)pHeader;
						
						MsgSetBlockRsp * req = (MsgSetBlockRsp *)tmp;
						req->header.length = sizeof(MsgSetBlockRsp);
						req->header.mclass = SET_BLOCK_RSP;					
						BcdToStr(req->GrpID,rsp->GrpID,11);
						req->Result = rsp->Result;

						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app SET_BLOCK_RSP %s %d\n",userid, pUser->socketfd);
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						
						break;
		          	}
		          	case APP_UPLOAD_MESSAGE_CMD_RSP:
						{
						Msg_Upload_Message_Rsp *rsp = (Msg_Upload_Message_Rsp *)pHeader;

						MsgUploadMessageRsp * req = (MsgUploadMessageRsp *)tmp;
						req->header.length = sizeof(MsgUploadMessageRsp);
						req->header.mclass = UPLOAD_MESSAGE_RSP;					
						req->Result = rsp->Result;
						req->ID = rsp->ID;
						
						satfi_log("APP_UPLOAD_MESSAGE_CMD_RSP %d\n", req->ID);
	
						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{					
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								satfi_log("sendto app UPLOAD_MESSAGE_RSP %.21s %d %d %d %d\n",req->MsID, req->Result, req->ID, pUser->socketfd, req->header.length);
								log_insert(pUser->userid, tmp, req->header.length);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}

						Pack_Del(req->ID);
						break;
		          	}
		          	case APP_UPLOAD_VOICE_CMD_RSP:
						{
						
						Msg_Upload_Voice_Rsp *rsp = (Msg_Upload_Voice_Rsp *)pHeader;
						MsgUploadVoiceRsp * req = (MsgUploadVoiceRsp *)tmp;
						req->header.mclass = UPLOAD_VOICE_RSP;		
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
						}

						satfi_log("APP_UPLOAD_VOICE_CMD_RSP %d %d\n", req->Name, req->Result);
						
						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								satfi_log("sendto app UPLOAD_VOICE_RSP %.21s %d %d %d %d\n",req->MsID, req->Result, req->Name, pUser->socketfd, req->header.length);
								log_insert(pUser->userid, tmp, 31);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}

						Pack_Del(req->Name);
						break;
		          	}
		          	case APP_UPLOAD_PICTURE_CMD_RSP:
						{
						Msg_Upload_Picture_Rsp *rsp = (Msg_Upload_Picture_Rsp *)pHeader;
						
						MsgUploadPictureRsp * req = (MsgUploadPictureRsp *)tmp;
						req->header.mclass = UPLOAD_PICTURE_RSP;		
						req->Result = rsp->Result;
						req->Name = rsp->Name;
						if(rsp->Result == 1)
						{
							req->header.length = sizeof(MsgUploadPictureRsp) + rsp->packnum * sizeof(MsgPackSeq);
							req->packnum = rsp->packnum;
							satfi_log("Result=%d Name=%d packnum=%d",req->Result, req->Name, req->packnum);
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
						}
						
						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								satfi_log("sendto app UPLOAD_PICTURE_RSP %.21s %d %d %d %d\n",req->MsID, req->Result, req->Name, pUser->socketfd, req->header.length);
								log_insert(pUser->userid, tmp, 31);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}

						Pack_Del(req->Name);
						break;
		          	}
		          	case APP_READ_OK_CMD_RSP:
						{
						Msg_Read_OK_Rsp *rsp = (Msg_Read_OK_Rsp *)pHeader;
						
						MsgReadOKRsp * req = (MsgReadOKRsp *)tmp;
						req->header.length = sizeof(MsgReadOKRsp);
						req->header.mclass = READ_OK_RSP;	
						req->ID= rsp->ID;
						req->Result = rsp->Result;

						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app READ_OK_RSP %s %d\n",userid, pUser->socketfd);
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						
						break;
		          	}
		          	case APP_GET_MESSAGE_CMD_RSP:
						{
						int messagelen = 0;
						Msg_Get_Message_Rsp *rsp = (Msg_Get_Message_Rsp *)pHeader;

						MsgGetMessageRsp * req = (MsgGetMessageRsp *)tmp;
						req->header.mclass = GET_MESSAGE_RSP;
						req->Result = rsp->Result;
						if(rsp->Result == 1)
						{
							messagelen = rsp->header.length - ((int)rsp->message - (int)rsp);
							req->header.length = sizeof(MsgGetMessageRsp) + messagelen;
							memcpy(req->SrcMsID, FixMsID, 9);
							BcdToStr(&(req->SrcMsID[9]),rsp->SrcMsID,6);
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
						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app GET_MESSAGE_RSP %s %d\n",userid, pUser->socketfd);								
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						
						break;
		          	}

					case APP_ZF_MESSAGE_RSP:
						{
						Msg_Zf_Message_Rsp *rsp = (Msg_Zf_Message_Rsp *)pHeader;

						MsgZfMessageRsp * req = (MsgZfMessageRsp *)tmp;
						req->header.length = sizeof(MsgZfMessageRsp);
						req->header.mclass = ZF_MESSAGE_RSP;
						req->Result = rsp->Result;
						req->ID = rsp->ID;

						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app ZF_MESSAGE_RSP %s %d\n",userid, pUser->socketfd);
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						
						break;
					}
					
					case APP_EMERGENCY_ALARM_CMD_RSP:
						{
						Msg_Emergency_Alarm_Rsp *rsp = (Msg_Emergency_Alarm_Rsp *)pHeader;

						MsgEmergencyAlarmRsp * req = (MsgEmergencyAlarmRsp *)tmp;
						req->header.length = sizeof(MsgEmergencyAlarmRsp);
						req->header.mclass = EMERGENCY_ALARM_RSP;
						req->Result = rsp->Result;

						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app EMERGENCY_ALARM_RSP %s %d\n",userid, pUser->socketfd);
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						
						break;
					}
					
					case APP_CANCEL_EMERGENCY_ALARM_CMD_RSP:
						{
						Msg_Cancel_Emergency_Alarm_Rsp *rsp = (Msg_Cancel_Emergency_Alarm_Rsp *)pHeader;

						MsgCancelEmergencyAlarm_Rsp * req = (MsgCancelEmergencyAlarm_Rsp *)tmp;
						req->header.length = sizeof(MsgCancelEmergencyAlarm_Rsp);
						req->header.mclass = CANCEL_EMERGENCY_ALARM_RSP;
						req->Result = rsp->Result;

						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app CANCEL_EMERGENCY_ALARM_RSP %s %d\n",userid, pUser->socketfd);
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}

						break;
					}

					case APP_OPERAT_CMD_RSP:
						{
						Msg_App_Operat_Rsp *rsp = (Msg_App_Operat_Rsp *)pHeader;
						
						MsgAppOperat_Rsp * req = (MsgAppOperat_Rsp *)tmp;
						req->header.length = sizeof(MsgCancelEmergencyAlarm_Rsp);
						req->header.mclass = OPERAT_RSP;
						req->Result = rsp->Result;

						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app CANCEL_EMERGENCY_ALARM_RSP %s %d\n",userid, pUser->socketfd);
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}

						break;
					}
					
		          	case TSC_NOTIFY_RSP:
						{
						Msg_Notify_Rsp *rsp = (Msg_Notify_Rsp *)pHeader;
						
						MsgNotifyRsp * req = (MsgNotifyRsp *)tmp;
						req->header.length = sizeof(MsgNotifyRsp);
						req->header.mclass = NOTIFY_CMD;
						req->ID = rsp->ID;
						req->Type = rsp->Type;
						
						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						satfi_log("TSC_NOTIFY_RSP %s ID=%x Type=%d\n",userid, req->ID, req->Type);
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app NOTIFY_CMD %s %d\n",userid, pUser->socketfd);
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						
						break;
		          	}
		          	case TSC_NO_ENOUGHMONEY_RSP:
					{
						Msg_NoEnough_Money_Rsp *rsp = (Msg_NoEnough_Money_Rsp *)pHeader;

						MsgNoEnoughMoneyRsp * req = (MsgNoEnoughMoneyRsp *)tmp;
						req->header.length = sizeof(MsgNoEnoughMoneyRsp);
						req->header.mclass = NOTENOUGH_MONEY_CMD;
						req->Money = rsp->Money;
						
						USER *pUser = gp_users;
						BcdToStr(userid,rsp->MsID,6);
						satfi_log("NOTENOUGH_MONEY_CMD %s\n",userid);

						if(PackHead != NULL)
						{
							Pack_Del(PackHead->Name);
						}
						
						while(pUser)
						{
							if(strncmp(&pUser->userid[USERID_LLEN - USERID_LEN], userid, USERID_LEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app NOTENOUGH_MONEY_CMD %s %d\n",userid, pUser->socketfd);
								strncpy(req->MsID, pUser->userid, USERID_LLEN);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						
						break;
		          	}
		          	case TSC_NOTIFY_EMERGENCY_ALARM_CMD_RSP:
					{
						Msg_Notify_Emergency_Alarm_Rsp *rsp = (Msg_Notify_Emergency_Alarm_Rsp *)pHeader;
						
						MsgNotifyEmergencyAlarm_Rsp * req = (MsgNotifyEmergencyAlarm_Rsp *)tmp;
						req->header.length = sizeof(MsgNotifyEmergencyAlarm_Rsp);
						req->header.mclass = NOTIFY_EMERGENCY_ALARM_CMD;
						memcpy(req->MsName, rsp->MsName, 30);
						req->Lg = rsp->Lg;
						req->Lt = rsp->Lt;
						req->AlarmType = rsp->AlarmType;

						USER *pUser = gp_users;						
						memcpy(userid, FixMsID, 9);
						BcdToStr(&(userid[9]), rsp->MsID, 6);
						
						while(pUser)
						{						
							if(strncmp(pUser->userid, userid, USERID_LLEN) != 0 && pUser->socketfd > 0)
							{
								strncpy(req->MsID, userid, USERID_LLEN);
								satfi_log("sendto app NOTIFY_EMERGENCY_ALARM_CMD %s\n", req->MsID);							
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								//break;
							}
							pUser = pUser->next;
						}

						break;
		          	}

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

						USER *pUser = gp_users;
						while(pUser)
						{		
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app NOTIFY_CANCEL_EMERGENCY_CMD %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], req->Type);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						break;
					}

					case APP_PHONE_MONEY_RSP:
					{
						Msg_Phone_Money_Rsp *rsp = (Msg_Phone_Money_Rsp *)pHeader;

						MsgPhoneMoneyRsp * req = (MsgPhoneMoneyRsp *)tmp;
						req->header.length = sizeof(MsgPhoneMoneyRsp);
						req->header.mclass = PHONE_MONEY_RSP;
						memcpy(req->MsID, FixMsID, 9);
						BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
						req->Money = rsp->Money;
						satfi_log("APP_PHONE_MONEY_RSP %.21s req->Money=%d\n", req->MsID, req->Money);
						ReadCallRecordsAndSendToTSC(CALL_RECORDS_FILE);
						break;
						
						USER *pUser = gp_users;
						while(pUser)
						{			
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app PHONE_MONEY_RSP %s\n",&pUser->userid[USERID_LLEN - USERID_LEN]);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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
						req->ID = rsp->ID;
						
						USER *pUser = gp_users;
						while(pUser)
						{							
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app GRP_UPLOAD_MESSAGE_RSP %s\n",&pUser->userid[USERID_LLEN - USERID_LEN]);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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
						}
						else if(rsp->Result == 1)
						{
							req->header.length = sizeof(MsgGrpUploadVoiceRsp) + rsp->packnum * sizeof(MsgPackSeq);
							req->packnum = rsp->packnum;
							for(cnt=0;cnt<rsp->packnum;cnt++)
							{
								req->msgpackseq[cnt].pack_seq_1 = rsp->packseq[cnt].pack_seq_1;
								req->msgpackseq[cnt].pack_seq_2 = rsp->packseq[cnt].pack_seq_2;
							}		
						}
						
						USER *pUser = gp_users;
						while(pUser)
						{							
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app GRP_UPLOAD_VOICE_RSP %s\n",&pUser->userid[USERID_LLEN - USERID_LEN]);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						
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
						}
						else if(rsp->Result == 1)
						{
							req->header.length = sizeof(MsgGrpUploadVoiceRsp) + rsp->packnum * sizeof(MsgPackSeq);
							req->packnum = rsp->packnum;
							for(cnt=0;cnt<rsp->packnum;cnt++)
							{
								req->msgpackseq[cnt].pack_seq_1 = rsp->packseq[cnt].pack_seq_1;
								req->msgpackseq[cnt].pack_seq_2 = rsp->packseq[cnt].pack_seq_2;
							}		
						}

						USER *pUser = gp_users;
						while(pUser)
						{							
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app GRP_UPLOAD_PICTURE_RSP %s\n",&pUser->userid[USERID_LLEN - USERID_LEN]);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						
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
						
						USER *pUser = gp_users;
						while(pUser)
						{							
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app GRP_READ_OK_RSP %s\n",&pUser->userid[USERID_LLEN - USERID_LEN]);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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

						USER *pUser = gp_users;
						while(pUser)
						{							
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app SET_GRP_CHAT_RSP %s\n",&pUser->userid[USERID_LLEN - USERID_LEN]);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						
					}
					break;

					case APP_GRP_NOTIFY_CMD:
					{
						Msg_Grp_Notify_Rsp *rsp = (Msg_Grp_Notify_Rsp *)pHeader;
						int datalen = rsp->header.length - ((int)rsp->data - (int)rsp);
						//printfhex(rsp, rsp->header.length);
						MsgGrpNotifyRsp * req = (MsgGrpNotifyRsp *)tmp;
						req->header.length = sizeof(MsgGrpNotifyRsp) + datalen;
						req->header.mclass = GRP_NOTIFY_CMD;
						memcpy(req->MsID, FixMsID, 9);
						BcdToStr(&(req->MsID[9]), rsp->MsID, 6);
						BcdToStr(req->TargetGrpID, rsp->TargetGrpID, 11);
						req->Name = rsp->Name;
						req->Type = rsp->Type;
						if(datalen>0)memcpy(req->data, rsp->data, datalen);

						//printfhex(req, req->header.length);
						USER *pUser = gp_users;
						while(pUser)
						{
							if(pUser->socketfd > 0)
							{
								satfi_log("sendto app GRP_NOTIFY_CMD %.21s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], rsp->Name);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
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

						USER *pUser = gp_users;
						while(pUser)
						{							
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								if(pUser->socketfd < 0)break;
								satfi_log("sendto app SET_MSG_RSP %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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

						base->tsc.tsc_hb_rsp_ltime = time(0);
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
						req->Lg_D 	= rsp->Lg_D;
						req->Lt 	= rsp->Lt;
						req->Lt_D 	= rsp->Lt_D;

						satfi_log("APP_SHARE_GPS %.21s %.21s\n",req->MsID, req->SrcMsID);
						
						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_SHARE_GPS %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_WL_COMMAND %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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
						
						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_SET_ONLINE_RSP %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app TSC_NOTIFY_MS %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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
						
						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_GPS_POINT %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
						
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

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_PTT_RSP %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}

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
						
						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_PTT_IN %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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
												
						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_HUPIN %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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
						//printfhex(tmp,req->header.length );
						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_HUP_RSP %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}

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
						//satfi_log("APP_MT_VOICE %.21s %d\n", req->MsID, datalen);

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_MT_VOICE %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], datalen);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}

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

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_SPTT_NOTIFY %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}

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

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_MS_HUP_RSP %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}

					}
					break;

					case FLAG_NOTIFY://no use
					{
						Msg_Flag_Notify *rsp = (Msg_Flag_Notify*)pHeader;
						satfi_log("FLAG_NOTIFY %s %d %d", rsp->Sat_IMSI, rsp->Flag, rsp->Id);

						bSatNetWorkActive = rsp->Flag;

						Msg_Flag_Notify_Rsp *req = (Msg_Flag_Notify_Rsp *)tmp;
						req->header.length = sizeof(Msg_Flag_Notify_Rsp);
						req->header.mclass = FLAG_NOTIFY_RSP;
						req->Id = rsp->Id;

						pthread_mutex_lock(&tsc_mutex);
						n = write(sock_tsc, tmp, req->header.length);
						pthread_mutex_unlock(&tsc_mutex);
						if(n < 0)
						{
							satfi_log("write to sock_tsc error");
						}
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

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_CLR_SPTT_RSP %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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
						
						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->TARGET_MS_Id, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_ASPTT_REQ %s 0X%04X %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], req->header.mclass, req->header.length);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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
						
						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_ASPTT_REQ_RSP %s 0X%04X %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], req->header.mclass, req->header.length);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_ASPTT_HUP %s 0X%04X %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], req->header.mclass, req->header.length);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_ASPTT_HUP_RSP %s 0X%04X %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], req->header.mclass, req->header.length);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->TARGET_MS_Id, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_ASPTT_CLER %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->MsID, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_ASPTT_CLER_RSP %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
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

						USER *pUser = gp_users;
						while(pUser)
						{
							if(strncmp(pUser->userid, req->TARGET_MS_Id, USERID_LLEN) == 0)
							{
								satfi_log("sendto app APP_ASPTT_CANCEL %s %d\n",&pUser->userid[USERID_LLEN - USERID_LEN], pUser->socketfd);
								if(pUser->socketfd < 0)break;
								pthread_mutex_lock(&(pUser->msg_mutex));
								n = write(pUser->socketfd, tmp, req->header.length);
								pthread_mutex_unlock(&(pUser->msg_mutex));
								if(n != req->header.length)
								{
									satfi_log("DATA ERROR n=%d p->length=%d %d\n",n,req->header.length,__LINE__);
									App_Remove(pUser->socketfd);
									close(pUser->socketfd);
									pUser->socketfd = -1;
									updateMSList();
								}
								if(n<0) satfi_log("sendto return error: errno=%d (%s) %d\n", errno, strerror(errno),__LINE__);
								break;
							}
							pUser = pUser->next;
						}
					}
					break;
					
					default:
						{
						satfi_log("received unrecognized message from tsc: %04x\n", pHeader->mclass);
						offset = 0;
						nread = 0;
						pHeader->length = 0;
						break;	
					}
				  }

		        }
				else
				{
					satfi_log("read return error: errno=%d (%s) nread=%d %d\n", errno, strerror(errno), nread, time(0));
					close(sock_tsc);
					sock_tsc = -1;
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
	      break;
	    }
	  }
}

int AudioPlayInit(int rate, int bits, int channels)
{
	int ret;
    int dspfd = open("/dev/dsp", O_WRONLY);
    if (dspfd <= 0) 
	{
		satfi_log("open of /dev/dsp failed %d %s\n", errno, strerror(errno));
		return -1;
    }

    /* set rate */
    ret = ioctl(dspfd, SOUND_PCM_WRITE_RATE, &rate);
    if (ret == -1)
    {
		satfi_log("SOUND_PCM_WRITE_WRITE ioctl failed");
		return -1;
	}

    ret = ioctl(dspfd, SOUND_PCM_WRITE_BITS, &bits);
    if (ret == -1)
    {
		satfi_log("SOUND_PCM_WRITE_BITS ioctl failed");
		return -1;
	}

    ret = ioctl(dspfd, SOUND_PCM_WRITE_CHANNELS, &channels);
    if (ret == -1)
    {
        satfi_log("SOUND_PCM_WRITE_CHANNELS ioctl failed");
		return -1;
	}

	return dspfd;
}

int AudioRecordInit(int rate, int bits, int channels)
{
	int ret;
    int dspfd = open("/dev/dsp", O_RDONLY);
    if (dspfd < 0) 
	{
		satfi_log("open of /dev/dsp failed %d %s\n", errno, strerror(errno));
		return -1;
    }

    /* set rate */
    ret = ioctl(dspfd, SOUND_PCM_WRITE_RATE, &rate);
    if (ret == -1)
    {
		satfi_log("SOUND_PCM_WRITE_WRITE ioctl failed\n");
		return -1;
	}       

    ret = ioctl(dspfd, SOUND_PCM_WRITE_BITS, &bits);
    if (ret == -1)
    {
		satfi_log("SOUND_PCM_WRITE_BITS ioctl failed\n");
		return -1;
	}

    ret = ioctl(dspfd, SOUND_PCM_WRITE_CHANNELS, &channels);
    if (ret == -1)
    {
        satfi_log("SOUND_PCM_WRITE_CHANNELS ioctl failed\n");
		return -1;
	}

	return dspfd;
}


int AudioPlay(int dspFd, char *data, int size)
{
	return write(dspFd, data, size);
}

void AudioPlayExit(int dspFd)
{
	close(dspFd);
}

static void *send_app_voice_data(void *p)
{
	BASE *base = (BASE *)p;
	int AudioRecordFd = -1;
	int size = 320;
	int ret;
	int rate = 8000;
	int bits = 16;
	int channels = 1;
	
	char VoiceData[1024];
	
	while(1)
	{
		//satfi_log("%d %d %d %d\n", AudioRecordFd, app_socket_voice, base->sat.sat_calling, base->sat.sat_state_phone);
		if(base->sat.sat_calling == 1 && base->sat.sat_state_phone == SAT_STATE_PHONE_ONLINE && 
			app_socket_voice > 0)
		{
			if(AudioRecordFd<0)
			{
				AudioRecordFd = AudioRecordInit(rate,bits,channels);
				if(AudioRecordFd>0)
				{
					satfi_log("AudioRecord Init app_socket_voice=%d\n", app_socket_voice);
				} 
				else
				{
					seconds_sleep(1);
				}
			}

			if(AudioRecordFd>0)
			{
				//satfi_log("read Audio %d %d %d\n", size, base->sat.sat_calling, app_socket_voice);
				ret = read(AudioRecordFd, VoiceData, size);
				if(ret != size)
				{
					satfi_log("AudioRecord Error %d %d\n",ret,size);
				}
				//satfi_log("write app %d  %d %d\n", ret, base->sat.sat_calling, app_socket_voice);
				if(app_socket_voice > 0)
				{
					ret = write(app_socket_voice, VoiceData, ret);
					if(ret < 0)
					{
						satfi_log("write app_socket_voice error: errno=%d (%s) %d %d\n", errno, strerror(errno), __LINE__, app_socket_voice);
					}
				}
			}
		}
		else
		{
			if(AudioRecordFd>0)
			{
				satfi_log("AudioRecord Exit app_socket_voice=%d\n", app_socket_voice);	
				close(AudioRecordFd);
				AudioRecordFd = -1;
				close(app_socket_voice);
				app_socket_voice = -1;
			}

			seconds_sleep(1);
		}
	}
}

static void *recv_app_voice_data(void *p)
{
	struct sockaddr_in client_addr;
	socklen_t cliaddr_len = sizeof(client_addr);
	int size=320;
	int ret;
	int rate = 8000;
	int bits = 16;
	int channels = 1;
	char VoiceData[1024];//8K 16bit 单通道

	while(1)
	{
		if(base.sat.sat_calling == 0 || base.sat.sat_state_phone != SAT_STATE_PHONE_ONLINE)
		{
			seconds_sleep(1);
			continue;
		}
		
		int AudioPlayFd = AudioPlayInit(rate,bits,channels);
		if(AudioPlayFd < 0)
		{
			satfi_log("AudioPlayInit Error\n");	
			seconds_sleep(1);
			continue;
		}
		
		int AcceptFd = appsocket_init(12060);
		//satfi_log("AcceptFd AcceptFd=%d\n", AcceptFd);
		app_socket_voice = accept(AcceptFd, (struct sockaddr*)&client_addr, &cliaddr_len);   							
		if(app_socket_voice < 0)
		{
			satfi_log("AcceptFd error\n");
		}
		else
		{
			close(AcceptFd);
			satfi_log("AudioPlay Init app_socket_voice=%d\n", app_socket_voice);	
			while((ret = read(app_socket_voice, VoiceData, size)) > 0)
			{
				//satfi_log("read app %d\n", ret);
				write(AudioPlayFd, VoiceData, ret);
				if(ret != size)
				{
					//satfi_log("read app_socket_voice error %d %d\n",ret,size);	
				}
				if(base.sat.sat_calling == 0)break;
			}

			satfi_log("AudioPlay Exit app_socket_voice=%d\n", app_socket_voice);	
			close(app_socket_voice);
			app_socket_voice = -1;
			AudioPlayExit(AudioPlayFd);
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
	
	bzero(cmd, sizeof(cmd));
	sprintf(cmd,"wget -c -P /tmp/ %s", config_url);
	myexec(cmd, NULL, NULL);

	if(!isFileExists("/tmp/update.ini"))
	{
		satfi_log("wget /tmp/update.ini not exit\n");
		bzero(cmd, sizeof(cmd));
		sprintf(cmd,"httpdown %s /tmp/update.ini %s",base.n3g.n3g_ifname, config_url);
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
			bzero(cmd, sizeof(cmd));
			sprintf(cmd,"wget -c -P /tmp/ %s", satfiurl);
			myexec(cmd, NULL, NULL); 

			if(!isFileExists("/tmp/satfi_1_ramips_24kec.ipk"))
			{
				satfi_log("wget /tmp/satfi_1_ramips_24kec.ipk not exit\n");
				bzero(cmd, sizeof(cmd));
				sprintf(cmd,"httpdown %s /tmp/satfi_1_ramips_24kec.ipk %s",base.n3g.n3g_ifname, satfiurl);
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
				close(sock_tsc);
				sock_tsc = -1;
				bTscConnected = 0;
				return;
			}
		}

		if(base->tsc.tsc_timeout > 0)
		{
			if(loop2 >= base->tsc.tsc_timeout || loop2 == 0)
			{
				sendHeartBeat();				
				if(bGetGpsData)
				{
					//SaveGpsToFile(GPS_DATA_FILE);
					//ReadGpsDataAndSendToTSC(GPS_DATA_FILE);
					//base->tsc.tsc_hb_req_ltime = time(0);
				}
				else
				{
					//sendHeartBeat();
				}

				iCntUserSave = 0;
				updateMSList();
							
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
				if(bGetGpsData)SaveGpsToFile(GPS_DATA_FILE);	
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


void *SystemServer(void *p)
{
	BASE *base = (BASE *)p;

	time_t now;
	int sm2700ledstaton = 0;//0表示灭
	int gpsledstaton = 0;	//0表示灭
	int stat = 0;

	int TimeOutCnt = 0;
	int HeartBeatTimeOutCnt = 0;
	int CheckProgramUpdateCnt = 0;
	int NeedCheckProgramUpdate = 1;
	int NotPpsDataCnt = 0;

	//insmod led_driver
	if(!isFileExists("/dev/leds_driver"))
	{
		myexec("insmod lib/modules/leds_driver.ko", NULL, NULL);	
		seconds_sleep(1);
	}

	led_control(SM2700_LED_ON);
	led_control(GPS96_LED_ON);

	while(1)
	{
		now = time(0);
		if(base->tsc.tsc_hb_req_ltime == 0)
		{
			stat &= (~(1<<0));
		}
		else
		{
			if(base->tsc.tsc_hb_rsp_ltime < base->tsc.tsc_hb_req_ltime)
			{
				//satfi_log("TimeOutCnt = %d %d %d\n", TimeOutCnt, base->tsc.tsc_hb_rsp_ltime, base->tsc.tsc_hb_req_ltime);
				stat &= (~(1<<0));
				TimeOutCnt++;
			}
			else
			{
				stat |= (1<<0);
				TimeOutCnt = 0;
				HeartBeatTimeOutCnt = 0;
			}

			if(TimeOutCnt >= 30)
			{
				HeartBeatTimeOutCnt++;
				satfi_log("HeartBeatSPTimeOutCnt = %d\n", HeartBeatTimeOutCnt);
				sendHeartbeatSP();
				TimeOutCnt = 0;
				if(HeartBeatTimeOutCnt >= 3)
				{
					HeartBeatTimeOutCnt = 0;
					stat &= (~(1<<0));
					close(sock_tsc);
					sock_tsc = -1;
					bTscConnected = 0;

					//if(bGetGpsData)SaveGpsToFile(GPS_DATA_FILE);	
				}
			}
		}

		if(bGetGpsData == 0)
		{
			stat &= (~(1<<1));//no gps
			NotPpsDataCnt++;
			if(NotPpsDataCnt == 60)
			{
				NotPpsDataCnt = 0;
				//satfi_log("BdGpsPowerControl not gps data\n");
				//BdGpsPowerControl(1);
			}
		}
		else
		{
			stat |= (1<<1);
			NotPpsDataCnt = 0;
		}

		if(stat == 0)
		{
			led_control(SM2700_LED_ON);
			led_control(GPS96_LED_ON);
			milliseconds_sleep(250);
			led_control(SM2700_LED_OFF);
			led_control(GPS96_LED_OFF);
			milliseconds_sleep(250);	
			led_control(SM2700_LED_ON);
			led_control(GPS96_LED_ON);
			milliseconds_sleep(250);
			led_control(SM2700_LED_OFF);
			led_control(GPS96_LED_OFF);
			milliseconds_sleep(250);

			gpsledstaton = 0;
			sm2700ledstaton = 0;
		}
		else if(stat == 1)
		{
			if(sm2700ledstaton == 0)
			{
				led_control(SM2700_LED_ON);
				sm2700ledstaton = 1;
			}

			led_control(GPS96_LED_ON);
			milliseconds_sleep(250);
			led_control(GPS96_LED_OFF);
			milliseconds_sleep(250);	
			led_control(GPS96_LED_ON);
			milliseconds_sleep(250);
			led_control(GPS96_LED_OFF);
			milliseconds_sleep(250);

			gpsledstaton = 0;
	
		}
		else if(stat == 2)
		{
			if(gpsledstaton == 0)
			{
				led_control(GPS96_LED_ON);
				gpsledstaton = 1;
			}

			led_control(SM2700_LED_ON);
			milliseconds_sleep(250);
			led_control(SM2700_LED_OFF);
			milliseconds_sleep(250);	
			led_control(SM2700_LED_ON);
			milliseconds_sleep(250);
			led_control(SM2700_LED_OFF);
			milliseconds_sleep(250);
			sm2700ledstaton = 0;
		}
		else if(stat == 3)
		{
			if(gpsledstaton == 0)
			{
				led_control(GPS96_LED_ON);
				gpsledstaton = 1;
			}

			if(sm2700ledstaton == 0)
			{
				led_control(SM2700_LED_ON);
				sm2700ledstaton = 1;
			}

			seconds_sleep(1);
		}

		if(base->n3g.n3g_status == 1)
		{
			if(NeedCheckProgramUpdate && CheckProgramUpdate() == 0)
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
		
		HeartBeat(base);

		if(sock_tsc > 0 && base->tsc.tsc_hb_rsp_ltime > 0)
		{
			SendPackToTSC();
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
	base.sat.sat_status = 0;
	base.sat.sat_state = -1;
	base.sat.sat_calling = 0;
	base.sat.sat_dialing = 0;
	base.sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
	base.sat.captain_socket = -1;;
	base.sat.start_time = 0;
	base.sat.end_time = 0;
	base.sat.sat_mode = 0;
	base.sat.forbid_dial = 0;
	base.sat.sat_wait_gps = 0;
	
	base.n3g.n3g_fd = -1;
	base.n3g.n3g_status = 0;
	base.n3g.n3g_state = -1;
	base.n3g.n3g_dialing = 0;
	base.n3g.forbid_dial = 0;

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


	GetIniKeyString("config","IP","/etc/rc.local",ucTmp);
	if(strlen(ucTmp)!=0)
	{
		strcpy(base.omc.omc_addr, ucTmp);
		satfi_log("IP:%s\n", ucTmp);
	}

	if(GetIniKeyInt("config","PORT","/etc/rc.local") > 0)
	{
		base.omc.omc_port = GetIniKeyInt("config","PORT","/etc/rc.local");
		satfi_log("PORT:%d\n", base.omc.omc_port);
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
		int n = write(socket, &rsp, rsp.header.length);
	    if(n<0) satfi_log("write return error: errno=%d (%s) %d %d %d\n", errno, strerror(errno),socket,__LINE__,sat_state_phone);
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
	satfi_log("CallUpThread net_lock\n");
	net_lock();
	satfi_log("CallUpThread net_lock pass\n");
	BASE *base = (BASE*)p;
	char buf[256] = {0};
	
	base->sat.sat_status = 0;
	base->sat.sat_available = 0;

	char ucbuf[256];
	sprintf(ucbuf, "ifdown %s", base->sat.sat_ifname_a);
	myexec(ucbuf, NULL, NULL);
	seconds_sleep(1);

	if(base->sat.sat_state == SAT_STATE_GPSTRACK_START_W)
	{
		base->sat.sat_state = SAT_STATE_GPSTRACK_STOP_W;
		uart_send(base->sat.sat_fd, "AT+GPSTRACK=0,0,16\r\n", 20);
		while(1)
		{
			satfi_log("base->sat.sat_state = %d\n", base->sat.sat_state);
			if(base->sat.sat_state != SAT_STATE_GPSTRACK_STOP_W)
			{
				base->sat.sat_state = SAT_STATE_GPSTRACK_START;
				break;
			}
			seconds_sleep(1);
		}
	}

    //satfi_log("power_mode sat_sm2500 reset %d\n",__LINE__);
    //myexec("power_mode sat_sm2500 reset",NULL,NULL);
	//seconds_sleep(20);
	close(base->sat.sat_fd);
	base->sat.sat_fd = -1;	

	base->sat.sat_state_phone = SAT_STATE_PHONE_CLCC;
	int atdwaitcnt=0,clcccnt=0,ringcnt=0,dialcnt=0,dialfailecnt=0;

	//base->sat.StartTime = time(0);
	//satfi_log("StartTime=%lld\n", base->sat.StartTime);

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
		
		satfi_log("base->sat.sat_state_phone = %d\n", base->sat.sat_state_phone);
		switch(base->sat.sat_state_phone)
		{
			case SAT_STATE_PHONE_CLCC:
		        //uart_send(base->sat.sat_fd, "ATH\r\n", 5);
		        uart_send(base->sat.sat_fd, "AT+CLVL=5\r\n", 11);
				uart_send(base->sat.sat_fd, "AT+GPSTRACK=0,0,16\r\n", 20);
				satfi_log("AT+CLVL=5,clcccnt=%d\n",clcccnt);
		        //uart_send(base->sat.sat_fd, "AT+CLCC\r\n", 9);
		        break;
			case SAT_STATE_PHONE_CLCC_OK:
				sprintf(buf,"ATD%s;\r\n",base->sat.calling_number);
				//satfi_log("ATD = %s\n", buf);
				uart_send(base->sat.sat_fd, buf, 6+strlen(base->sat.calling_number));
				base->sat.sat_state_phone = SAT_STATE_PHONE_ATD_W;
				break;
			case SAT_STATE_PHONE_ATD_W:
				AppCallUpRsp(base->sat.socket, get_sat_dailstatus());
				atdwaitcnt++;
				if(atdwaitcnt == 10)
				{
					base->sat.sat_state_phone = SAT_STATE_PHONE_DIALING_ATH_W;
					uart_send(base->sat.sat_fd, "ATH\r\n", 5);
					atdwaitcnt = 0;
				}
				break;
			case SAT_STATE_PHONE_DIALING_CLCC:
				uart_send(base->sat.sat_fd, "AT+CLCC\r\n", 9);
			break;
			case SAT_STATE_PHONE_DIALING_ATH_W:
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
				AppCallUpRsp(base->sat.socket, get_sat_dailstatus());
				if(base->sat.StartTime == 0)
				{
					base->sat.StartTime = time(0);
					satfi_log("StartTime=%lld\n", base->sat.StartTime);
				}

				//close(base->sat.sat_fd);
				//base->sat.sat_fd = -1;
				break;
		}

		if(base->sat.sat_state_phone == SAT_STATE_PHONE_CLCC)
		{
			++clcccnt;
			if(clcccnt >= 10)
			{
				close(base->sat.sat_fd);
				base->sat.sat_fd = -1;	
			    satfi_log("power_mode sat_sm2500 reset %d\n",__LINE__);
			    myexec("power_mode sat_sm2500 reset",NULL,NULL);
				seconds_sleep(20);
				clcccnt = 0;
				base->sat.sat_state_phone = SAT_STATE_PHONE_DIALINGFAILE;
			}

		}
		
		if(base->sat.sat_state_phone == SAT_STATE_PHONE_DIALING_RING)
		{
			//satfi_log("ringcnt %d",ringcnt);
			++ringcnt;
			if(ringcnt >= 40)
			{
				base->sat.sat_state_phone = SAT_STATE_PHONE_NOANSWER;
				uart_send(base->sat.sat_fd, "ATH\r\n", 5);
				//ringcnt = 0;
			}
		}
		
		if(base->sat.sat_state_phone == SAT_STATE_PHONE_DIALING)
		{
			++dialcnt;
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

	//base->sat.EndTime = time(0);
	//satfi_log("EndTime=%lld\n", base->sat.EndTime);

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

		//CallRecordsADD((char *)rsp, rsp->header.length);
		SaveDataToFile(CALL_RECORDS_FILE ,(char *)rsp, rsp->header.length);

	}
	
	close(app_socket_voice);
	app_socket_voice = -1;
	base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
	base->sat.socket = 0;
	base->sat.sat_calling = 0;
	close(base->sat.sat_fd);
	base->sat.sat_fd = -1;
	satfi_log("CallUpThread Exit\n");
	net_unlock();
}

void StartCallUp(char calling_number[15])
{
	pthread_t thread;
	strncpy(base.sat.calling_number, calling_number,sizeof(base.sat.called_number));
	pthread_create(&thread,NULL,CallUpThread,&base);
}

static void *HangingUpThread(void *p)
{
	satfi_log("HangingUpThread Create\n");
	//printf("HangingUpThread Create\n");
	BASE *base = (BASE*)p;
	int cnt = 10;
	while(cnt--)
	{
		//printf("HangingUpThread sat_state_phone %d\n",base->sat.sat_state_phone);
		if(base->sat.sat_fd < 0)break;
		base->sat.sat_state_phone = SAT_STATE_PHONE_ATH_W;
		//satfi_log("ATH base->sat.sat_fd=%d\n", base->sat.sat_fd);
		uart_send(base->sat.sat_fd, "ATH\r\n", 5);
		seconds_sleep(1);
		//printf("ATH %d\n",base->sat.sat_state_phone);
		if(base->sat.sat_state_phone == SAT_STATE_PHONE_HANGUP ||
			base->sat.sat_state_phone == SAT_STATE_PHONE_COMING_HANGUP || 
			base->sat.sat_state_phone == SAT_STATE_PHONE_IDLE)
		{
			//printf("HangingUpThread quit sat_state_phone %d\n", base->sat.sat_state_phone);
			if(base->sat.sat_state_phone != SAT_STATE_PHONE_IDLE)
			{
				AppCallUpRsp(base->sat.socket, get_sat_dailstatus());
			}
			base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
			//break;
		}
	}
	//base->sat.sat_state_phone = SAT_STATE_PHONE_ATH_W;
	close(app_socket_voice);
	app_socket_voice = -1;
	base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
	base->sat.socket = 0;
	base->sat.sat_calling = 0;
	close(base->sat.sat_fd);
	base->sat.sat_fd = -1;
	satfi_log("HangingUpThread Exit\n");
	//printf("HangingUpThread Exit\n");

}

void HangingUp(void)
{
	pthread_t thread;
	pthread_create(&thread,NULL,HangingUpThread,&base);
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
	int optlen = sizeof(int);
	getsockopt(socket, SOL_SOCKET, SO_SNDBUF,&size1,&optlen);
	getsockopt(socket, SOL_SOCKET, SO_RCVBUF,&size2,&optlen);
	satfi_log("%s=%d SO_SNDBUF=%d SO_RCVBUF=%d\n",prefix,socket,size1,size2);
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

	//setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, (void *)&size_send, optlen);
	//setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY,&on,sizeof(on));        
	if(timeout > 0) setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeo, sizeof(timeo));

	if(routename)
	{
		strncpy(struIR.ifr_name, routename, IFNAMSIZ);
		if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, &struIR, sizeof(struIR)) < 0)
		{
			satfi_log("setsockopt Error:%s\n", strerror(errno));  
			close(sockfd);
			return -1; 
		}
	}

    server_addr.sin_family = AF_INET;   
    server_addr.sin_port = htons(port);   

    if(connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) 
	{
		if(err != NULL) *err = errno;
        satfi_log("Connect Error %s %d:%s errno=%d\n", ip, port, strerror(errno), errno);   
		close(sockfd);
        return -1;
    }
	else
	{
		satfi_log("ConnectTSC success sockfd=%d, ip=%s\n", sockfd, ip);
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

void handle_gps_data1(int gpsfd)
{
	static int idx = 0;
	char buf[1024] = {0};
	static char gpsbuf[1024];

	int i=0,k=0,kk=0;

	int n = read(gpsfd, buf, 1024);
	if(n>0)
	{
		if(idx+n >= 1024)
		{
			satfi_log("GPS data is too long to read idx=%d n=%d\n",idx,n);
			idx = 0;
		}
		else
		{
			memcpy(&gpsbuf[idx], buf, n);
			idx += n;
		}
			
		for(i=0;i<idx;i++)
		{
			if(gpsbuf[i] == '$')
			{
				k=i;
				for(i=k;i<idx;i++)
				{
					if(gpsbuf[i-1]==0xA && gpsbuf[i] == 0xA)//换行符
					{
						kk = i;
						//printf("%d %d %d\n",k, kk, idx);
						if(k>=0 && kk>0 && kk>k && kk-k<256 && (kk+1)<=idx)
						{
							char data[256] = {0};
							memcpy(data, &gpsbuf[k], kk-k-1);
							idx -= (kk+1);
							memcpy(buf, &gpsbuf[kk+2], idx);
							memcpy(gpsbuf, buf, idx);
							
							//printf("%s\n",data);
							if((strncmp(data,"$GPRMC",6) == 0) || (strncmp(data,"$GNRMC",6) == 0))
							{
								strncpy(GpsData, data, 256);
								strncpy(base.gps.gps_bd, data, 256);
								parseGpsData(data, kk-k);	
							}
						}
						else
						{
							satfi_log("data error %d %d %d",k,kk,idx);
						}
					}
				}
			}
		}
	}
}

void handle_gps_data(int gpsfd)
{
	static int idx = 0;
	char buf[1024];
	static char gpsbuf[1024];

	int n = read(gpsfd, buf, 1024);
	if(n>0)
	{
		if(idx+n<1024)
		{
			memcpy(&gpsbuf[idx], buf, n);
			idx += n;

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
							strncpy(GpsData, &gpsbuf[k], kk-k);
							GpsData[kk-k] = 0;
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
}

static void *ring_detect(void *p)
{
	BASE *base = (BASE *)p;
	seconds_sleep(1);

	int ring = 0;
	int ringnocarriercnt = 0;
	int ringcnt = 0;
	int get_gpio22_cnt = 0;

	int ringsocket = -1;
	
	while(1)
	{
		if(get_gpio22_value() == 0)
		{
			satfi_log("ring %d\n", get_gpio22_cnt);
			//printf("ring\n");
			close(base->sat.sat_fd);
			base->sat.sat_fd = -1;
			get_gpio22_cnt++;
			if(get_gpio22_cnt >= 2)
			{
				ring = 1;
				get_gpio22_cnt = 0;
			}
		}

		ringsocket = base->sat.captain_socket;

		if(ringsocket < 0)
		{
			if(gp_users)
			{
				ringsocket = gp_users->socketfd;
			}
		}
		
		if(ring == 1)
		{
			satfi_log("captain_socket=%d ringsocket=%d\n", base->sat.captain_socket, ringsocket);
			base->sat.sat_status = 0;
			base->sat.sat_available = 0;
			base->sat.sat_calling = 1;
			base->sat.EndTime = 0;
			base->sat.StartTime = 0;
			
			base->sat.sat_state_phone = SAT_STATE_PHONE_RING_COMING;
			//satfi_log("SAT_STATE_PHONE_RING_COMING captain_socket=%d\n", base->sat.captain_socket);
			//AppCallUpRsp(base->sat.captain_socket, get_sat_dailstatus());
			char ucbuf[256];
			bzero(ucbuf,sizeof(ucbuf));
			sprintf(ucbuf, "ifdown %s", base->sat.sat_ifname_a);
			myexec(ucbuf, NULL, NULL);
			seconds_sleep(5);
			if(base->sat.sat_fd == -1)
		    {
		    	init_serial(&base->sat.sat_fd, base->sat.sat_dev_name, base->sat.sat_baud_rate);
		    }

			ringnocarriercnt = 0;
			ringcnt = 0;
			int clcccnt = 0;
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

				
				if(strlen(base->sat.called_number) == 0)
				{
					clcccnt++;
					satfi_log("base->sat.captain_socket=%d ringsocket=%d\n", base->sat.captain_socket, ringsocket);
					uart_send(base->sat.sat_fd, "AT+CLCC\r\n", 9);//查询来电电话号码
					if(clcccnt >= 10)
					{
						base->sat.sat_calling = 0;
						break;
					}
				}
				else
				{
					satfi_log("SAT_STATE_PHONE_RING_COMING ringsocket=%d\n", ringsocket);
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
					ringcnt++;
					if(ringcnt >= 60)
					{
						base->sat.sat_state_phone = SAT_STATE_PHONE_COMING_HANGUP;
						satfi_log("SAT_STATE_PHONE_COMING_HANGUP captain_socket=%d\n", base->sat.captain_socket);
						AppCallUpRsp(ringsocket, get_sat_dailstatus());
						break;
					}
				}

				if(base->sat.sat_state_phone == SAT_STATE_PHONE_ONLINE)
				{
					//satfi_log("SAT_STATE_PHONE_ONLINE captain_socket=%d\n", base->sat.captain_socket);
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
	        myexec("power_mode sat_sm2500 reset",NULL,NULL);
	        seconds_sleep(10);
			isNeedReset = 0;
			base->sat.sat_calling = 0;
			base->sat.sat_state_phone = SAT_STATE_PHONE_IDLE;
			close(base->sat.sat_fd);
			base->sat.sat_fd = -1;
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


static void *select_voice_udp(void *p)
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

	char voicebuf[3200];
	struct sockaddr_in clientAddr;
	struct sockaddr_in *clientAddr1 = &(base->sat.clientAddr1);
	struct sockaddr_in *clientAddr2 = &(base->sat.clientAddr2);

	base->sat.voice_socket_udp = sock;

	int len = sizeof(clientAddr);
	
	bzero(&clientAddr, len);
	bzero(clientAddr1, len);
	bzero(clientAddr2, len);
	
	int n;
	struct timeval tout = {10,0};	
	fd_set fds;
	int maxfd;
	//struct timeval tv;

	satfi_log("select_voice_udp %d", sock);

	int rate = 8000;
	int bits = 16;
	int channels = 1;
	int AudioPlayFd = -1;

    while (1)
    {
		FD_ZERO(&fds);/* 每次循环都需要清空 */
		FD_SET(sock, &fds); /* 添加描述符 */
		maxfd = sock;
		tout.tv_sec = 10;
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
					if(AudioPlayFd > 0)
					{
						close(AudioPlayFd);
						AudioPlayFd = -1;
					}
				}
				
				if(ntohs(clientAddr2->sin_port) != 0)
				{
					satfi_log("bzero clientAddr2");
					bzero(clientAddr2, len);
				}
				
				break;
			default:
								
				if(FD_ISSET(sock, &fds))
				{
			        n = recvfrom(sock, voicebuf, 3200, 0, (struct sockaddr*)&clientAddr, &len);
			        if (n>0 && base->sat.sat_calling == 1)
			        {
						if(memcmp(clientAddr1 ,&clientAddr, len) == 0)
						{
							//gettimeofday(&tv, NULL);
							//satfi_log("%s %d %d",inet_ntoa(clientAddr1->sin_addr) ,ntohs(clientAddr.sin_port), AudioPlayFd);
							if(ntohs(clientAddr2->sin_port) != 0)
							{
								if(base->sat.sat_state_phone == SAT_STATE_PHONE_ONLINE)
								{
									n = sendto(sock, voicebuf, n, 0, (struct sockaddr *)clientAddr2, len);
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
									if(AudioPlayFd > 0)
									{
										//satfi_log("AudioPlayFd=%d %d %s %d", AudioPlayFd, n, inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));
										write(AudioPlayFd, voicebuf, n);
									}
									else
									{
										AudioPlayFd = AudioPlayInit(rate,bits,channels);
									}
								}
							}
						}
						else if(memcmp(clientAddr2 ,&clientAddr, len) == 0)
						{
							if(ntohs(clientAddr1->sin_port) != 0 && base->sat.sat_state_phone == SAT_STATE_PHONE_ONLINE)
							{
								//gettimeofday(&tv, NULL);
								//satfi_log(" 								   %s %d %06d %d",inet_ntoa(clientAddr2->sin_addr) ,tv.tv_sec, tv.tv_usec, n);
								n = sendto(sock, voicebuf, n, 0, (struct sockaddr *)clientAddr1, len);
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


static void *send_app_voice_udp(void *p)
{
	BASE *base = (BASE *)p;
	int AudioRecordFd = -1;
	int size = 320;
	int ret;
	int rate = 8000;
	int bits = 16;
	int channels = 1;
	
	char VoiceData[1024];
	
	struct sockaddr_in *clientAddr1 = &(base->sat.clientAddr1);
	int len = sizeof(struct sockaddr_in);
	
	while(1)
	{
		//satfi_log("%d %d %d %d\n", AudioRecordFd, app_socket_voice, base->sat.sat_calling, base->sat.sat_state_phone);
		if(base->sat.sat_calling == 1)
		{
			if(ntohs(clientAddr1->sin_port) != 0)
			{
				if(AudioRecordFd<0)
				{
					AudioRecordFd = AudioRecordInit(rate,bits,channels);
					if(AudioRecordFd>0)
					{
						satfi_log("AudioRecord Init AudioRecordFd=%d\n", AudioRecordFd);
					}
					else
					{
						seconds_sleep(1);
					}
				}
				
				if(AudioRecordFd>0)
				{
					ret = read(AudioRecordFd, VoiceData, size);
					if(ret > 0 && ntohs(clientAddr1->sin_port) != 0)
					{
						ret = sendto(base->sat.voice_socket_udp, VoiceData, ret, 0, (struct sockaddr *)clientAddr1, len);
						if (ret < 0)
						{
							satfi_log("sendto clientAddr11");
						}
						else
						{
							//satfi_log("sendto app voice=%s %d %d\n", inet_ntoa(clientAddr1->sin_addr), ntohs(clientAddr1->sin_port), ret);
						}
					}
				}			
			}
			else
			{
				seconds_sleep(1);
			}
		}
		else
		{
			if(AudioRecordFd>0)
			{
				satfi_log("AudioRecord Exit AudioRecordFd=%d %d\n", AudioRecordFd, base->sat.sat_calling);	
				close(AudioRecordFd);
				AudioRecordFd = -1;
				
				if(ntohs(clientAddr1->sin_port) != 0)
				{
					satfi_log("bzero clientAddr1");
					bzero(clientAddr1, len);
				}
			}

			seconds_sleep(1);
		}
	}
}


int main_fork(void)
{
	fd_set fds;
	int maxfd = 0;
	struct timeval timeout={3,0};
	char ucbuf[256];

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
	if(pthread_create(&id_2, NULL, select_tsc_udp, (void *)&base) == -1) exit(1);

	if(detect_interface(base.sat.sat_ifname) != 0)
	{
		if (!isFileExists(base.sat.sat_dev_name))
		{
			satfi_log("power_mode sat_sm2500 on");
			myexec("power_mode sat_sm2500 on", NULL, NULL);
			seconds_sleep(10);
		}
	}
	
	//sprintf(ucbuf, "ifdown %s", base.sat.sat_ifname_a);
	//myexec(ucbuf, NULL, NULL);
	
	if(detect_interface(base.n3g.n3g_ifname)!=0)
	{
		if (!isFileExists(base.n3g.n3g_dev_name))
		{
			satfi_log("power_mode gprs on");
			myexec("power_mode gprs on", NULL, NULL);
			seconds_sleep(10);
		}
	}
	
	//sprintf(ucbuf, "ifdown %s", base.n3g.n3g_ifname_a);
	//myexec(ucbuf, NULL, NULL);

	//System检测
	if(pthread_create(&id_3, NULL, SystemServer, (void *)&base) == -1) exit(1);

	//sat拨号线程,ring检测
	if(pthread_create(&id_4, NULL, func_y, (void *)&base) == -1) exit(1);
	if(pthread_create(&id_5, NULL, ring_detect, (void *)&base) == -1) exit(1);

	//gprs拨号线程
	if(pthread_create(&id_6, NULL, func_z, (void *)&base) == -1) exit(1);

	//电话音频处理
	//if(pthread_create(&id_7, NULL, recv_app_voice_data, NULL) == -1) exit(1);
	//if(pthread_create(&id_8, NULL, send_app_voice_data, (void *)&base) == -1) exit(1);
	if(pthread_create(&id_7, NULL, select_voice_udp, (void *)&base) == -1) exit(1);
	if(pthread_create(&id_8, NULL, send_app_voice_udp, (void *)&base) == -1) exit(1);

	//gps模块上电
	BdGpsPowerControl(1); //旧版船家宝需要
	init_serial(&base.gps.gps_fd, base.gps.gps_dev_name, base.gps.gps_baud_rate);

	satfi_log("PID=%d PPID=%d\n",getpid(), getppid());
	
	while(1)
	{
		FD_ZERO(&fds);
		maxfd=0;

		if(base.gps.gps_fd > 0)
		{
			FD_SET(base.gps.gps_fd, &fds);
			if(base.gps.gps_fd > maxfd)
			{
				maxfd = base.gps.gps_fd;
			}
		}

		if(base.sat.sat_fd > 0)
		{
			FD_SET(base.sat.sat_fd, &fds);
			if(base.sat.sat_fd > maxfd)
			{
				maxfd = base.sat.sat_fd;
			}
		}

		if(base.n3g.n3g_fd > 0)
		{
			FD_SET(base.n3g.n3g_fd, &fds);
			if(base.n3g.n3g_fd > maxfd)
			{
				maxfd = base.n3g.n3g_fd;
			}
		}

		if(base.gps.usbSerialStm32 > 0)
		{
			FD_SET(base.gps.usbSerialStm32, &fds);
			if(base.gps.usbSerialStm32 > maxfd)
			{
				maxfd = base.gps.usbSerialStm32;
			}
		}
		else
		{
			if(init_serial(&base.gps.usbSerialStm32, "/dev/ttyACM0", 9600) < 0)
			{
				if(init_serial(&base.gps.usbSerialStm32, "/dev/ttyACM1", 9600) < 0)
				{
					seconds_sleep(1);
				}
			}
		}

		if(maxfd == 0)
		{
			seconds_sleep(3);
			continue;
		}

		timeout.tv_sec = 3;
		switch(select(maxfd+1, &fds, NULL, NULL, &timeout))
		{
			case -1: break;
			case  0: break;
			default:
				if(base.gps.gps_fd > 0 && FD_ISSET(base.gps.gps_fd, &fds))
				{
					handle_gps_data(base.gps.gps_fd);	//定位数据
				}

				if(base.sat.sat_fd > 0 && FD_ISSET(base.sat.sat_fd, &fds))
				{
					handle_sat_data(base.sat.sat_fd);	//卫星模块
				}

				if(base.n3g.n3g_fd > 0 && FD_ISSET(base.n3g.n3g_fd, &fds))
				{
					handle_gprs_data(base.n3g.n3g_fd);	//gprs模块
				}
				
				if(base.gps.usbSerialStm32 > 0 && FD_ISSET(base.gps.usbSerialStm32, &fds))
				{
					unsigned char buf[1024] = {0};
					int n = read(base.gps.usbSerialStm32, buf, 1024);
					if(n > 0)
					{
						satfi_log("n=%d base.gps.serverFd=%d\n", n, base.gps.serverFd);
						if(base.gps.serverFd > 0) write(base.gps.serverFd, buf, n);
					}
					else
					{
						close(base.gps.usbSerialStm32);
						base.gps.usbSerialStm32 = 0;
					}
				}
				
				break;
		}
	}

	return 0;
}

void prev_version_clean(void)
{
	if(isFileExists("/etc/rc.d/S01SatFi"))
	{
		remove("/etc/rc.d/S01SatFi");
	}
	
	if(isFileExists("/etc/init.d/SatFi"))
	{
		remove("/etc/init.d/SatFi");
	}
}

int main(int argc, char *argv[])
{
	int status;
	pid_t fpid;

	//等效于ulimit -c unlimited
    struct rlimit coreFileSize;
    bzero(&coreFileSize, sizeof(coreFileSize));
    coreFileSize.rlim_cur = 1000*1024;
    coreFileSize.rlim_max = 4294967295UL;
    setrlimit(RLIMIT_CORE, &coreFileSize);

	//prev_version_clean();

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

