#ifndef __TSC_H__
#define __TSC_H__

#define SATFI_CONNECT_CMD					0X0000	/*SATFI CONNECT TSC*/
#define SATFI_HEART_BEAT_CMD				0X0001	/*SATFI HEART_BEAT TSC*/
#define SATFI_HEART_BEAT_SP_CMD				0X0016	/*SATFI HEART_BEAT TSC*/
#define SATFI_MS_LIST_CMD					0X0019	/*SATFI HEART_BEAT TSC*/

#define SATFI_CONNECT_CMD_RSP				0X8000	/*SATFI CONNECT TSC RSP*/
#define SATFI_HEART_BEAT_CMD_RSP			0X8001	/*SATFI HEART BEAT RSP*/
#define SATFI_HEART_BEAT_SP_CMD_RSP			0X8016	/*SATFI_HEART_BEAT_SP_CMD_RSP*/
#define SATFI_MS_LIST_CMD_RSP				0X8019	/*SATFI_MS_LIST_CMD_RSP*/

#define TSC_NOTIFY_RSP						0X0008	/*服务器下发通知给APP*/
#define TSC_NO_ENOUGHMONEY_RSP				0X0010	/*服务器下发余额不足给APP*/
#define TSC_NOTIFY_EMERGENCY_ALARM_CMD_RSP	0X0014	/*通知同一个企业内部的终端紧急报警*/
#define TSC_NOTIFY_CANCEL_EMERGENCY_CMD_RSP	0X0018	/*通知终端取消报警指令*/

#define APP_QUERY_GROUP_CMD					0X0002	/*APP查询所在的全部群组*/
#define APP_QUERY_MS_CMD					0X0003	/*APP查询组内的所有终端*/
#define APP_SET_BLOCK_CMD					0X0004	/*APP切换群组*/
#define APP_UPLOAD_MESSAGE_CMD				0X0005	/*APP发送文字信息*/
#define APP_UPLOAD_VOICE_CMD				0X0006	/*APP发送语音信息*/
#define APP_UPLOAD_PICTURE_CMD				0X0007	/*APP发送图片信息*/
#define APP_READ_OK_CMD						0X0009	/*APP发送读取消息成功*/
#define APP_GET_MESSAGE_CMD					0X0011	/*APP读取未读消息*/
#define APP_ZF_MESSAGE_CMD					0X0012	/*APP转发消息*/
#define APP_EMERGENCY_ALARM_CMD				0X0013	/*APP终端紧急报警*/
#define APP_CANCEL_EMERGENCY_ALARM_CMD		0X0015	/*APP终端取消紧急报警*/
#define APP_OPERAT_CMD						0X0017	/*APP同步操作*/

#define APP_QUERY_GROUP_CMD_RSP				0X8002	/*APP RSP FROM TSC*/
#define APP_QUERY_MS_CMD_RSP				0X8003	
#define APP_SET_BLOCK_CMD_RSP				0X8004	
#define APP_UPLOAD_MESSAGE_CMD_RSP			0X8005	
#define APP_UPLOAD_VOICE_CMD_RSP			0X8006	
#define APP_UPLOAD_PICTURE_CMD_RSP			0X8007	
#define APP_READ_OK_CMD_RSP					0X8009	
#define APP_GET_MESSAGE_CMD_RSP				0X8011	
#define APP_ZF_MESSAGE_RSP					0X8012	
#define APP_EMERGENCY_ALARM_CMD_RSP			0X8013	
#define APP_CANCEL_EMERGENCY_ALARM_CMD_RSP	0X8015	
#define APP_OPERAT_CMD_RSP					0X8017	

//群组
#define APP_PHONE_MONEY_CMD			  		0X0020
#define APP_PHONE_MONEY_RSP           		0X8020

#define APP_MESSAGE_MONEY_CMD			  	0X0054
#define APP_MESSAGE_MONEY_RSP           	0X8054

#define APP_GRP_UPLOAD_MESSAGE_CMD    		0X0021
#define APP_GRP_UPLOAD_MESSAGE_RSP    		0X8021
#define APP_GRP_UPLOAD_VOICE_CMD      		0X0022
#define APP_GRP_UPLOAD_VOICE_RSP      		0X8022
#define APP_GRP_UPLOAD_PICTURE_CMD     		0X0023
#define APP_GRP_UPLOAD_PICTURE_RSP    		0X8023
#define APP_GRP_NOTIFY_CMD            		0X0024
#define APP_GRP_READ_OK_CMD           		0X0025
#define APP_GRP_READ_OK_RSP           		0X8025
#define APP_SET_GRP_CAHT_CMD          		0X0026
#define APP_SET_GRP_CAHT_RSP          		0X8026

//调度查询/设置船家宝数据
#define GS_CJB_DATA							0X0027				
#define GS_CJB_DATA_RSP						0X8027				

//船家宝日志
#define CJB_LOG								0X0028				
#define CJB_LOG_RSP							0X8028				

#define APP_SET_MSG_CMD						0X0029				
#define APP_SET_MSG_RSP						0X8029				

#define BLIND_AREA_GPS						0X0030
#define BLIND_AREA_GPS_RSP					0X8030

#define APP_SHARE_GPS						0X0031
#define APP_WL_COMMAND						0X0032

#define APP_SET_ONLINE						0X0033
#define APP_SET_ONLINE_RSP					0X8033

#define TSC_NOTIFY_MS						0X0034

#define APP_GPS_POINT						0X0035
#define APP_GPS_POINT_RSP					0X8035

#define APP_PTT								0X0036
#define APP_PTT_RSP							0X8036

#define APP_PTT_IN							0X0037
#define APP_PTT_IN_RSP						0X8037

#define APP_HUP								0X0038
#define APP_HUP_RSP							0X8038

#define APP_SPTT							0X0039
#define APP_SPTT_HUP						0X0040
#define APP_MO_VOICE						0X0041
#define APP_MT_VOICE						0X0042
#define APP_SPTT_NOTIFY						0X0043

#define APP_HUPIN							0X0044
#define APP_HUPIN_RSP						0X8044

#define APP_MS_HUP							0X0045
#define APP_MS_HUP_RSP						0X8045

#define FLAG_NOTIFY							0X1046//no use
#define FLAG_NOTIFY_RSP						0X1046//no use

#define APP_CLR_SPTT						0X0046
#define APP_CLR_SPTT_RSP					0X8046

//new
#define APP_ASPTT_REQ						0X0047
#define APP_ASPTT_REQ_RSP					0X8047

#define APP_ASPTT_HUP						0X0048
#define APP_ASPTT_HUP_RSP					0X8048

#define APP_ASPTT_CLER						0X0049
#define APP_ASPTT_CLER_RSP					0X8049

#define APP_ASPTT_CANCEL					0X0050

#define APP_MMT_VOICE						0X0051

#define PPT_VOICE							0X0052

#pragma pack (1)

typedef struct _header
{
  unsigned short length;
  unsigned short mclass;
} Header;

/**/
/*SATFI_CONNECT_CMD_RSP*/
typedef struct _connect_rsp
{
	Header header;
	char FixMsID[9];
	unsigned short Flag:8;//0:1 冻结:激活
} Msg_Connect_Rsp;

/*APP_QUERY_GROUP_CMD*/
typedef struct _query_group_req
{
	Header header;
	char MsID[6];
} Msg_Query_Group_Req;

typedef struct _group
{
	char GrpID[11];
	char GrpName[30];
	unsigned short Config:8;
	unsigned short Creator:8;
	unsigned short IsGrpChat:8;
} Group;

/*APP_QUERY_GROUP_CMD_RSP*/
typedef struct _query_group_rsp
{
	Header header;
	char MsID[6];
	unsigned short Count:8;//group Count
	Group group[0]; 
} Msg_Query_Group_Rsp;

/*APP_QUERY_MS_CMD*/
typedef struct _query_ms_req
{
	Header header;
	char MsID[6];
	char GrpID[11];
} Msg_Query_Ms_Req;

typedef struct _terminal
{
	char MsID[6];
	unsigned short Config:8;
	char MsName[30];
	unsigned short Type:8;
	unsigned short OnlineStatus:8;
} Terminal;

typedef struct _query_ms_rsp
{
	Header header;
	char MsID[6];
	char GrpID[11];
	unsigned short Count;//terminal Count
	Terminal terminal[0];
} Msg_QueryMs_Rsp;

/*APP_SET_BLOCK_CMD*/
typedef struct _set_block_req
{
	Header header;
	char MsID[6];
	char GrpID[11];
} Msg_SetBlock_Req;

typedef struct _set_block_rsp
{
	Header header;
	char MsID[6];
	char GrpID[11];
	unsigned short Result:8;
} Msg_SetBlock_Rsp;

/*APP_UPLOAD_MESSAGE_CMD*/
typedef struct _upload_message_req
{
	Header header;
	char MsID[6];
	char TargetGrpID[6];
	int Name;
	char Message[0]; 
} Msg_Upload_Message_Req;

typedef struct _upload_message_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;
	int Name;
} Msg_Upload_Message_Rsp;


/*APP_UPLOAD_VOICE_CMD*/
typedef struct _upload_voice_req
{
	Header header;
	char MsID[6];
	char TargetGrpID[6];
	unsigned int Name;
	unsigned int lengthtotal;
	unsigned short packseq;
	unsigned short packtotal;
	char data[0];
} Msg_Upload_Voice_Req;

typedef struct _pack_seq
{
	unsigned short pack_seq_1;
	unsigned short pack_seq_2;
} PackSeq;

typedef struct _upload_voice_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;
	int Name;
	unsigned short packnum;/*当Result=1时，如下字段存在*/
	PackSeq packseq[0];
} Msg_Upload_Voice_Rsp;

/*APP_UPLOAD_PICTURE_CMD*/
typedef struct _upload_picture_req
{
	Header header;
	char MsID[6];
	char TargetGrpID[6];
	unsigned int Name;
	unsigned int lengthtotal;
	unsigned short packseq;
	unsigned short packtotal;
	char data[0];
} Msg_Upload_Picture_Req;

typedef struct _upload_picture_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;
	int Name;
	unsigned short packnum;/*当Result=1时，如下字段存在*/
	PackSeq packseq[0]; 
} Msg_Upload_Picture_Rsp;

/*APP_READ_OK_CMD*/
typedef struct _read_ok_req
{
	Header header;
	char MsID[6];
	unsigned int ID;
	unsigned short Type:8;
} Msg_Read_OK_Req;

typedef struct _read_ok_rsp
{
	Header header;
	char MsID[6];
	int ID;
	unsigned short Result:8;
} Msg_Read_OK_Rsp;

/*APP_GET_MESSAGE_CMD*/
typedef struct _get_message_req
{
	Header header;
	char MsID[6];
} Msg_Get_Message_Req;

typedef struct _get_message_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;/*当Result=1时，如下字段存在*/
	char SrcMsID[6];
	unsigned short Type:8;
	unsigned int ID;
	char Date[8];
	char message[0];
} Msg_Get_Message_Rsp;

/*APP_ZF_MESSAGE_CMD*/
typedef struct _zf_message1_req
{
	Header header;
	char MsID[6];
	unsigned short Type:8;
	char TargetID[6];/*6或11*/
	int ID;
	unsigned short MessageType:8;
	char Path[0];
} Msg_Zf_Message1_Req;

typedef struct _zf_message2_req
{
	Header header;
	char MsID[6];
	unsigned short Type:8;
	char TargetID[11];/*6或11*/
	int ID;
	unsigned short MessageType:8;
	char Path[0];
} Msg_Zf_Message2_Req;

typedef struct _zf_message_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;
	int ID;
} Msg_Zf_Message_Rsp;

/*APP_EMERGENCY_ALARM_CMD*/
typedef struct _emergency_alarm_req
{
	Header header;
	char MsID[6];
	unsigned int Lg;
	unsigned int Lt;
	unsigned short AlarmType:8;
} Msg_Emergency_Alarm_Req;

typedef struct _emergency_alarm_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;
} Msg_Emergency_Alarm_Rsp;

/*APP_CANCEL_EMERGENCY_ALARM_CMD*/
typedef struct _cancel_emergency_alarm_req
{
	Header header;
	char MsID[6];
	unsigned short Type:8;
	char Remark[0];
} Msg_Cancel_Emergency_Alarm_Req;

typedef struct _cancel_emergency_alarm_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;
} Msg_Cancel_Emergency_Alarm_Rsp;

/*APP_OPERAT_CMD*/
typedef struct _operat_req
{
	Header header;
	char MsID[6];
	unsigned short Operation:8;
	char GrpID[11];
	char GrpName[0];
} Msg_App_Operat_Req;

typedef struct _operat_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;
} Msg_App_Operat_Rsp;

/*TSC_NOTIFY_CMD*/
typedef struct _notify_rsp
{
	Header header;
	char MsID[6];
	unsigned int  ID;
	unsigned short Type:8;
} Msg_Notify_Rsp;

/*TSC_NO_ENOUGHMONEY_CMD*/
typedef struct _noenough_money_rsp
{
	Header header;
	char MsID[6];
	unsigned int  Money;
} Msg_NoEnough_Money_Rsp;

/*TSC_NOTIFY_EMERGENCY_ALARM_CMD_RSP*/
typedef struct _notify_emergency_alarm_rsp
{
	Header header;
	char MsID[6];
	char MsName[30];
	unsigned int Lg;
	unsigned int Lt;
	unsigned short AlarmType:8;
} Msg_Notify_Emergency_Alarm_Rsp;

/*TSC_NOTIFY_CANCEL_EMERGENCY_CMD_RSP*/
typedef struct _notify_cancel_emergency_alarm_rsp
{
	Header header;
	char MsID[6];
	char toMsID[6];
	unsigned short Type:8;
	char Remark[0];
} Msg_Notify_Cancel_Emergency_Alarm_Rsp;

/*SATFI_CONNECT_CMD*/
typedef struct _connect_req
{
	Header header;
	char sat_imsi[15];
} Msg_Connect_Req;

/*SATFI_MS_LIST_CMD*/
typedef struct _ms_list_rep
{
	Header header;
	char sat_imsi[15];
	unsigned short usercnt;
	char Users[0];
} Msg_MS_List_Rep;

//Phone Money
typedef struct _phone_money_req
{
	Header header;
	char MsID[6];
	char MsPhoneNum[6];
	char DesPhoneNum[6];
	unsigned long long StartTime;
	unsigned short CallTime;
	unsigned int Money;
} Msg_Phone_Money_Req;

typedef struct _phone_money_rsp
{
	Header header;
	char MsID[6];
	unsigned int Money;
} Msg_Phone_Money_Rsp;

//APP_MESSAGE_MONEY_CMD
typedef struct _message_money_req
{
	Header header;
	char MsID[6];
	char MsPhoneNum[6];
	char DesPhoneNum[6];
	unsigned long long StartTime;
	unsigned int Money;
} Msg_Message_Money_Req;

typedef struct _message_money_rsp
{
	Header header;
	char MsID[6];
	unsigned int Money;
} Msg_Message_Money_Rsp;

//Grp Upload Message
typedef struct _grp_upload_message_req
{
	Header header;
	char MsID[6];
	char TargetGrpID[11];
	int Name;
	char Message[0]; 
} Msg_Grp_Upload_Message_Req;

typedef struct _grp_upload_message_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;
	int Name;
} Msg_Grp_Upload_Message_Rsp;

//Grp Upload Voice
typedef struct _grp_upload_voice_req
{
	Header header;
	char MsID[6];
	char TargetGrpID[11];
	unsigned int Name;
	unsigned int lengthtotal;
	unsigned short packseq;
	unsigned short packtotal;
	char data[0];
} Msg_Grp_Upload_Voice_Req;

typedef struct _grp_upload_voice_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;
	int Name;
	unsigned short packnum;/*当Result=1时，如下字段存在*/
	PackSeq packseq[0];
} Msg_Grp_Upload_Voice_Rsp;

//Grp Upload Picture
typedef struct _grp_upload_picture_req
{
	Header header;
	char MsID[6];
	char TargetGrpID[11];
	unsigned int Name;
	unsigned int lengthtotal;
	unsigned short packseq;
	unsigned short packtotal;
	char data[0];
} Msg_Grp_Upload_Picture_Req;

typedef struct _grp_upload_picture_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;
	int Name;
	unsigned short packnum;/*当Result=1时，如下字段存在*/
	PackSeq packseq[0]; 
} Msg_Grp_Upload_Picture_Rsp;

//Grp Notify
typedef struct _grp_notify_rsp
{
	Header header;
	char MsID[6];
	char TargetGrpID[11];
	unsigned int  Name;
	unsigned short Type:8;//type=0,Message  type=1、2 Path
	char data[0];
} Msg_Grp_Notify_Rsp;

//Grp Read OK
typedef struct _grp_read_ok_req
{
	Header header;
	char MsID[6];
	unsigned int ID;
	unsigned short Type:8;
} Msg_Grp_Read_OK_Req;

typedef struct _grp_read_ok_rsp
{
	Header header;
	char MsID[6];
	int ID;
	unsigned short Result:8;
} Msg_Grp_Read_OK_Rsp;

//Set Grp Chat
typedef struct _set_grp_chat_req
{
	Header header;
	char MsID[6];
	char GrpID[11];
	unsigned short Type:8;
	unsigned short MsgType:8;
} Msg_Set_Grp_Chat_Req;

typedef struct _set_grp_chat_rsp
{
	Header header;
	char MsID[6];
	char GrpID[11];
	unsigned short Type:8;
	unsigned short MsgType:8;
} Msg_Set_Grp_Chat_Rsp;

//GS_CJB_DATA
typedef struct _gs_cjb_data
{
	Header header;
	char Sat_IMSI[15];
	unsigned short Type:8;
} Msg_Gs_Cjb_Data;

typedef struct _gs_cjb_gps_rsp
{
	Header header;
	char Sat_IMSI[15];
	unsigned short Type:8;
	int Lg;
	char Lg_D;
	int Lt;
	char Lt_D;
} Msg_Gs_Cjb_Gps_Rsp;

//CJB_LOG

//APP_SET_MSG_CMD
typedef struct _set_msg_req
{
	Header header;
	char MsID[6];
	unsigned short Type:8;
	char TargetMsID[6];
} Msg_Set_Msg_Req;

typedef struct _set_msg_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result:8;
	char TargetMsID[6];
} Msg_Set_Msg_Rsp;

//BLIND_AREA_GPS
typedef struct _blind_area_gps
{
	Header header;
	char Sat_IMSI[15];
	int Lg;						//经度
	char Lg_D;					//东西经
	int Lt;						//纬度
	char Lt_D;					//南北纬
	unsigned long long Date;	//时间戳
	int Speed;					//速度
} Msg_Blind_Area_Gps;

typedef struct _blind_area_gps_rsp
{
	Header header;
	unsigned long long Date;
} Msg_Blind_Area_Gps_Rsp;

//APP_SHARE_GPS
typedef struct _share_gps
{
	Header header;
	char MsID[6];
	char SrcMsID[6];
	int Lg;
	char Lg_D;
	int Lt;
	char Lt_D;
} Msg_Share_Gps;

//APP_WL_COMMAND
typedef struct _wl_command
{
	Header header;
	char MsID[6];
	char Msg[0];
} Msg_Wl_Command;

//APP_SET_ONLINE
typedef struct _set_online_req
{
	Header header;
	char MsID[6];
	char GrpID[11];
	unsigned short Type:8;
} Msg_Set_Online_Req;

typedef struct _set_online_rsp
{
	Header header;
	char MsID[6];
	char GrpID[11];
	unsigned short Type:8;
} Msg_Set_Online_Rsp;

//TSC_NOTIFY_MS
typedef struct _notify_ms
{
	Header header;
	char MsID[6];
	char SrcMsID[6];
	char GrpID[11];
	unsigned short Type:8;
} Msg_Notify_Ms;

//APP_GPS_POINT
typedef struct _gps_point_req
{
	Header header;
	char MsID[6];
	unsigned short Type:8;
	int Id;
	char Operator[40];
	char data[0];
//	unsigned long long Date;
//	char Content[];
} Msg_Gps_Point_Req;

typedef struct _gps_point_rsp
{
	Header header;
	char MsID[6];
	int Id;
	unsigned short Result:8;
} Msg_Gps_Point_Rsp;

//FLAG_NOTIFY
typedef struct _flag_notify
{
	Header header;
	char Sat_IMSI[15];
	unsigned short Flag:8;
	int Id;
} Msg_Flag_Notify;

typedef struct _flag_notify_rsp
{
	Header header;
	int Id;
} Msg_Flag_Notify_Rsp;

//APP_PTT
typedef struct _ptt
{
	Header header;
	char MsID[6];
	char GrpID[11];
} Msg_Ptt;

typedef struct _ptt_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result;
} Msg_Ptt_Rsp;

//APP_PTT_IN
typedef struct _ptt_in
{
	Header header;
	char MsID[6];
	char GrpID[11];
	char ToMsID[6];
	char FromMsID[6];
	char TSC[6];
	char PTT_MS_Name[30];
} Msg_Ptt_In;

//PTT_VOICE
typedef struct _ptt_voice
{
	Header header;
	char FromMsID[6];
} Msg_Ptt_Voice;

typedef struct _ptt_in_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result;
} Msg_Ptt_In_Rsp;

//APP_HUP
typedef struct _hup
{
	Header header;
	char MsID[6];
	char GrpID[11];
} Msg_Hup;

typedef struct _hup_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result;
} Msg_Hup_Rsp;

//APP_SPTT
typedef struct _sptt
{
	Header header;
	char MsID[6];
	char GrpID[11];
	unsigned short MS_Count;
	char data[0];
} Msg_Sptt;

//APP_SPTT_HUP
typedef struct _sptt_hup
{
	Header header;
	char MsID[6];
	char GrpID[11];
	char Reserve[8];
} Msg_Sptt_Hup;

//APP_MO_VOICE
typedef struct _mo_voice
{
	Header header;
	char MsID[6];
	char data[0]; //voicedata + Reserve[8]
} Msg_Mo_Voice;

//APP_MT_VOICE
typedef struct _mt_voice
{
	Header header;
	char MsID[6];
	char data[0];
} Msg_Mt_Voice;

//APP_MMT_VOICE
typedef struct _mmt_voice
{
	Header header;
	unsigned short MsNum;
	char MsID[6];
//	char data[0];
} Msg_Mmt_Voice;

//APP_SPTT_NOTIFY
typedef struct _sptt_notify
{
	Header header;
	char MsID[6];
	char GrpID[11];
	unsigned short Operation:8;
} Msg_Sptt_Notify;

//APP_HUPIN
typedef struct _hupin
{
	Header header;
	char MsID[6];
	char GrpID[11];
	char Dest_MsID[6];
	unsigned short Option:8;
} Msg_Hupin;

typedef struct _hupin_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result;
} Msg_Hupin_Rsp;

//APP_MS_HUP
typedef struct _ms_hup
{
	Header header;
	char MsID[6];
	char Src_Name[21];
	unsigned short Operation:8;
} Msg_Ms_Hup;

typedef struct _ms_hup_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result;
} Msg_Ms_Hup_Rsp;

//APP_CLR_SPTT
typedef struct _clr_sptt
{
	Header header;
	char MsID[6];
} Msg_Clr_Sptt;

typedef struct _clr_sptt_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result;
} Msg_Ms_Clr_Sptt_Rsp;

//APP_ASPTT_REQ
typedef struct _asptt
{
	Header header;
	char MsID[6];
	char TARGET_MS_Id[6];
	char Reserve[8];
} Msg_Asptt;

typedef struct _asptt_rsp
{
	Header header;
	char MsID[6];
	char TARGET_MS_Id[6];
	unsigned short Result;
} Msg_Asptt_Rsp;

//APP_ASPTT_HUP
typedef struct _asptt_hup
{
	Header header;
	char MsID[6];
	char Reserve[8];
} Msg_Asptt_Hup;

typedef struct _asptt_hup_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result;
} Msg_Asptt_Hup_Rsp;

//APP_ASPTT_CLER
typedef struct _asptt_cler
{
	Header header;
	char MsID[6];
	char TARGET_MS_Id[6];
	char Reserve[8];
} Msg_Asptt_Cler;

typedef struct _asptt_cler_rsp
{
	Header header;
	char MsID[6];
	unsigned short Result;
} Msg_Asptt_Cler_Rsp;

//APP_ASPTT_CANCEL
typedef struct _asptt_cancel
{
	Header header;
	char MsID[6];
	char TARGET_MS_Id[6];
	char Reserve[8];
} Msg_Asptt_Cancel;

#pragma pack ()

#endif

