#ifndef __MSG_H__
#define __MSG_H__

#define CONNECT_CMD                 0X0000
#define HEART_BEAT_CMD              0X0001
#define SENDOPERATION_CMD           0X0002
#define SENDLANMESSAGE1_CMD         0X0003
#define QUERY_GROUP_CMD             0X0004
#define QUERY_MS_CMD                0X0005
#define SET_BLOCK_CMD               0X0006
#define UPLOAD_MESSAGE_CMD          0X0007
#define UPLOAD_VOICE_CMD            0X0008
#define UPLOAD_PICTURE_CMD          0X0009
#define READ_OK_CMD                 0X000A 
#define GET_MESSAGE_CMD             0X000B
#define NOTIFY_CMD                  0X000C //
#define NOTENOUGH_MONEY_CMD         0X000D //
#define CALLUP_CMD                  0X000E
#define HANGINGUP_CMD               0X000F
#define RINGUP_CMD                  0X0010
#define ANSWERINGPHONE_CMD          0X0011
#define ZF_MESSAGE_CMD              0X0012
#define EMERGENCY_ALARM_CMD         0X0013
#define NOTIFY_EMERGENCY_ALARM_CMD  0X0014 //
#define CANCEL_EMERGENCY_ALARM_CMD	0X0015
#define QUERY_MS_LIST_CMD			0X0016
#define OPERAT_CMD					0X0017
#define NOTIFY_CANCEL_EMERGENCY_CMD 0X0018
#define GET_LOG_CMD 				0X0019

#define CONNECT_RSP                 0X8000
#define HEART_BEAT_RSP              0X8001
#define SENDOPERATION_RSP           0X8002
#define SENDLANMESSAGE1_RSP         0X8003
#define QUERY_GROUP_RSP             0X8004
#define QUERY_MS_RSP                0X8005
#define SET_BLOCK_RSP               0X8006
#define UPLOAD_MESSAGE_RSP          0X8007
#define UPLOAD_VOICE_RSP            0X8008
#define UPLOAD_PICTURE_RSP          0X8009
#define READ_OK_RSP                 0X800A
#define GET_MESSAGE_RSP             0X800B
#define CALLUP_RSP                  0X800E
#define HANGINGUP_RSP               0X800F
#define RINGUP_RSP                  0X8010
#define ANSWERINGPHONE_RSP          0X8011
#define ZF_MESSAGE_RSP              0X8012
#define EMERGENCY_ALARM_RSP         0X8013
#define CANCEL_EMERGENCY_ALARM_RSP	0X8015
#define QUERY_MS_LIST_RSP			0X8016
#define OPERAT_RSP					0X8017
#define GET_LOG_CMD_RSP				0X8019

//群组
#define PHONE_MONEY_CMD			  	0X0020
#define PHONE_MONEY_RSP           	0X8020

#define GRP_UPLOAD_MESSAGE_CMD    	0X0021
#define GRP_UPLOAD_MESSAGE_RSP    	0X8021
#define GRP_UPLOAD_VOICE_CMD      	0X0022
#define GRP_UPLOAD_VOICE_RSP      	0X8022
#define GRP_UPLOAD_PICTURE_CMD     	0X0023
#define GRP_UPLOAD_PICTURE_RSP    	0X8023

#define GRP_NOTIFY_CMD            	0X0024

#define GRP_READ_OK_CMD           	0X0025
#define GRP_READ_OK_RSP           	0X8025
#define SET_GRP_CHAT_CMD          	0X0026
#define SET_GRP_CHAT_RSP          	0X8026

#define GRP_CHAT_CMD          		0X0027
#define GRP_CHAT_RSP          		0X8027

#define SENDLANMESSAGE2_CMD         0X0028
#define SENDLANMESSAGE2_RSP         0X8028

#define SAT_MODE_SWITCH_CMD         0X0029
#define SAT_MODE_SWITCH_CMD_RSP     0X8029

#define SET_WIFI_PASSWD_CMD         0X0030
#define SET_WIFI_PASSWD_CMD_RSP     0X8030

#define SATFI_RESTART_CMD         	0X0031
#define SATFI_RESTART_CMD_RSP     	0X8031

#define SATFI_UPDATE_CMD         	0X0032
#define SATFI_UPDATE_CMD_RSP     	0X8032

#define NOTIFY_ALL_USER_SAT_STATE 	0X0033

#define SET_MSG_CMD          		0X0034
#define SET_MSG_RSP          		0X8034

#define SHARE_GPS          			0X0035
#define WL_COMMAND1          		0X0036
#define WL_COMMAND2          		0X0037

#define SET_ONLINE					0X0038
#define SET_ONLINE_RSP				0X8038

#define NOTIFY_MS					0X0039

#define GPS_POINT					0X0040
#define GPS_POINT_RSP				0X8040

#define PTT							0X0041
#define PTT_RSP						0X8041

#define PTT_IN						0X0042
#define PTT_IN_RSP					0X8042

#define HUP							0X0043
#define HUP_RSP						0X8043

#define SPTT						0X0044
#define SPTT_HUP					0X0045

#define MO_VOICE					0X0046
#define MT_VOICE					0X0047
#define SPTT_NOTIFY					0X0048

#define HUPIN						0X0049
#define HUPIN_RSP					0X8049

#define MS_HUP						0X0050
#define MS_HUP_RSP					0X8050

#define CLR_SPTT					0X0051
#define CLR_SPTT_RSP				0X8051

//new
#define ASPTT_REQ					0X0052
#define ASPTT_REQ_RSP				0X8052

#define ASPTT_HUP					0X0053
#define ASPTT_HUP_RSP				0X8053

#define ASPTT_CLER					0X0054
#define ASPTT_CLER_RSP				0X8054

#define ASPTT_CANCEL				0X0055

#define SATFI_PTT_IN				0X0056
#define SATFI_PTT_HUPIN				0X0057

#define SEND_MESSAGE				0x0058
#define SEND_MESSAGE_RESP			0x8058

#define RECV_MESSAGE				0x0059

#define MODIFY_FAMILY_PHONE			0x0060
#define ADD_FAMILY_PHONE			0x0061
#define REMOVE_FAMILY_PHONE			0x0062

#pragma pack (1)

typedef struct _msg_header
{
  unsigned short length;
  unsigned short mclass;
}MsgHeader;

/*CONNECT_CMD*/
typedef struct _app_connect_req
{
  MsgHeader header;
  char MsID[21];
  unsigned short Count;
  char FamiliarityNumber[10][21];
} MsgAppConnectReq;

/*CONNECT_RSP*/
typedef struct _app_connect_rsp
{
  MsgHeader header;
  int result;
  short n3g_status;
  short sat_status;
  char n3g_imei[16];
  char n3g_imsi[16];
  char sat_imei[16];
  char sat_imsi[16];
  char version[32];
}MsgAppConnectRsp;

/*HEART_BEAT_CMD*/
typedef struct _app_heartbeat_req
{
  MsgHeader header;
  int Type;			//0:普通终端 1:调度 3：船长
  char userid[21];
}MsgAppHeartbeatReq;

/*HEART_BEAT_RSP*/
typedef struct _app_heartbeat_rsp
{
  MsgHeader header;
  int result;
  short n3g_status;
  char n3g_imei[16];
  char n3g_imsi[16];
  char n3g_csq[32];
  short sat_status;
  char sat_imei[16];
  char sat_imsi[16];
  char sat_gps[128];
  char sat_csq[32];
  char bd_gps[128];
  char version[32];
}MsgAppHeartbeatRsp;

/*QUERY_GROUP_CMD*/
typedef struct _app_query_group_req
{
	MsgHeader header;
	char MsID[21];
}MsgQueryGroupReq;

typedef struct _app_group
{
	char GrpID[21];
	char GrpName[30];
	unsigned short Config;
	unsigned short Creator;
	unsigned short IsGrpChat;
}MsgGroup;

/*QUERY_GROUP_RSP*/
typedef struct _app_query_group_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Count;//group Count
	MsgGroup msggroup[0]; 
}MsgQueryGroupRsp;

/*QUERY_MS_CMD*/
typedef struct _app_query_ms_req
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
}MsgQueryMsReq;

typedef struct _app_terminal
{
	char MsID[21];
	unsigned short Config;
	char MsName[30];
	unsigned short Type;
	unsigned short OnlineStatus;
}MsgTerminal;

/*QUERY_MS_RSP*/
typedef struct _app_query_ms_rsp
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
	unsigned short Count;//terminal Count
	MsgTerminal msgterminal[0];
}MsgQueryMsRsp;

/*SET_BLOCK_CMD*/
typedef struct _app_set_block_req
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
}MsgSetBlockReq;

/*SET_BLOCK_RSP*/
typedef struct _app_set_block_rsp
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
	unsigned short Result;
}MsgSetBlockRsp;

/*UPLOAD_MESSAGE_CMD*/
typedef struct _app_upload_message_req
{
	MsgHeader header;
	char MsID[21];
	char TargetGrpID[21];
	int Name;
	char Message[0]; 
}MsgUploadMessageReq;

/*UPLOAD_MESSAGE_RSP*/
typedef struct _app_upload_message_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
	int Name;
}MsgUploadMessageRsp;

/*UPLOAD_VOICE_CMD*/
typedef struct _app_upload_voice_req
{
	MsgHeader header;
	char MsID[21];
	char TargetGrpID[21];
	unsigned int Name;
	unsigned int lengthtotal;
	unsigned short packseq;
	unsigned short packtotal;
	char data[0];
}MsgUploadVoiceReq;

typedef struct _app_pack_seq
{
	unsigned short pack_seq_1;
	unsigned short pack_seq_2;
}MsgPackSeq;

/*UPLOAD_VOICE_RSP*/
typedef struct _app_upload_voice_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
	int Name;
	unsigned short packnum;//packseq nu /*当Result=1时，如下字段存在*/m
	MsgPackSeq msgpackseq[0];
}MsgUploadVoiceRsp;

/*UPLOAD_PICTURE_CMD*/
typedef struct _app_upload_picture_req
{
	MsgHeader header;
	char MsID[21];
	char TargetGrpID[21];
	unsigned int Name;
	unsigned int lengthtotal;
	unsigned short packseq;
	unsigned short packtotal;
	char data[0];
}MsgUploadPictureReq;

/*UPLOAD_PICTURE_RSP*/
typedef struct _app_upload_picture_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
	int Name;
	unsigned short packnum;//packseq nu /*当Result=1时，如下字段存在*/m
	MsgPackSeq msgpackseq[0]; 
}MsgUploadPictureRsp;

/*READ_OK_CMD*/
typedef struct _app_read_ok_req
{
	MsgHeader header;
	char MsID[21];
	unsigned int ID;
	unsigned short Type;
}MsgReadOKReq;

/*READ_OK_RSP*/
typedef struct _app_read_ok_rsp
{
	MsgHeader header;
	char MsID[21];
	int ID;
	unsigned short Result;
}MsgReadOKRsp;

/*GET_MESSAGE_CMD*/
typedef struct _app_get_message_req
{
	MsgHeader header;
	char MsID[21];
}MsgGetMessageReq;

/*GET_MESSAGE_RSP*/
typedef struct _app_get_message_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;/*当Result=1时，如下字段存在*/
	char SrcMsID[21];
	unsigned short Type;
	unsigned int ID;
	char Date[8];
	char message[0];
}MsgGetMessageRsp;

/*NOTIFY_CMD*/
typedef struct _app_notify_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned int  ID;
	unsigned short Type;
}MsgNotifyRsp;

/*NOTENOUGH_MONEY_CMD*/
typedef struct _app_noenough_money_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned int  Money;
}MsgNoEnoughMoneyRsp;


/*ZF_MESSAGE_CMD*/
typedef struct _app_zf_message1_req
{
	MsgHeader header;
	char MsID[21];
	unsigned short Type;
	char TargetID[21];
	int ID;
	unsigned short MessageType;
	char Path[0];
}MsgZfMessageReq;

/*ZF_MESSAGE_RSP*/
typedef struct _app_zf_message_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
	int ID;
}MsgZfMessageRsp;

/*EMERGENCY_ALARM_CMD*/
typedef struct _app_emergency_alarm_req
{
	MsgHeader header;
	char MsID[21];
	unsigned int Lg;
	unsigned int Lt;
	unsigned short AlarmType;
}MsgEmergencyAlarmReq;

/*EMERGENCY_ALARM_RSP*/
typedef struct _app_emergency_alarm_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
}MsgEmergencyAlarmRsp;

/*NOTIFY_EMERGENCY_ALARM_CMD*/
typedef struct _app_notify_emergency_alarm_rsp
{
	MsgHeader header;
	char MsID[21];
	char MsName[30];
	unsigned int Lg;
	unsigned int Lt;
	unsigned short AlarmType;
}MsgNotifyEmergencyAlarm_Rsp;

/*CANCEL_EMERGENCY_ALARM_CMD*/
typedef struct _app_cancel_emergency_alarm_req
{
	MsgHeader header;
	char MsID[21];
	unsigned short Type;
	char Remark[0];
}MsgCancelEmergencyAlarm_Req;

/*CANCEL_EMERGENCY_ALARM_RSP*/
typedef struct _app_cancel_emergency_alarm_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
}MsgCancelEmergencyAlarm_Rsp;

/*QUERY_MS_LIST_CMD*/
typedef struct _app_query_ms_list_req
{
	MsgHeader header;
	char MsID[21];
}MsgQueryMsList_Req;

typedef struct _app_query_ms_list_rsp
{
	MsgHeader header;
	unsigned short MsNum;
	char UserList[0];
}MsgQueryMsList_Rsp;

/*OPERAT_CMD*/
typedef struct _app_operat_req
{
	MsgHeader header;
	char MsID[21];
	unsigned short Operation;
	char GrpID[21];
	char GrpName[0];
} MsgAppOperat_Req;

typedef struct _app_operat_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
} MsgAppOperat_Rsp;

/*NOTIFY_CANCEL_EMERGENCY_CMD*/
typedef struct _app_notify_cancel_emergency_alarm_rsp
{
	MsgHeader header;
	char MsID[21];
	char toMsID[21];
	unsigned short Type;
	char Remark[0];
} MsgNotifyCancelEmergencyAlarm_Rsp;

/*CALL*/
typedef struct _app_callup_rsp
{
  MsgHeader header;
  short result;
}MsgAppCallUpRsp;

typedef struct _app_hangup_rsp
{
  MsgHeader header;
  short result;
}MsgAppHangUpRsp;

typedef struct _app_ringup_rsp
{
  MsgHeader header;
  char called_number[15];
}MsgAppRingUpRsp;

//Phone Money
typedef struct _app_phone_money_req
{
	MsgHeader header;
	char MsID[21];
	char MsPhoneNum[11];
	char DesPhoneNum[11];
	unsigned long long StartTime;
	unsigned short CallTime;
	unsigned int Money;
} MsgPhoneMoneyReq;

typedef struct _app_phone_money_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned int Money;
} MsgPhoneMoneyRsp;

//Grp Upload Message
typedef struct _app_grp_upload_message_req
{
	MsgHeader header;
	char MsID[21];
	char TargetGrpID[21];
	int Name;
	char Message[0]; 
} MsgGrpUploadMessageReq;

typedef struct _app_grp_upload_message_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
	int Name;
} MsgGrpUploadMessageRsp;

//Grp Upload Voice
typedef struct _app_grp_upload_voice_req
{
	MsgHeader header;
	char MsID[21];
	char TargetGrpID[21];
	unsigned int Name;
	unsigned int lengthtotal;
	unsigned short packseq;
	unsigned short packtotal;
	char data[0];
} MsgGrpUploadVoiceReq;

typedef struct _app_grp_upload_voice_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
	int Name;
	unsigned short packnum;/*当Result=1时，如下字段存在*/
	MsgPackSeq msgpackseq[0];
} MsgGrpUploadVoiceRsp;

//Grp Upload Picture
typedef struct _app_grp_upload_picture_req
{
	MsgHeader header;
	char MsID[21];
	char TargetGrpID[21];
	unsigned int Name;
	unsigned int lengthtotal;
	unsigned short packseq;
	unsigned short packtotal;
	char data[0];
} MsgGrpUploadPictureReq;

typedef struct _app_grp_upload_picture_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
	int Name;
	unsigned short packnum;/*当Result=1时，如下字段存在*/
	MsgPackSeq msgpackseq[0]; 
} MsgGrpUploadPictureRsp;

//Grp Notify
typedef struct _app_grp_notify_rsp
{
	MsgHeader header;
	char MsID[21];
	char TargetGrpID[21];
	unsigned int  Name;
	unsigned short Type;//type=0,Message  type=1、2 Path
	char data[0];
} MsgGrpNotifyRsp;

//Grp Read OK
typedef struct _app_grp_read_ok_req
{
	MsgHeader header;
	char MsID[21];
	unsigned int ID;
	unsigned short Type;
} MsgGrpReadOKReq;

typedef struct _app_grp_read_ok_rsp
{
	MsgHeader header;
	char MsID[21];
	int ID;
	unsigned short Result;
} MsgGrpReadOKRsp;

//Set Grp Chat
typedef struct _app_set_grp_chat_req
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
	unsigned short Type;
	unsigned short MsgType;
} MsgSetGrpChatReq;

typedef struct _app_set_grp_chat_rsp
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
	unsigned short Type;
	unsigned short MsgType;
} MsgSetGrpChatRsp;

//Grp Chat
typedef struct _app_grp_chat_req
{
	MsgHeader header;
	char MsID[21];
	char ID[21];
	char TargetGrpID[21];
	unsigned int  Name;
	unsigned short Type;//type=0,Message  type=1、2 Path
	char data[0];
} MsgGrpChatReq;

typedef struct _app_grp_chat_rsp
{
	MsgHeader header;
	char MsID[21];
	char TargetGrpID[21];
	unsigned int  Name;
	unsigned short Type;//type=0,Message  type=1、2 Path
	char data[0];
} MsgGrpChatRsp;

//SET_MSG_CMD
typedef struct _app_set_msg_req
{
	MsgHeader header;
	char MsID[21];
	unsigned short Type;
	char TargetMsID[21];
} MsgSetMsgReq;

typedef struct _app_set_msg_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
	char TargetMsID[21];
} MsgSetMsgRsp;

//SHARE_GPS
typedef struct _app_share_gps
{
	MsgHeader header;
	char MsID[21];
	char SrcMsID[21];
	int Lg;
	unsigned short Lg_D;
	int Lt;
	unsigned short Lt_D;
} MsgShareGps;

//WL_COMMAND2
typedef struct _app_wl_command
{
	MsgHeader header;
	char MsID[21];
	char Msg[0];
} MsgWlCommand;

//SET_ONLINE
typedef struct _app_set_online_req
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
	unsigned short Type;
} MsgSetOnlineReq;

typedef struct _app_set_online_rsp
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
	unsigned short Type;
} MsgSetOnlineRsp;

//NOTIFY_MS
typedef struct _app_notify_ms
{
	MsgHeader header;
	char MsID[21];
	char SrcMsID[21];
	char GrpID[21];
	unsigned short Type;
} MsgNotifyMs;

//GPS_POINT
typedef struct _app_gps_point_req
{
	MsgHeader header;
	char MsID[21];
	unsigned short Type;
	int Id;
	char Operator[40];
	char data[0];
//	unsigned long long Date;
//	char Content[];
} MsgGpsPointReq;

typedef struct _app_gps_point_rsp
{
	MsgHeader header;
	char MsID[21];
	int Id;
	unsigned short Result;
} MsgGpsPointRsp;

//PTT
typedef struct _app_ptt
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
} MsgPtt;

typedef struct _app_ptt_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
} MsgPttRsp;

//PTT_IN
typedef struct _app_ptt_in
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
	char ToMsID[21];
	char FromMsID[21];
	char TSC[6];
	char PTT_MS_Name[30];
} MsgPttIn;

typedef struct _app_ptt_in_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
} MsgPttInRsp;

//HUP
typedef struct _app_hup
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
} MsgHup;

typedef struct _app_hup_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
} MsgHupRsp;

//SPTT
typedef struct _app_sptt
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
	unsigned short MS_Count;
	char data[0];
} MsgSptt;

//SPTT_HUP
typedef struct _app_sptt_hup
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
	char Reserve[8];
} MsgSpttHup;

//MO_VOICE
typedef struct _app_mo_voice
{
	MsgHeader header;
	char MsID[21];
	char data[0]; //voicedata + Reserve[8]
} MsgMoVoice;

//MT_VOICE
typedef struct _app_mt_voice
{
	MsgHeader header;
	char MsID[21];
	char data[0];
} MsgMtVoice;

//SPTT_NOTIFY
typedef struct _app_sptt_notify
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
	unsigned short Operation;
} MsgSpttNotify;

//HUPIN
typedef struct _app_hupin
{
	MsgHeader header;
	char MsID[21];
	char GrpID[21];
	char Dest_MsID[21];
	unsigned short Option;
} MsgHupin;

typedef struct _app_hupin_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
} MsgHupinRsp;

//MS_HUP
typedef struct _app_ms_hup
{
	MsgHeader header;
	char MsID[21];
	char Src_Name[21];
	unsigned short Operation;
} MsgMsHup;

typedef struct _app_ms_hup_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
} MsgMsHupRsp;

//CLR_SPTT
typedef struct _app_clr_sptt
{
	MsgHeader header;
	char MsID[21];
} MsgClrSptt;

typedef struct _app_clr_sptt_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
} MsgClrSpttRsp;


//ASPTT_REQ
typedef struct _app_asptt
{
	MsgHeader header;
	char MsID[21];
	char TARGET_MS_Id[21];
	char Reserve[8];
} MsgAsptt;

typedef struct _app_asptt_rsp
{
	MsgHeader header;
	char MsID[21];
	char TARGET_MS_Id[21];
	unsigned short Result;
} MsgAspttRsp;

//ASPTT_HUP
typedef struct _app_asptt_hup
{
	MsgHeader header;
	char MsID[21];
	char Reserve[8];
} MsgAspttHup;

typedef struct _app_asptt_hup_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
} MsgAspttHupRsp;

//ASPTT_CLER
typedef struct _app_asptt_cler
{
	MsgHeader header;
	char MsID[21];
	char TARGET_MS_Id[21];
	char Reserve[8];
} MsgAspttCler;

typedef struct _app_asptt_cler_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
} MsgAspttClerRsp;

//ASPTT_CANCEL
typedef struct _app_asptt_cancel
{
	MsgHeader header;
	char MsID[21];
	char TARGET_MS_Id[21];
	char Reserve[8];
} MsgAspttCancel;

//SEND_MESSAGE
typedef struct _app_send_msg
{
	MsgHeader header;
	char MsID[21];
	char phonenum[21];
	int ID;
	char messagedata[0];
} MsgSendMsg;

typedef struct _app_send_msg_rsp
{
	MsgHeader header;
	char MsID[21];
	unsigned short Result;
	int ID;
} MsgSendMsgRsp;

//RECV_MESSAGE
typedef struct _app_recv_msg
{
	MsgHeader header;
	char MsID[21];
	char phonenum[21];
	char messagedata[0];
} MsgRecvMsg;

//MODIFY_FAMILY_PHONE
typedef struct _app_modify_family_phone
{
	MsgHeader header;
	char MsID[21];
	char oldPhone[21];
	char newPhone[21];
} MsgModifyPamilyPhone;

//ADD_FAMILY_PHONE
typedef struct _app_add_family_phone
{
	MsgHeader header;
	char MsID[21];
	char Phone[21];
} MsgAddPamilyPhone;

//REMOVE_FAMILY_PHONE
typedef struct _app_remove_family_phone
{
	MsgHeader header;
	char MsID[21];
	char Phone[21];
} MsgRemovePamilyPhone;

#pragma pack ()

#endif
