#include<stdio.h>
#include<unistd.h>
#include<error.h>
#include<sys/ipc.h>
#include<sys/types.h>
#include<sys/msg.h>
#include<string.h>
#include<time.h>
#include<sys/timeb.h>
#include"sequence.h"
#include"log.h"

int get_sq_error(int err)
{
  int dwErr;
  switch(err)
  {
  case EACCES:dwErr = SQ_EACCESS;break;
  case EEXIST:dwErr = SQ_EEXIST;break;
  case EIDRM :dwErr = SQ_EIDRM; break;
  case ENOENT:dwErr = SQ_ENOENT;break;
  case ENOMEM:dwErr = SQ_ENOMEM;break;
  case ENOSPC:dwErr = SQ_ENOSPC;break;
  case E2BIG :dwErr = SQ_E2BIG; break;
  case EFAULT:dwErr = SQ_EFAULT;break;
  case EINTR :dwErr = SQ_EINTR; break;
  case ENOMSG:dwErr = SQ_ENOMSG;break;
  case EAGAIN:dwErr = SQ_EAGAIN;break;
  case EINVAL:dwErr = SQ_EINVAL;break;
  default:    dwErr = err;      break;
  }
  return dwErr;
}

int sq_ident(key_t stQueName, int iFlag, int *pdwQueId)
{
  char ucLog[256];
  int iQueId;

  iQueId = msgget(stQueName, iFlag);
  if(iQueId == -1)
  {
    sprintf(ucLog, "QueName:%#x, ErrNo:%#x", stQueName, errno);
    ErrorLog(ucLog, __FUNCTION__,__LINE__);
    return get_sq_error(errno);
  }
  *pdwQueId = (int)iQueId;
  sprintf(ucLog, "sq_ident:%#x", *pdwQueId);
  NormalLog(ucLog);
  return SQ_NOERR;
}

int sq_ctl(int dwQueId, int iQueCnt, int iMsgLen)
{
  char ucLog[256];
  int iErr;
  struct msqid_ds stMsgQid;
  iErr = msgctl(dwQueId, IPC_STAT, &stMsgQid);
  if(iErr == -1)
  {
    sprintf(ucLog, "ErrNo: %#x", errno);
    ErrorLog(ucLog, __FUNCTION__, __LINE__);
    return get_sq_error(errno);
  }
  stMsgQid.msg_qbytes = iQueCnt * iMsgLen;
  iErr = msgctl(dwQueId, IPC_SET, &stMsgQid);
  if(iErr == -1)
  {
    sprintf(ucLog, "ErrNo: %#x", errno);
    ErrorLog(ucLog, __FUNCTION__, __LINE__);
    return get_sq_error(errno);
  }
  return SQ_NOERR;
}

int sq_create(key_t stQueName, int iFlag, int iQueCnt, int iMsgLen, int *pdwQueId)
{
  char ucLog[256];
  int iQueId, iErr;
  struct msqid_ds stMsgQid;
  iQueId = msgget(stQueName, iFlag);
  if(iQueId != -1)
  {
    if(msgctl(iQueId, IPC_RMID, 0) == -1)
    {
      sprintf(ucLog, "ErrNo: %#x", errno);
      ErrorLog(ucLog, __FUNCTION__, __LINE__);
      return get_sq_error(errno);
    }
  }
  iQueId = msgget(stQueName, iFlag|IPC_CREAT);
  if(iQueId==-1)
  {
    sprintf(ucLog, "ErrNo: %#x", errno);
    ErrorLog(ucLog, __FUNCTION__, __LINE__);
    return get_sq_error(errno);
  }
  *pdwQueId = (int)iQueId;
  iErr = msgctl(iQueId, IPC_STAT, &stMsgQid);
  if(iErr==-1)
  {
    sprintf(ucLog, "ErrNo: %#x", errno);
    ErrorLog(ucLog, __FUNCTION__, __LINE__);
    return get_sq_error(errno);
  }
  stMsgQid.msg_qbytes = iQueCnt * iMsgLen;
  iErr = msgctl(iQueId, IPC_SET, &stMsgQid);
  if(iErr==-1)
  {
    printf("ErrNo:%#x", errno);
    iErr = errno;
    msgctl(iQueId, IPC_RMID, 0);
    return get_sq_error(errno);
  }
  return SQ_NOERR;
}

int sq_send(int dwQueId, unsigned char *pucSndMsgBuf, int iMsgLen, int iFlag)
{
  char ucLog[256];
  MSGBUF stSndMsgBuf;

  if(iMsgLen <=0)
  {
    sprintf(ucLog,"sqsend err:%#X",iMsgLen);
    ErrorLog(ucLog,__FUNCTION__,__LINE__);
    return SQ_NOERR;
  }

  if(iMsgLen > MAX_SQ_BUF_LEN)
  {
    sprintf(ucLog,"Msg Too Long:%d",iMsgLen);
    ErrorLog(ucLog,__FUNCTION__,__LINE__);
    return SQ_MSGTOOLONG;
  }

  stSndMsgBuf.mtype = SVR_SND_MSG_TYPE;
  memcpy(stSndMsgBuf.mtext, pucSndMsgBuf, iMsgLen);

  if(msgsnd(dwQueId, &stSndMsgBuf, iMsgLen,iFlag|IPC_NOWAIT) == -1)
  {
    sprintf(ucLog,"ErrNo:%#x,Qid:%d,MsgLen:%d",errno,dwQueId,iMsgLen);
    ErrorLog(ucLog,__FUNCTION__,__LINE__);
    return(get_sq_error(errno));
  }
  Sq_Send_Log(dwQueId, pucSndMsgBuf, iMsgLen);

  return SQ_NOERR;
}

int sq_receive(int dwQueId, unsigned char *pucRcvMsgBuf, int *piMsgLen, int iFlag)
{
  MSGBUF stRcvMsgBuf;
  char ucLog[256];

  *piMsgLen = msgrcv(dwQueId, &stRcvMsgBuf, MAX_SQ_BUF_LEN, SVR_RCV_MSG_TYPE, iFlag|MSG_NOERROR);

  if(*piMsgLen == -1)
  {
    if(errno != ENOMSG)
    {
      sprintf(ucLog,"Qid:%d,ErrNo:%#x",dwQueId,errno);
      ErrorLog(ucLog,__FUNCTION__,__LINE__);
    }
    return(get_sq_error(errno));
  }

  memcpy(pucRcvMsgBuf, stRcvMsgBuf.mtext, *piMsgLen);

  Sq_Rcv_Log(dwQueId, pucRcvMsgBuf, *piMsgLen);

  return SQ_NOERR;
}

int sq_delete(int dwQueId)
{
  char ucLog[256];

  if(msgctl(dwQueId,IPC_RMID,0) == -1)
  {
    sprintf(ucLog,"sq delete err");
    ErrorLog(ucLog,__FUNCTION__,__LINE__);
    return(get_sq_error(errno));
  }
  return SQ_NOERR;
}

int clr_que_msg(key_t stQueName)
{
  int dwQid;
  int dwErr;
  int dwRcvLen;
  char ucLog[256];

  unsigned char ucRcvBuf[MAX_SQ_BUF_LEN];

  sprintf(ucLog,"Start Clear Que:%x...",stQueName);
  NormalLog(ucLog);
  dwErr = sq_ident(stQueName,0777,&dwQid);
  if(dwErr)
  {
    sprintf(ucLog,"Clear Que:%x Fail",stQueName);
    ErrorLog(ucLog,__FUNCTION__,__LINE__);
    return (dwErr);
  }
  while(1)
  {
    dwErr = sq_receive(dwQid, ucRcvBuf, &dwRcvLen, IPC_NOWAIT);
    if(dwErr == SQ_ENOMSG)
    {
      break;
    }
    else if(dwErr == SQ_NOERR)
    {
      continue;
    }
    else
    {
      sprintf(ucLog,"Clear Que:%x Fail",stQueName);
      ErrorLog(ucLog,__FUNCTION__,__LINE__);
      return (dwErr);
    }
  }
  sprintf(ucLog,"Clear Que:%x Ok.",stQueName);
  NormalLog(ucLog);
  return dwErr;
}
