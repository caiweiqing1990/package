#ifndef __SQ_H__
#define __SQ_H__

#include <errno.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/msg.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#define MAX_SQ_BUF_LEN 2048
#define SVR_SND_MSG_TYPE  1
#define SVR_RCV_MSG_TYPE  0

#define SQ_NOERR 0
#define SQ_EACCESS 1
#define SQ_EEXIST 2
#define SQ_EIDRM 3
#define SQ_ENOENT 4
#define SQ_ENOMEM 5
#define SQ_ENOSPC 6
#define SQ_E2BIG 7
#define SQ_EFAULT 8
#define SQ_EINTR 9
#define SQ_ENOMSG 10
#define SQ_EAGAIN 11
#define SQ_EINVAL 12
#define SQ_MSGTOOLONG 13

typedef struct {
  long mtype;
  unsigned char mtext[MAX_SQ_BUF_LEN];
}MSGBUF;

int get_sq_error(int err);
int sq_ident(key_t stQueName, int iFlg, int *pdwQueId);
int sq_ctl(int dwQueId, int iQueCnt, int iMsgLen);
int sq_create(key_t stQueName, int iFlg, int iQueCnt, int iMsgLen, int *pdwQueId);
int sq_send(int dwQueId, unsigned char *pucSndMsgBuf, int iMsgLen, int iFlg);
int sq_receive(int dwQueId, unsigned char *pucRcvMsgBuf, int *piMsgLen, int iFlg);
int sq_delete(int dwQueId);
int Clr_que_msg(key_t stQueName);

#endif

