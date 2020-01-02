#ifndef __LOG_H__
#define __LOG_H__

#include<time.h>
#include<sys/types.h>
#include<unistd.h>

void NormalLog(char *pLog);
void ErrorLog(char *pLog, const char *pFunc, int dwLine);
void Sq_Rcv_Log( int dwQid, unsigned char *pBuf, int Len);
void Sq_Send_Log( int dwQid, unsigned char *pBuf, int Len);

void satfi_log_printf(const char *fmt,...);

//#define satfi_log(...)  
#define satfi_log satfi_log_printf


#endif
