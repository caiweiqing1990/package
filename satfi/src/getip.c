#include<stdio.h>
#include<netdb.h>
#include<stdlib.h>
#include <string.h>

#define MAXLINE 1024
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

int checkroute(char *ifname, const char *addr, int checkaddr)
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


int main(int argc, char **argv)
{
  struct addrinfo *res, *pt;
  struct sockaddr_in *sinp;
  const char *addr;
  char abuf[INET_ADDRSTRLEN];
  int succ=0, i=0;

  char cmd[128];
  
  if (argc != 3)
  {
    printf("Usage: %s <server name> <route>\nFor example:\n        %s www.google.com 3g-wan0\n", argv[0], argv[0]);
    exit(1);
  }

  succ = getaddrinfo(argv[1], NULL, NULL, &res);
  if (succ != 0)
  {
    printf("Can't get address info : error code = %d\n", succ);
    exit(succ);
  }

  for (pt=res,i=0; pt!=NULL;pt=pt->ai_next,i++)
  {
    sinp=(struct sockaddr_in *)pt->ai_addr;
    addr=(const char *)inet_ntop(AF_INET,&sinp->sin_addr,abuf,INET_ADDRSTRLEN);
	if(checkroute(argv[2], addr, 1) != 0)
	{
		printf("%2d.IP=%s\n",i,addr);
		sprintf(cmd,"route add %s dev %s", addr, argv[2]);
		system(cmd);	
		//sprintf(cmd,"echo %s %s >> /etc/hosts", addr, argv[1]);
		//system(cmd);
		//return 0;
	}
	else
	{
		printf("%2d.IP=%s exist\n",i,addr);
	}
  }
}
