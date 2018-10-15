#include <stdio.h>
#include <string.h>
#include <math.h>
#include <wchar.h>
#include <iconv.h>

//cp ./staging_dir/target-mipsel_24kec+dsp_uClibc-0.9.33.2/usr/lib/libiconv-stub/include/iconv.h ./staging_dir/toolchain-mipsel_24kec+dsp_gcc-4.8-linaro_uClibc-0.9.33.2/include/

//cp ./staging_dir/target-mipsel_24kec+dsp_uClibc-0.9.33.2/usr/lib/libiconv-stub/lib/libiconv.a ./staging_dir/toolchain-mipsel_24kec+dsp_gcc-4.8-linaro_uClibc-0.9.33.2/lib/

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


int SendMsg(char *phonenum, char *data)
{
	unsigned char msgbuf[1024] = {0};
	unsigned char tmp[1024] = {0};
	unsigned char userdata[1024] = {0};
	
	int i;
	int plen = strlen(phonenum);
	int dlen = strlen(data);
	int outlen = sizeof(tmp);
	int msglen;
	
	strcat(msgbuf, "00");//SCA 服务中心号码
	strcat(msgbuf, "01");//PDU-Type 00表示收，01表示发
	strcat(msgbuf, "00");//MR
	
	bzero(tmp, sizeof(tmp));
	sprintf(tmp,"%02x", plen);
	//printf("%s\n",tmp);
	strcat(msgbuf, tmp);//0B 电话号码长度
	strcat(msgbuf, "81");//国内是A1或81 国际是91

	bzero(tmp, sizeof(tmp));
	if(plen%2)
	{
		//号码长度奇数
		for(i=0; i<plen-1;i+=2)
		{
			tmp[i] = phonenum[i+1];
			tmp[i+1] = phonenum[i];
		}
		tmp[i] = 'F';
		tmp[i+1] = phonenum[i];
		//printf("%s\n",tmp);
	}
	else
	{
		//号码长度偶数
		for(i=0; i<plen;i+=2)
		{
			tmp[i] = phonenum[i+1];
			tmp[i+1] = phonenum[i];
		}
		//printf("%s\n",tmp);
	}
	strcat(msgbuf, tmp);//电话号码
	strcat(msgbuf, "00");//PID
	strcat(msgbuf, "08");//DCS 	00h 7bit数据编码 默认字符集 
						 //		F6h 8bit数据编码 Class1
						 //		08h USC2（16bit）双字节字符集
	
	//bzero(userdata, sizeof(userdata));	
	code_convert("UTF-16BE", "UTF-8", data, &dlen, userdata, &outlen);

	bzero(tmp, sizeof(tmp));
	sprintf(tmp,"%02x", outlen);
	//printf("%s\n",tmp);
	strcat(msgbuf, tmp);//UDL 用户数据长度

	bzero(tmp, sizeof(tmp));	
	for (i=0; i<outlen; i++)
	{
		//printf("[%d].%02x ",i,userdata[i]);
		sprintf(&tmp[i*2],"%02x", userdata[i]);
	}
	strcat(msgbuf, tmp);//UD 用户数据
	printf("%s\n",msgbuf);
	
	msglen = strlen(msgbuf)/2 - 1;//for AT+CMGS=msglen
	
	return msglen;
}

int main()
{
	int r = SendMsg("13112121509", "看看abc");
	printf("%d\n",r);
	return 0 ;
}
