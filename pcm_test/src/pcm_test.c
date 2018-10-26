#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#define PCM_SET_RECORD			0
#define PCM_SET_UNRECORD		1	
#define PCM_READ_PCM			2
#define PCM_START				3
#define PCM_STOP				4
#define PCM_SET_PLAYBACK		5
#define PCM_SET_UNPLAYBACK		6
#define PCM_WRITE_PCM			7
#define PCM_SET_CODEC_TYPE		8
#define PCM_EXT_LOOPBACK_ON		9
#define PCM_EXT_LOOPBACK_OFF	10
#define PCM_PUTDATA				11
#define PCM_GETDATA				12
#define PCM_OPEN				13
#define PCM_CLOSE				14

#define G711ULAW_CODEC				1
#define G711ALAW_CODEC				2
#define G729AB_CODEC				3
#define G723A_CODEC					4

#define PCM_PAGE_SIZE			8000

typedef struct pcm_buffer_t
{
	char* pcmbuf;
	int size;
	int playback_max_size;
}pcm_record_type, pcm_playback_type;

int main(int argc, char **argv)
{		
 	if (argc != 3)
	{
		printf("%s test.raw maxsize\n", argv[0]);
		return -1;
	}

	char voicebuf[64000];
	int pcmfd=-1;
	int ret;
	
	pcm_playback_type playback;
	playback.pcmbuf = voicebuf;
	playback.playback_max_size = atoi(argv[2]);

	int iRecordCH=0;
	pcmfd = open("/dev/pcm0", O_RDWR);
	ioctl(pcmfd, PCM_OPEN, &ret);
	printf("PCM_OPEN=%d %d\n", ret, playback.playback_max_size);
	if (ret < 0)
	{
		ioctl(pcmfd, PCM_CLOSE);
		return NULL;
	}
	ioctl(pcmfd, PCM_SET_PLAYBACK, &iRecordCH);
	ioctl(pcmfd, PCM_SET_RECORD, &iRecordCH);
	ioctl(pcmfd, PCM_START, 0);
	
	int fdr = open(argv[1], O_RDWR);
	if (fdr < 0)
	{
		printf("can't open %s\n", argv[1]);
		return -1;
	}
	
	while(1)
	{
		//printf("readfdr=%d\n", record.size);
		ret=read(fdr, voicebuf, 320);
		if(ret != 320)
		{
			break;
		}
		//usleep(13000);
		playback.size = ret;
		ioctl(pcmfd, PCM_WRITE_PCM, &playback);
	}

	//sleep(5);
	printf("quit\n");		
	ioctl(pcmfd, PCM_STOP, 0);
	ioctl(pcmfd, PCM_SET_UNPLAYBACK);
	ioctl(pcmfd, PCM_SET_UNRECORD);
	ioctl(pcmfd, PCM_CLOSE);
	
	close(fdr);
	
	return 0; 	
}

