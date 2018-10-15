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
	int i;
	int fd;
	int fdr;
	int fdw;
	int ret;
	int iRecordCH=0;
	long codec_type[2] = {iRecordCH, G711ULAW_CODEC};
	
	pcm_record_type record;
	
 	if (argc != 4)
	{
		printf("%s /dev/pcm0 test.wav maxsize\n", argv[0]);
		return -1;
	}

	fd = open(argv[1], O_RDWR);
	if (fd < 0)
	{
		printf("can't open %s\n", argv[1]);
		return -1;
	}
	
	fdr = open(argv[2], O_RDWR);
	if (fdr < 0)
	{
		printf("can't open %s\n", argv[2]);
		return -1;
	}
	
	//fdw = open(argv[3], O_RDWR|O_CREAT);
	//if (fdw < 0)
	//{
	//	printf("can't open %s\n", argv[3]);
	//	return -1;
	//}
	
	ioctl(fd, PCM_OPEN, &ret);
	printf("PCM_OPEN=%d\n", ret);
	if (ret < 0)
	{
		ioctl(fd, PCM_CLOSE);
		return -1;
	}
	ioctl(fd, PCM_SET_PLAYBACK, &iRecordCH);
	ioctl(fd, PCM_SET_RECORD, &iRecordCH);
	
	ioctl(fd, PCM_SET_CODEC_TYPE, codec_type);
	
	ioctl(fd, PCM_START, 0);
	
	record.pcmbuf = malloc(PCM_PAGE_SIZE);
	record.size = 320;
	record.playback_max_size = PCM_PAGE_SIZE;
	if(record.pcmbuf == NULL)
	{
		printf("can't malloc for record\n");
		ioctl(fd, PCM_STOP, 0);
		ioctl(fd, PCM_SET_UNPLAYBACK);
		ioctl(fd, PCM_SET_UNRECORD);
		
		ioctl(fd, PCM_CLOSE);
		return -1;		
	}
	
	int offset=0;
	while(1)
	{
		//printf("readfdr=%d\n", record.size);
		//ret = read(fdr, record.pcmbuf, PCM_PAGE_SIZE);
		//printf("readfdr=%d\n", ret);
		ioctl(fd, PCM_READ_PCM, &record);
		offset+=record.size;
		usleep(20000);
		if(offset >= PCM_PAGE_SIZE)
		{
			break;
		}
	}

	//sleep(5);
	printf("quit\n");		
	ioctl(fd, PCM_STOP, 0);
	ioctl(fd, PCM_SET_UNPLAYBACK);
	ioctl(fd, PCM_SET_UNRECORD);
	
	ioctl(fd, PCM_CLOSE);
	close(fd);
	close(fdr);
	close(fdw);
	free(record.pcmbuf);
	
	return 0; 	
}

