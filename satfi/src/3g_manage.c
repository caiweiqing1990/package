#include <sys/types.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
       
/*
 * �ж�/dev/ttyUSB'index'�Ƿ����ж����͵�endpoint
 */
int hasInterruptEndpoint(int index)
{
    /* 1. ����ÿһ��/dev/ttyUSBX
     *    ����һ����Ӧ��/sys/class/tty/ttyUSBX
     * 2. ����һ�������ļ�, ָ��: /sys/......../1-1:1.0/ttyUSB0/tty/ttyUSB0
     * 3. ����/sys/......../1-1:1.0/Ŀ¼,
     *    �����ж��"ep_"����Ŀ¼
     * 4. ��Ŀ¼������Ϊtype���ļ�
     * 5. �������ļ�������ΪInterrupt, �򷵻�1
     *    
     */
    char tmpBuf[1024];
    char tmpBuf2[1024];
    int cnt;
    char *ptr;

    DIR *dir;
    struct dirent *entry;

    int fd;

    sprintf(tmpBuf, "/sys/class/tty/ttyUSB%d", index);
    cnt = readlink(tmpBuf, tmpBuf2, 1024);
    if (cnt == -1)
    {
        return 0;
    }
    else
    {
        tmpBuf2[cnt] = '\0';
    }
    sprintf(tmpBuf, "/sys/class/tty/%s", tmpBuf2); /* tmpBuf: /sys/devices/platform/s3c2410-ohci/usb1/1-1/1-1:1.3/ttyUSB2/tty/ttyUSB2 */
    ptr = strstr(tmpBuf, "/ttyUSB");
    if (!ptr)
    {
        return 0;
    }
    tmpBuf[ptr-tmpBuf] = '\0';  /* tmpBuf: /sys/......../1-1:1.0 */

    //printf("Dir: %s\n", tmpBuf);

    dir = opendir(tmpBuf);
    if (!dir)
    {
        printf("can not open %s\n", tmpBuf);
        return 0;
    }
    while((entry = readdir(dir)))
    {
        if (strncasecmp(entry->d_name, "ep_", 3) == 0)
        {
            sprintf(tmpBuf2, "%s/%s/type", tmpBuf, entry->d_name);
            //printf("type: %s\n", tmpBuf2);
            fd = open(tmpBuf2, O_RDONLY);
            if (fd < 0)
            {
                printf("can not open %s\n", tmpBuf2);
                return 0;
            }
            read(fd, tmpBuf2, 1024);
            //printf("endpoint: %s\n", tmpBuf2);
            if (!strncmp(tmpBuf2, "Interrupt", 9))
            {
                return 1;
            }
            close(fd);
        }
    }
    return 0;
}

/* 
 * ����/dev/gsmmodem����, ָ��ĳ��/dev/ttyUSBX
 * 3g_manager link ttyUSB0,1,2
 */
int do_link(int argc, char **argv)
{
    char cmdBuf[1024];
    int index;
    int i;
    
    /*
     * ����ж��/dev/ttyUSBX���ж����Ͷ˵�, 
     * ��/dev/gsmmodemָ����С��/dev/ttyUSBX
     */
    sscanf(argv[1]+6, "%d", &index);
	printf("%d\n",index);
    for (i = 0; i <= index; i++)
    {
        if (hasInterruptEndpoint(i))
        {
            sprintf(cmdBuf, "ln -s /dev/ttyUSB%d /dev/ttyGPRS", i);
            system("rm -f /dev/gsmmodem");
            system(cmdBuf);
            return 0;
        }
    }
    return -1;
}

/* ����usbģʽ�л�: ����usb_modeswitch
 * ����/dev/gsmmodem����, ָ��ĳ��/dev/ttyUSBX
 */

/* �÷�:
 * 3g_manager /dev/ttyUSBX
 */
int main(int argc, char **argv)
{
	if(argc != 2)
	{
        printf("Usage:\n");
        printf("%s <ttyUSBX>   : Create Link to /dev/ttyUSBX if it has interrupt endpoint\n", argv[0]);
        return -1;
	}
    return do_link(argc, argv);
}

