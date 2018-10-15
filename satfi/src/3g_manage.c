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
 * 判断/dev/ttyUSB'index'是否有中断类型的endpoint
 */
int hasInterruptEndpoint(int index)
{
    /* 1. 对于每一个/dev/ttyUSBX
     *    都有一个对应的/sys/class/tty/ttyUSBX
     * 2. 它是一个链接文件, 指向: /sys/......../1-1:1.0/ttyUSB0/tty/ttyUSB0
     * 3. 进入/sys/......../1-1:1.0/目录,
     *    里面有多个"ep_"的子目录
     * 4. 子目录里有名为type的文件
     * 5. 如果这个文件的内容为Interrupt, 则返回1
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
 * 创建/dev/gsmmodem链接, 指向某个/dev/ttyUSBX
 * 3g_manager link ttyUSB0,1,2
 */
int do_link(int argc, char **argv)
{
    char cmdBuf[1024];
    int index;
    int i;
    
    /*
     * 如果有多个/dev/ttyUSBX有中断类型端点, 
     * 则/dev/gsmmodem指向最小的/dev/ttyUSBX
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

/* 进行usb模式切换: 调用usb_modeswitch
 * 创建/dev/gsmmodem链接, 指向某个/dev/ttyUSBX
 */

/* 用法:
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

