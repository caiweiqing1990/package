/* According to POSIX.1-2001 */
#include <sys/select.h>

/* According to earlier standards */
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/* 初始化串口设备
 * @fd
 * @device
 * @baud_rate
 */
int init_serial(int *fd, char *device, int baud_rate)
{
  satfi_log("open serial port : %s ...\n", device);
  int fd_serial = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_serial < 0)
  {
    perror("open fd_serial");
    return -1;
  } 

  *fd = fd_serial;

  /* 串口主要设置结构体termios <termios.h> */
  struct termios options;

  /* 1.tcgetattr()用于获取与终端相关的参数
   *   参数fd为终端的文件描述符，返回的结果保存在termios结构体中
   */
  tcgetattr(fd_serial, &options);

  /* 2.修改获得的参数 */
  options.c_cflag |= CLOCAL | CREAD; /* 设置控制模块状态：本地连接，接收使能 */
  options.c_cflag &= ~CSIZE;         /* 字符长度，设置数据位之前，一定要屏蔽这一位 */
  options.c_cflag &= ~CRTSCTS;       /* 无硬件流控 */
  options.c_cflag |= CS8;            /* 8位数据长度 */
  options.c_cflag &= ~CSTOPB;        /* 1位停止位 */
  options.c_iflag |= IGNPAR;         /* 无奇偶校验 */
  options.c_oflag = 0;               /* 输出模式 */
  options.c_lflag = 0;               /* 不激活终端模式 */
  cfsetospeed(&options, baud_rate);    /* 设置波特率 */

  /* 3.设置新属性: TCSANOW，所有改变立即生效 */
  tcflush(fd_serial, TCIFLUSH);      /* 溢出数据可以接收，但不读 */
  tcsetattr(fd_serial, TCSANOW, &options);

  satfi_log("open serial port : %s successfully!!!\n", device);
  return 0;
}

/* 串口发送数据
 * @fd:     串口描述符
 * @data:   待发送数据
 * @datalen:数据长度
 */
int uart_send(int fd, char *data, int datalen)
{
  int len = 0;
  len = write(fd, data, datalen); /* 实际写入的长度 */
  if (len == datalen)
  {
    return len;
  }
  else
  {
    tcflush(fd, TCOFLUSH); /* TCOFLUSH，刷新写入的数据，但不传送 */
    return -1;
  }

  return 0;
}

/* 串口接收数据 */
int uart_recv(int fd, char *data, int datalen)
{
  int len = 0, ret = 0;
  fd_set fs_read;
  struct timeval tv = {1,0};

  FD_ZERO(&fs_read);
  FD_SET(fd, &fs_read);

  ret = select(fd+1, &fs_read, NULL, NULL, &tv);

  if (ret>0)
  {
    if(FD_ISSET(fd, &fs_read))
    {
      len = read(fd, data, datalen);
      return len;
    }
  }

  return 0;
}