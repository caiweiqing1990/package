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

/* ��ʼ�������豸
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

  /* ������Ҫ���ýṹ��termios <termios.h> */
  struct termios options;

  /* 1.tcgetattr()���ڻ�ȡ���ն���صĲ���
   *   ����fdΪ�ն˵��ļ������������صĽ��������termios�ṹ����
   */
  tcgetattr(fd_serial, &options);

  /* 2.�޸Ļ�õĲ��� */
  options.c_cflag |= CLOCAL | CREAD; /* ���ÿ���ģ��״̬���������ӣ�����ʹ�� */
  options.c_cflag &= ~CSIZE;         /* �ַ����ȣ���������λ֮ǰ��һ��Ҫ������һλ */
  options.c_cflag &= ~CRTSCTS;       /* ��Ӳ������ */
  options.c_cflag |= CS8;            /* 8λ���ݳ��� */
  options.c_cflag &= ~CSTOPB;        /* 1λֹͣλ */
  options.c_iflag |= IGNPAR;         /* ����żУ�� */
  options.c_oflag = 0;               /* ���ģʽ */
  options.c_lflag = 0;               /* �������ն�ģʽ */
  cfsetospeed(&options, baud_rate);    /* ���ò����� */

  /* 3.����������: TCSANOW�����иı�������Ч */
  tcflush(fd_serial, TCIFLUSH);      /* ������ݿ��Խ��գ������� */
  tcsetattr(fd_serial, TCSANOW, &options);

  satfi_log("open serial port : %s successfully!!!\n", device);
  return 0;
}

/* ���ڷ�������
 * @fd:     ����������
 * @data:   ����������
 * @datalen:���ݳ���
 */
int uart_send(int fd, char *data, int datalen)
{
  int len = 0;
  len = write(fd, data, datalen); /* ʵ��д��ĳ��� */
  if (len == datalen)
  {
    return len;
  }
  else
  {
    tcflush(fd, TCOFLUSH); /* TCOFLUSH��ˢ��д������ݣ��������� */
    return -1;
  }

  return 0;
}

/* ���ڽ������� */
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