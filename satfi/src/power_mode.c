#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <curses.h>
#include <string.h> 
#include <linux/serial.h>


#define gprs_on				1
#define gprs_off			2
#define sat_system_reset	3
#define sat_on				4
#define sat_off				5
#define gps_on				6
#define gps_off				7

void print_usage(char *file)
{
	printf("Usage:\n");
	printf("%s <dev> <on|off|switch>\n",file);
	printf("eg. \n");
	printf("%s gprs on\n", file);
	printf("%s gprs off\n", file);
	printf("%s msm01a reset\n", file);
	printf("%s msm01a on\n", file);
	printf("%s msm01a off\n", file);
}

int main(int argc, char **argv)
{
	if (argc != 3){
		print_usage(argv[0]);
		return 0;
	}	
	
	int fd_power = open("/dev/power_mode",O_RDWR | O_NONBLOCK);
	if(fd_power<0)return -1;
	
	if(!strcmp("gprs", argv[1])){
		if (!strcmp("on", argv[2])){
			ioctl(fd_power, gprs_on);
		}
		else if (!strcmp("off", argv[2])){
			ioctl(fd_power, gprs_off);
		}
		else{
			print_usage(argv[0]);
			return 0;
		}
	}
	else if(!strcmp("msm01a", argv[1])){
		if (!strcmp("reset", argv[2])){
			ioctl(fd_power, sat_system_reset);
		}
		else if (!strcmp("on", argv[2])){
			ioctl(fd_power, sat_on);
		}
		else if (!strcmp("off", argv[2])){
			ioctl(fd_power, sat_off);
		}
		else{
			print_usage(argv[0]);
			return 0;
		}
	}
	else{
		print_usage(argv[0]);
		return 0;
	}
	
}
