#include "led_control.h"

int led_fd = -1;
int led_ctl(int Cmd)
{
	if(led_fd < 0)
	{
		led_fd = open(LED_DRIVER_NAME,O_RDWR);
		if(led_fd < 0)
		{
			perror("open led_driver");
			return -1;
		}
	}

	if(ioctl(led_fd, Cmd, NULL) < 0)
	{
		perror("ioctl led");
		close(led_fd);
		led_fd = -1;
		return -1;		
	}

	//close(led_fd);
	
	return 0;
}

