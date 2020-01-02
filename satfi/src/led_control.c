#include "led_control.h"



int led_fd = -1;
int led_control(int Cmd)
{	
	if(led_fd < 0)
	{
		led_fd = open(LED_DRIVER_NAME,O_RDWR);
		if(led_fd < 0)
		{
			perror("led_control open led_driver");
			return -1;
		}
	}

	if(ioctl(led_fd, Cmd, NULL) < 0)
	{
		perror("led_control ioctl led");
		close(led_fd);
		led_fd = -1;
		return -1;		
	}

	//close(led_fd);
	
	return 0;
}

int get_gpio22_value()
{
	int arg;
	int led_fd = -1;
	
	if(led_fd < 0)
	{
		led_fd = open(LED_DRIVER_NAME,O_RDWR);
		if(led_fd < 0)
		{
			perror("get_gpio22_value open led_driver");
			return -1;
		}
	}
	
	if(ioctl(led_fd, GPIO22_GET_IODATA, &arg) < 0)
	{
		perror("get_gpio22_value ioctl led");
		close(led_fd);
		led_fd = -1;
		return -1;		
	}

	close(led_fd);

	return arg;
	
}
