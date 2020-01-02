#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define SM2700_LED_ON 		0
#define SM2700_LED_OFF 		1
#define GPS96_LED_ON 		2
#define GPS96_LED_OFF 		3
#define GPIO22_GET_IODATA 	4

#define LED_DRIVER_NAME "/dev/leds_driver"

int led_control(int Cmd);


