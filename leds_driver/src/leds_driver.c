#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/random.h>
#include <linux/init.h>
#include <linux/raw.h>
#include <linux/tty.h>
#include <linux/capability.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/crash_dump.h>
#include <linux/backing-dev.h>
#include <linux/bootmem.h>
#include <linux/splice.h>
#include <linux/pfn.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/aio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h>

#define SM2700_LED_ON 		0
#define SM2700_LED_OFF 		1
#define GPS96_LED_ON 		2
#define GPS96_LED_OFF 		3
#define GPIO22_GET_IODATA 	4
#define GE1_TXD2_ON			8
#define GE1_TXD2_OFF		9

#define PTT1_ON				10
#define PTT1_OFF			11
#define PTT2_ON				12
#define PTT2_OFF			13

#define GE1_TXD2_OUT	*GPIO39_24_DIR |= (0x1<<2)
#define GE1_TXD2_H		*GPIO39_24_DATA |= (0x1<<2)
#define GE1_TXD2_L		*GPIO39_24_DATA &= ~(0x1<<2)

//ptt1
#define PTT1_OUT		*GPIO39_24_DIR |= (1<<5)
#define PTT1_H			*GPIO39_24_DATA |= (1<<5)
#define PTT1_L			*GPIO39_24_DATA &= ~(1<<5)

//ptt2
#define PTT2_OUT		*GPIO71_40_DIR |= (1<<24)
#define PTT2_H			*GPIO71_40_DATA |= (1<<24)
#define PTT2_L			*GPIO71_40_DATA &= ~(1<<24)

#define LED_DRIVER_NAME "leds_driver"

volatile unsigned long *GPIOMODE;
volatile unsigned long *GPIO71_40_DIR;
volatile unsigned long *GPIO71_40_DATA;

volatile unsigned long *GPIO39_24_DIR;
volatile unsigned long *GPIO39_24_DATA;

volatile unsigned long *GPIO23_00_DIR;
volatile unsigned long *GPIO23_00_DATA;

static struct class *leds_class;

static int leds_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int leds_close(struct inode *inode, struct file *file)
{
	return 0;
}

static long leds_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ioarg = 0;
	
	switch(cmd)
	{
		case SM2700_LED_ON:
			*GPIO71_40_DATA |= (1<<5);
			break;
		case SM2700_LED_OFF:
			*GPIO71_40_DATA &= ~(1<<5);
			break;
		case GPS96_LED_ON:	
			*GPIO71_40_DATA |= (1<<7);
			break;
		case GPS96_LED_OFF:
			*GPIO71_40_DATA &= ~(1<<7);
			break;
		case GPIO22_GET_IODATA:
			ioarg = (*GPIO71_40_DATA >> 31) & 0X1; 
			put_user(ioarg, (int *)arg);
			break;
		case GE1_TXD2_ON:
			GE1_TXD2_H;
			break;
		case GE1_TXD2_OFF:
			GE1_TXD2_L;
			break;
		case PTT1_ON:
			PTT1_H;
			break;
		case PTT1_OFF:
			PTT1_L;
			break;
		case PTT2_ON:
			PTT2_H;
			break;
		case PTT2_OFF:
			PTT2_L;
			break;
		default:
			break;
	}
	
	return 0;
}

/* 1.分配、设置一个file_operations结构体 */
static struct file_operations myleds_fops = {
	.owner   			= THIS_MODULE,   /* 这是一个宏，推向编译模块时自动创建的__this_module变量 */
	.open    			= leds_open,
	.release    		= leds_close, 
	.unlocked_ioctl		= leds_unlocked_ioctl,
};

int major;
static int __init myleds_init(void)
{
	/* 2.注册 */
	major = register_chrdev(0, LED_DRIVER_NAME, &myleds_fops);

	/* 3.自动创建设备节点 */
	/* 创建类 */
	leds_class = class_create(THIS_MODULE, LED_DRIVER_NAME);
	/* 类下面创建设备节点 */
	device_create(leds_class, NULL, MKDEV(major, 0), NULL, LED_DRIVER_NAME);

	/* 4.硬件相关的操作 */
	/* 映射寄存器的地址 */
	GPIOMODE = (volatile unsigned long *)ioremap(0x10000060, 4);
	GPIO71_40_DATA = (volatile unsigned long *)ioremap(0x10000670, 4);
	GPIO71_40_DIR = (volatile unsigned long *)ioremap(0x10000674, 4);
	GPIO39_24_DIR = (volatile unsigned long *)ioremap(0x1000064C, 4);
	GPIO39_24_DATA = (volatile unsigned long *)ioremap(0x10000648, 4);
	GPIO23_00_DIR = (volatile unsigned long *)ioremap(0x10000624, 4);
	GPIO23_00_DATA = (volatile unsigned long *)ioremap(0x10000620, 4);
	
	/* 设置相应管脚用于GPIO */
	//*GPIOMODE = (0x2<<18)|(0X1<<9)|(0X1<<10)|(0X1<<15);//NAD_SD  RGMII2  RGMII1|   EPHY_LED_GPIO_MODE
   *GPIOMODE = (0x2<<18)|(0X1<<9)|(0X1<<10)|(0X1<<15)|(0X2<<7);//NAD_SD | RGMII2 | RGMII1 | EPHY_LED_GPIO_MODE | MDIO
	
	GE1_TXD2_OUT;//用于电池电量提示
	GE1_TXD2_H;

	PTT1_OUT;
	PTT2_OUT;
	PTT1_L;
	PTT2_L;
	
	PTT1_H;
	PTT2_H;
	
	return 0; 
}

static void __exit myleds_exit(void)
{
	PTT1_L;
	PTT2_L;
	*GPIO71_40_DATA |= ((1<<5)|(1<<7)); //点亮LED
	//printk("GPIO71_40_DIR = %x\n",*GPIO71_40_DIR);
	//printk("GPIO71_40_DATA = %x\n",*GPIO71_40_DATA);
	unregister_chrdev(major, LED_DRIVER_NAME);
	device_destroy(leds_class, MKDEV(major, 0));
	class_destroy(leds_class);
	iounmap(GPIOMODE);
	iounmap(GPIO71_40_DIR);
	iounmap(GPIO71_40_DATA);
	iounmap(GPIO23_00_DIR);
	iounmap(GPIO23_00_DATA);
	iounmap(GPIO39_24_DIR);
	iounmap(GPIO39_24_DATA);
}

module_init(myleds_init);
module_exit(myleds_exit);

MODULE_LICENSE("GPL");


