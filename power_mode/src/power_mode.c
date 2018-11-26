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
#include <linux/slab.h>
#include <asm/rt2880/rt_mmap.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <asm/rt2880/surfboardint.h> 
#include "power_mode_switch.h"
#define DEVICE_NAME  "power_mode"
#define DEVIRE_MAJOR 0
#define RALINK_PHYSICAL_ADDR	0x1000000
#define RALINK_PRGIO_ADDR        RALINK_PIO_BASE // Programmable I/O

#define RALINK_REG_PIO7140INT    (RALINK_PRGIO_ADDR + 0x60)//中断地址 
#define RALINK_REG_PIO7140EDGE   (RALINK_PRGIO_ADDR + 0x64)//边沿触发方式地址 
#define RALINK_REG_PIO7140RENA  (RALINK_PRGIO_ADDR + 0x68)//上升沿触发掩码 
#define RALINK_REG_PIO7140FENA    (RALINK_PRGIO_ADDR + 0x6C)

#define  RALINK_IRQ_ADDR         RALINK_INTCL_BASE

#define  RALINK_REG_INTENA        (RALINK_IRQ_ADDR   + 0x34)//enable 中断地址 
#define  RALINK_REG_INTDIS        (RALINK_IRQ_ADDR   + 0x38)//disable 中断地址

volatile unsigned long *GPIOMODE;
volatile unsigned long *GPIO71_40_DIR;
volatile unsigned long *GPIO71_40_DATA;
volatile unsigned long *GPIO39_24_DIR;
volatile unsigned long *GPIO39_24_DATA;
volatile unsigned long *GPIO23_00_DIR;
volatile unsigned long *GPIO23_00_DATA;

static struct class *power_mode_class;
#if defined (CONFIG_RALINK_interrupt)
//保存当前中断方式 
static u32 ralink_gpio7140_intp = 0; 
//保存当前边沿触发方式 
static u32 ralink_gpio7140_edge = 0;

//gpio状态 
typedef enum 
{ 
    e_gpio_rising = 0, 
    e_gpio_falling, 
    e_gpio_edge_unknow 
}e_gpio_edge_t;

//gpio对应信息 
struct gpio_status 
{ 
    int gpio_num; 
    e_gpio_edge_t edge; 
    u32 key_value;//键值传给用户空间 
};
 //生成一个等待队列头wait_queue_head_t,名字为key_waitq 
/* 等待队列: 
* 当没有按键被按下时，如果有进程调用key_driver_read函数， 
* 它将休眠 
*/
static DECLARE_WAIT_QUEUE_HEAD(thuraya_waitq);

/* 中断事件标志, 中断服务程序将它置1，key_driver_read将它清0*/ 
static volatile int ev_press = 0;

static struct gpio_status g_gpio67 = {55, e_gpio_edge_unknow, 0}; 

/*static void sec_delay(unsigned int sec)
{
   unsigned char flag=0;
   while(1) {
     if (flag){
	break;
     }
    else if (sec--){
     msleep(500);//反正一个较大的数字
    } else {
    flag=1;
    msleep(1);//一个较小的数字
    }
  }
}*/

//设置pio enable interrupt 
static void enable_intp(void) 
{    
    //在rt_mmap.h头文件中定义RALINK_INTCTL_PIO ，即第六位控制pio中断 
    //#define RALINK_INTCTL_PIO       (1<<6) 
    *(volatile u32 *)(RALINK_REG_INTENA) = cpu_to_le32(RALINK_INTCTL_PIO); 
}

//设置pio disable interrupt 
static void disable_intp(void) 
{ 
    //在rt_mmap.h头文件中定义RALINK_INTCTL_PIO ，即第六位控制pio中断 
    //#define RALINK_INTCTL_PIO       (1<<6) 
    *(volatile u32 *)(RALINK_REG_INTDIS) = cpu_to_le32(RALINK_INTCTL_PIO); 
}


//set Edge Interrupt 
static void gpio_reg_irq(int irq) 
{ 
    unsigned long tmp; 
    
    tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140RENA)); 
    tmp |= (0x1 << (irq-40)); 
    *(volatile u32 *)(RALINK_REG_PIO7140RENA) = cpu_to_le32(tmp); 
    /*tmp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140FENA)); 
    tmp |= (0x1 << (irq-40)); 
    *(volatile u32 *)(RALINK_REG_PIO7140FENA) = cpu_to_le32(tmp);*/ 
}

//先保存当前中断及触发寄存器的值,再清空 
static void ralink_gpio7140_save_clear_intp(void) 
{ 
    //保存当前中断寄存器数据 
    ralink_gpio7140_intp = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140INT)); 
    //保存当前边沿触发方式 
    ralink_gpio7140_edge = le32_to_cpu(*(volatile u32 *)(RALINK_REG_PIO7140EDGE)); 
    *(volatile u32 *)(RALINK_REG_PIO7140INT) = cpu_to_le32(0xFFFFFFFF); 
    *(volatile u32 *)(RALINK_REG_PIO7140EDGE) = cpu_to_le32(0xFFFFFFFF); 
}

//扫描时候是gpio67被按下 
static int scan_gpio_num(void) 
{ 
    if(!(ralink_gpio7140_intp & (1<<(g_gpio67.gpio_num - 40)))) 
    { 
        printk("Have no key pressed...\n"); 
        return -1; 
    } 
    if (ralink_gpio7140_edge & (1<<(g_gpio67.gpio_num - 40))) { 
        printk("set gpio value..\n"); 
            g_gpio67.edge = e_gpio_rising; 
            g_gpio67.key_value = 1; 
                //上升沿才有效，具体需要根据硬件设计。 
    } 
    else { 
        printk("Have no edag...\n"); 
        return -1; 
    } 
    return 0; 
}

static int thuraya_driver_read(struct file *filp, char __user *buff, size_t count, loff_t *offp) 
{ 
    unsigned long err;
/* 当没有按键按下时，休眠。
     * 即ev_press = 0;
     * 当有按键按下时，发生中断，在中断处理函数会唤醒
     * 即ev_press = 1; 
     * 唤醒后，接着继续将数据通过copy_to_user函数传递给应用程序
     */
    if(!ev_press) { 
        if(filp->f_flags & O_NONBLOCK) 
            return -EAGAIN; 
        else//由wake_up_interruptible等待中断 
            wait_event_interruptible(thuraya_waitq, ev_press); 
    } 
    ev_press = 0; 
    err = copy_to_user(buff, (const void *)&g_gpio67.key_value, sizeof(g_gpio67.key_value));

    g_gpio67.edge = e_gpio_edge_unknow; 
    g_gpio67.key_value = 0x0; 
    
    return err ? -EFAULT : sizeof(g_gpio67.key_value); 
}

static unsigned int thuraya_driver_poll( struct file *file, struct poll_table_struct *wait) 
{ 
    unsigned int mask = 0;

    /*把调用poll 或者select 的进程挂入队列，以便被驱动程序唤醒*/ 
    /*需要注意的是这个函数是不会引起阻塞的*/ 
    poll_wait(file, &thuraya_waitq, wait); 
      /* 当没有按键按下时，即不会进入按键中断处理函数，此时ev_press = 0 
     * 当按键按下时，就会进入按键中断处理函数，此时ev_press被设置为1
     */
  if(ev_press)  
    {  
        mask |= POLLIN | POLLRDNORM;  /* 表示有数据可读 */
    }  
/* 如果有按键按下时，mask |= POLLIN | POLLRDNORM,否则mask = 0 */
  return mask;  
}

//中断处理函数 
/*关于中断处理函数的返回值:中断程序的返回值是一个特殊类型—irqreturn_t。 
中断程序的返回值却只有两个: IRQ_NONE和IRQ_HANDLED。*/ 
static irqreturn_t thuraya_interrupt(int irq, void *irqaction) 
{ 
    int ret;

    printk("interrupt handler...\n"); 
    
    //先保存当前中断及触发寄存器的值,再清空 
    ralink_gpio7140_save_clear_intp();

    //查看是否是gpio67被按下 
    ret = scan_gpio_num();

    //printk("Int func,gpio num %d.jiffies %ld,HZ %d\n",num,jiffies,HZ); 
    if(ret < 0) 
        return IRQ_RETVAL(IRQ_NONE);

    //设置键值 
    
    
    ev_press = 1; /*设置中断标志为1*/ 
     wake_up_interruptible(&thuraya_waitq); /*唤醒等待队列*/

    printk("interrupt wake up...\n"); 
        
    return IRQ_RETVAL(IRQ_HANDLED); 
}

#endif

static void peripheral_init_io(void)
{
       PERIOHERAL_GPIO_OUT;
       VD5_ON;
       USB_HUP_5V_EN;
}

static void gps_mode_on(void)
{
	GPS_ONOFF_OUT; 
	GPS_ONOFF_H;
	msleep(1000);
	GPS_ONOFF_L;
}

static void gps_mode_off(void)
{
	GPS_ONOFF_OUT; 
	GPS_ONOFF_H;
}

static void gprs_init_io(void)
{
	M3732_GPIO_OUT;
	GPRS_POWER_H;
	M3732_ONOFF_L;
}

static void thuraya_init_io(void)
{ 
	SAT_GPIO_OUT;
	GPIO_OUT;
	SM2500_POWER_H;
	SM2500_ON_OFF_H;
	SAT_RESET_L;
	SAT_ON_L;
	SAT_OFF_L;
}

static void gprs_init_mode(void)
{
	GPRS_POWER_H;
	USB_HUP_5V_disab;
	msleep(5);
}

static void turn_on_mode(void)
{
	gprs_init_mode();
	GPRS_POWER_L;//V_MAIN 一上电， ON/OFF 信号就会同步建立为高电平
	msleep(500);
	USB_HUP_5V_EN;
	msleep(1000);
	M3732_ONOFF_H;
	msleep(3000);
	M3732_ONOFF_L;
}

static void turn_off_mode(void)
{
	M3732_ONOFF_H;
	msleep(3000);
	M3732_ONOFF_L;
}

static void SAT_Reset_mode(void)
{
	SM2500_POWER_L;//SM2500_V3.7上拉
	SAT_RESET_L;
	SAT_ON_L;
	msleep(1);
	SAT_ON_H;
	udelay(3);//T1
	SAT_RESET_H;
	udelay(15);
	SAT_RESET_L;
	msleep(650);
	udelay(3);//T2
	SAT_ON_L;
}

static void SAT_Turn_on(void)
{
/*把SAT_ON拉高至少一秒之后，再插上电源VPWR, 5V_TX，然后把电平拉低*/
	USB_HUP_5V_EN;
	SM2500_ON_OFF_L;
	SM2500_POWER_L;
	SAT_ON_H;
	msleep(2000);
	SAT_ON_L;
}

static void SAT_Turn_off(void)
{
	SM2500_POWER_L;//SM2500_V3.7上拉
	SAT_ON_L;
	SAT_OFF_H;
	msleep(2000);
	SAT_OFF_L;
}

void MSM01A_power_on(void)
{
	printk("MSM01A_power_on\n");
	RSTIN_N_OUT;
	POWER_KEY_OUT;
	POWER_KEY_L;
	RSTIN_N_L;
	msleep(500);
	RSTIN_N_H;
	msleep(500);
	POWER_KEY_H;
	msleep(500);
	POWER_KEY_L;
}

void MSM01A_wakeup(void)
{
	printk("MSM01A_wakeup\n");
	A2B_WAKEUP_OUT;
	A2B_WAKEUP_H;
	msleep(500);
	A2B_WAKEUP_L;
}

void MSM01A_reset(void)
{
	RSTIN_N_OUT;
	POWER_KEY_OUT;
	POWER_KEY_L;
	RSTIN_N_L;
	msleep(300);
	POWER_KEY_H;
	RSTIN_N_H;
	msleep(300);
}

void MSM01A_off(void)
{
	RSTIN_N_OUT;
	POWER_KEY_OUT;
	POWER_KEY_L;
	RSTIN_N_L;
	msleep(300);
}

void MSM01A_on(void)
{
	RSTIN_N_OUT;
	POWER_KEY_OUT;
	POWER_KEY_H;
	RSTIN_N_H;
	msleep(300);
}

static int power_mode_open(struct inode *inode, struct file *file)
{
    //VD5_ON; 
#if defined (CONFIG_RALINK_interrupt)
    enable_intp();        //set pio enable interrupt 
    gpio_reg_irq(55);     //set Edge Interrupt 
#endif    
    return 0;
}


static ssize_t power_mode_write(struct file * file, const char __user *buf, size_t size,
 loff_t *ppos)
{
    return 0;
}

//关闭设备 
static int thuraya_driver_close(struct inode *inode, struct file *file) 
{ 
#if defined (CONFIG_RALINK_interrupt)
    disable_intp(); 
    //禁止中断 
#endif
    return 0; 
}

static long power_mode_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch(cmd)
	{
		case gprs_on:
			turn_on_mode();//
		break;
		case gprs_off:
			turn_off_mode();
		break;
		case SAT_System_Reset:
			//SAT_Reset_mode();
			MSM01A_reset();
		break;
		case SAT_ON:
			MSM01A_on();//
		break;
		case SAT_OFF:
			MSM01A_off();
		break;
		case GPS_ON:
			gps_mode_on();//
		break;
		case GPS_OFF:
			gps_mode_off();
		break;
		case GE1_TXD2_ON:
			GE1_TXD2_H;
		break;
		case GE1_TXD2_OFF:
			GE1_TXD2_L;
		break;
		return -EINVAL;
	}
   return 0;
}
static struct file_operations power_mode_fops = {
    .owner = THIS_MODULE,/*这是一个宏，推向编译模块时自动创建的__this_module变量*/
    .open  = power_mode_open,
#ifdef IRQ_DEBUG
    .read  = thuraya_driver_read,
#endif
    .write = power_mode_write,
#ifdef IRQ_DEBUG
    .poll  = thuraya_driver_poll,
#endif
    .release    = thuraya_driver_close,
    .unlocked_ioctl = power_mode_unlocked_ioctl,
};

int major;
#define SURFBOARDINT_GPIO	 6	/* GPIO */
static irqreturn_t ralink_key_interrupt(int irq, void *irqaction)
{
	 printk("interrupt handler... %d\n", irq);
	 
	 return IRQ_RETVAL(IRQ_HANDLED);
}

static int __init power_mode_init(void)
{
#ifdef IRQ_DEBUG
    int ret; 
   /* 注册中断请求 
     中断号:SURFBOARDINT_GPIO 
     中断处理函数:ralink_key_interrupt 
     中断属性(方式):上升沿触发 
     使用此中断的设备:gpio_key 
     函数:*/
      request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags,                                                                                          
      const char *name, void *dev)
 {
    return request_threaded_irq(irq, handler, NULL, flags, name, dev);
 }
     
    ret = request_irq(SURFBOARDINT_GPIO, thuraya_interrupt, IRQF_TRIGGER_RISING, 
                            DEVICE_NAME, NULL); 
    if (ret<0) {
    	 printk("can't register irq %d\n",ret);
        return ret;
   	}
#endif
    /*注册字符设备
    **参数为主设备号、设备名字、file_operations结构
    **操作主设备为devire_MAJOR的设备文件时，就会调用devire_drv_fops中的相关成员函数*/
    major = register_chrdev(DEVIRE_MAJOR, DEVICE_NAME, &power_mode_fops);
    if(major < 0)/*注册失败*/
    {
        printk(DEVICE_NAME"can't register major number\n");
        return major; 
    }
	/*下面两行用于创建设备节点，
	如果不写这两行，必须在linux系统命令行中通过mknod创建设备节点
	*/
	/*创建类*/
	power_mode_class = class_create(THIS_MODULE, "power_mode");
	/*类下面创建设备节点*/
	device_create(power_mode_class, NULL, MKDEV(major, 0), NULL, "power_mode");

	GPIOMODE = (volatile unsigned long *)ioremap(0x10000060, 4);
	GPIO71_40_DIR = (volatile unsigned long *)ioremap(0x10000674, 4);
	GPIO71_40_DATA = (volatile unsigned long *)ioremap(0x10000670, 4); 
	GPIO39_24_DIR = (volatile unsigned long *)ioremap(0x1000064C, 4);
	GPIO39_24_DATA = (volatile unsigned long *)ioremap(0x10000648, 4);
	GPIO23_00_DIR = (volatile unsigned long *)ioremap(0x10000624, 4);
	GPIO23_00_DATA = (volatile unsigned long *)ioremap(0x10000620, 4);
	
	*GPIOMODE &= ~((0x3<<18)|(0X1<<9)|(0X1<<10)|(0X1<<15)|(0X3<<7));
	*GPIOMODE |= (0x2<<18)|(0X1<<9)|(0X1<<10)|(0X1<<15)|(0X2<<7);//NAD_SD | RGMII2 | RGMII1 | EPHY_LED_GPIO_MODE | MDIO
	peripheral_init_io();
	// gprs_init_io();
	// thuraya_init_io();
	M3732_GPIO_OUT; //SC2_4.2V_ON_OFF_OUT L
	GPRS_POWER_L;
	//msleep(2000);
	msleep(2000);
	GPS_ONOFF_OUT;	////SC2_UP_OUT 
	GPS_ONOFF_L;

	msleep(2000);
	TX_5V_ONOFF9602_OUT; //5V_SC2_USB_ON_OFF_OUT  L
	TX_5V_ONOFF9602_L;

	GE1_TXD2_OUT;//用于电池电量提示
	GE1_TXD2_H;
	ENABLE_GSTAR_OUT;  	//GE1_TXCLK GPIO#29 ENABLE_GSTAR_OUT
	ENABLE_GSTAR;	//ENABLE_GSTAR

	printk("GPIO39_24_DIR=%08x,GPIO39_24_DATA=%08x,GPIOMODE=%08x\n", *GPIO39_24_DIR, *GPIO39_24_DATA, *GPIOMODE);
	//MSM01A_power_on();
	//MSM01A_reset();
	return 0;
}

static void __exit power_mode_exit(void)
{
	USB_HUP_5V_disab;//USB HUB DISABLE
	DISABLE_GSTAR; //DISABLE_GSTAR
#ifdef IRQ_DEBUG
    free_irq(SURFBOARDINT_GPIO,NULL);//注销中断 
#endif
    unregister_chrdev(major, "power_mode");/*与register配对用*/
    device_destroy(power_mode_class, MKDEV(major, 0));/*与入口device_create*/
    class_destroy(power_mode_class);/*与入口calss_create*/
    iounmap(GPIOMODE);
    iounmap(GPIO71_40_DIR);
    iounmap(GPIO71_40_DATA);
    iounmap(GPIO39_24_DIR);
    iounmap(GPIO39_24_DATA);
    iounmap(GPIO23_00_DIR);
    iounmap(GPIO23_00_DATA);
}

module_init(power_mode_init);
module_exit(power_mode_exit);

MODULE_LICENSE("GPL");


