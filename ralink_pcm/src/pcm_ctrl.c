#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/config.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/rt2880/surfboardint.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include "pcm_ctrl.h"
#include "ralink_gpio.h"
#include "ralink_gdma.h"

static DECLARE_WAIT_QUEUE_HEAD(dma_waitq_r);
static volatile int ev_dma_r = 1;	//record

static DECLARE_WAIT_QUEUE_HEAD(dma_waitq_t);
static volatile int ev_dma_t = 1;	//playback

#define PLAYBACK_PERIOD_SIZE	3200
#define PLAYBACK_BUF_SIZE 		(8*PLAYBACK_PERIOD_SIZE)
char *playbackbuf=NULL;			//playback 环形缓冲区
int playbackr=0;				//playback	读位置
int playbackw=0;				//playback	写位置
int isplaybackStart=0;

#define RECORD_PERIOD_SIZE		4800
#define RECORD_BUF_SIZE 		(8*RECORD_PERIOD_SIZE)
char *recordbuf=NULL;			//record 环形缓冲区
int recordr=0;					//record	读位置
int recordw=0;					//record	写位置
int isrecordStart=0;

static int is_playbackbuf_full(void)
{
	return ((playbackw + PLAYBACK_PERIOD_SIZE)% PLAYBACK_BUF_SIZE == playbackr);
}

static int is_playbackbuf_empty(void)
{
	return (playbackr == playbackw);
}

static void setplayback(void)
{
	playbackr = playbackw = isplaybackStart = 0;
}

void stopplayback(void)
{
	printk("stopplayback playbackr=%d playbackw=%d isplaybackStart=%d\n", playbackr, playbackw, isplaybackStart);
	msleep(3000);//等待播放完毕
	isplaybackStart = 0;
	if(!is_playbackbuf_empty())
	{
		ev_dma_t = 0;
		wait_event_interruptible(dma_waitq_t, ev_dma_t);//休眠
		printk("stoprecord ffinish\n");
	}
}

static int is_recordbuf_full(void)
{
	return ((recordw + RECORD_PERIOD_SIZE)% RECORD_BUF_SIZE == recordr);
}

static int is_recordbuf_empty(void)
{
	return (recordr == recordw);
}

static void setrecord(void)
{
	recordr = recordw = isrecordStart = 0;
}

static void stoprecord(void)
{
	printk("stoprecord recordr=%d recordw=%d isrecordStart=%d\n", recordr, recordw, isrecordStart);
	if(isrecordStart == 1)
	{
		isrecordStart = 0;
		ev_dma_r = 0;
		wait_event_interruptible(dma_waitq_r, ev_dma_r);//休眠
		printk("stoprecord ffinish\n");
	}
}

pcm_config_type* ppcm_config;

static int pcm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
struct file_operations pcmdrv_fops = {
	unlocked_ioctl:     pcm_ioctl,
};

int pcm_open(void)
{
	int i, data, flags;
	
	/* set pcm_config */
	ppcm_config = (pcm_config_type*)kmalloc(sizeof(pcm_config_type), GFP_KERNEL);
	if(ppcm_config==NULL)
		return PCM_OUTOFMEM;
	memset(ppcm_config, 0, sizeof(pcm_config_type));
	
	ppcm_config->pcm_ch_num = CONFIG_PCM_CH;
	ppcm_config->codec_ch_num = MAX_CODEC_CH;
	ppcm_config->nch_active = 0;
	ppcm_config->extclk_en = CONFIG_PCM_EXT_CLK_EN;
	ppcm_config->clkout_en = CONFIG_PCM_CLKOUT_EN;
	ppcm_config->ext_fsync = CONFIG_PCM_EXT_FSYNC;
	ppcm_config->long_fynsc = CONFIG_PCM_LONG_FSYNC;
	ppcm_config->fsync_pol = CONFIG_PCM_FSYNC_POL;
	ppcm_config->drx_tri = CONFIG_PCM_DRX_TRI;
	ppcm_config->slot_mode = CONFIG_RALINK_PCMSLOTMODE;//CONFIG_RALINK_PCMSLOTMODE;
	
	ppcm_config->tff_thres = CONFIG_PCM_TFF_THRES;
	ppcm_config->rff_thres = CONFIG_PCM_RFF_THRES;
		
	for ( i = 0 ; i < ppcm_config->pcm_ch_num; i ++ )
	{
		ppcm_config->lbk[i] = CONFIG_PCM_LBK;
		ppcm_config->ext_lbk[i] = CONFIG_PCM_EXT_LBK;
		ppcm_config->cmp_mode[i] = CONFIG_PCM_CMP_MODE;
#if defined(PCM_LINEAR) || defined(PCM_U2L2U) || defined(PCM_A2L2A)		
		ppcm_config->ts_start[i] = CONFIG_PCM_TS_START + i*16;	
#else
        ppcm_config->ts_start[i] = CONFIG_PCM_TS_START + i*8;
#endif		
		ppcm_config->txfifo_rd_idx[i] = 0;
		ppcm_config->txfifo_wt_idx[i] = 0;
		ppcm_config->rxfifo_rd_idx[i] = 0;
		ppcm_config->rxfifo_wt_idx[i] = 0;
		ppcm_config->bsfifo_rd_idx[i] = 0;
		ppcm_config->bsfifo_wt_idx[i] = 0;

	}	
PCM_RESET:	
	
	data = pcm_inw(RALINK_SYSCTL_BASE+0x34);
	data |= 0x00000800;
	pcm_outw(RALINK_SYSCTL_BASE+0x34,data);
	data = pcm_inw(RALINK_SYSCTL_BASE+0x34);
	data &= 0xFFFFF7FF;
	pcm_outw(RALINK_SYSCTL_BASE+0x34,data);
	for(i=0;i<100000;i++);
	
	data = pcm_inw(RALINK_SYSCTL_BASE+0x34);
	data |= 0x00040000;
	pcm_outw(RALINK_SYSCTL_BASE+0x34, data);
	data = pcm_inw(RALINK_SYSCTL_BASE+0x34);
	data &= 0xFFFBFFFF;
	pcm_outw(RALINK_SYSCTL_BASE+0x34,data);

	data = pcm_inw(RALINK_REG_GPIOMODE);
	/* Set UARTF_SHARE_MODE field */	
	data &= 0xFFFFFFE1;
	data |= 0x00000004;
	pcm_outw(RALINK_REG_GPIOMODE, data);
	MSG("RALINK_REG_GPIOMODE=0x%08X\n",data);

	if(pcm_reg_setup(ppcm_config)!=PCM_OK)
		MSG("PCM:pcm_reg_setup() failed\n");
	
	pcm_clock_setup();
	
	spin_lock_irqsave(&ppcm_config->lock, flags);

	/* Set to SP1_CS1_MODE mode and SPI_GPIO_MODE to spi mode */
	data = pcm_inw(RALINK_REG_GPIOMODE);
	data &= ~(1<<12);	
	pcm_outw(RALINK_REG_GPIOMODE, data);
    data = pcm_inw(PCM_GLBCFG);
    data |= REGBIT(0x1, PCM_EN);
    pcm_outw(PCM_GLBCFG, data);

	pcm_clock_enable();	
	spin_unlock_irqrestore(&ppcm_config->lock, flags);

	MSG("pcm_open done...\n");
	return PCM_OK;
	
PCM_OPEN_FAIL:
	MSG("pcm_open failed...\n");
	//pcm_close();
	return -1;

}

int pcm_reg_setup(pcm_config_type* ptrpcm_config)
{
	unsigned int data = 0;
	int i;	
	/* set GLBCFG's threshold fields */

	data = pcm_inw(PCM_GLBCFG);
#if defined (CONFIG_RALINK_MT7620) || defined (CONFIG_RALINK_MT7621) || defined (CONFIG_RALINK_MT7628)
	data |= REGBIT(CONFIG_PCM_LBK, PCM_LBK);
#else
#endif	
	data &= ~REGBIT(0, 29);
	data |= REGBIT(ptrpcm_config->tff_thres, TFF_THRES);
	data |= REGBIT(ptrpcm_config->rff_thres, RFF_THRES);
	MSG("PCM_GLBCFG=0x%08X\n",data);
	pcm_outw(PCM_GLBCFG, data);
	
	/* set PCMCFG */
	data = pcm_inw(PCM_PCMCFG);
	data |= REGBIT(ptrpcm_config->ext_fsync, PCM_EXT_FSYNC);
	data |= REGBIT(ptrpcm_config->long_fynsc, PCM_LONG_FSYNC);
	data |= REGBIT(ptrpcm_config->fsync_pol, PCM_FSYNC_POL);
	data |= REGBIT(ptrpcm_config->drx_tri, PCM_DRX_TRI);
	data &=  ~REGBIT(0x7, PCM_SLOTMODE);
	data |= REGBIT(ptrpcm_config->slot_mode, PCM_SLOTMODE);
	//data |= REGBIT(ptrpcm_config->clkout_en, PCM_CLKOUT);
	MSG("PCM_PCMCFG=0x%08X\n",data);
	pcm_outw(PCM_PCMCFG, data);
#if defined (CONFIG_RALINK_MT7620) || defined (CONFIG_RALINK_MT7621) || defined (CONFIG_RALINK_MT7628)
	for (i = 0; i < CONFIG_PCM_CH; i++)
	{
		data = pcm_inw(PCM_CH_CFG(i));
		data &= ~REGBIT(0x7, PCM_CMP_MODE);
		data &= ~REGBIT(0x3FF, PCM_TS_START);
		data |= REGBIT(ptrpcm_config->cmp_mode[i], PCM_CMP_MODE);
		data |= REGBIT(ptrpcm_config->ts_start[i], PCM_TS_START);
		MSG("PCM_CH_CFG(%d)=0x%08X\n",i,data);
		pcm_outw(PCM_CH_CFG(i), data);
	}  
#else	
	/* set CH0/1_CFG */	
	data = pcm_inw(PCM_CH0_CFG);
	data |= REGBIT(ptrpcm_config->lbk[0], PCM_LBK);
	data |= REGBIT(ptrpcm_config->ext_lbk[0], PCM_EXT_LBK);
	data |= REGBIT(ptrpcm_config->cmp_mode[0], PCM_CMP_MODE);
	data |= REGBIT(ptrpcm_config->ts_start[0], PCM_TS_START);
	MSG("PCM_CH0_CFG=0x%08X\n",data);
	pcm_outw(PCM_CH0_CFG, data);

	data = pcm_inw(PCM_CH1_CFG);
	data |= REGBIT(ptrpcm_config->lbk[1], PCM_LBK);
	data |= REGBIT(ptrpcm_config->ext_lbk[1], PCM_EXT_LBK);
	data |= REGBIT(ptrpcm_config->cmp_mode[1], PCM_CMP_MODE);
	data |= REGBIT(ptrpcm_config->ts_start[1], PCM_TS_START);
	MSG("PCM_CH1_CFG=0x%08X\n",data);
	pcm_outw(PCM_CH1_CFG, data);
#endif

#if defined(CONFIG_RALINK_RT3352) || defined(CONFIG_RALINK_RT5350) || defined (CONFIG_RALINK_RT6855A) \
	|| defined(CONFIG_RALINK_MT7620) || defined(CONFIG_RALINK_MT7621) || defined (CONFIG_RALINK_MT7628)
	data = pcm_inw(PCM_DIGDELAY_CFG);
	data = 0x00008484;
	MSG("PCM_DIGDELAY_CFG=0x%08X\n",data);
	pcm_outw(PCM_DIGDELAY_CFG, data);
#endif
	
	return PCM_OK;
}

int pcm_clock_setup(void)
{
	unsigned long data;

#if defined(CONFIG_RALINK_RT3352)||defined(CONFIG_RALINK_RT3883)||defined(CONFIG_RALINK_RT5350) \
	|| defined (CONFIG_RALINK_RT6855A) || defined(CONFIG_RALINK_MT7620) || defined(CONFIG_RALINK_MT7621) \
	|| defined (CONFIG_RALINK_MT7628)
	pcm_outw(RALINK_PCM_BASE+0x38, 1);
	data = pcm_inw(RALINK_PCM_BASE+0x38);
	MSG("PCM: enable fractinal PCM_CLK\n");
	pcm_outw(PCM_DIVINT_CFG, CONFIG_RALINK_PCMINTDIV);
	pcm_outw(PCM_DIVCOMP_CFG, CONFIG_RALINK_PCMCOMPDIV|0x80000000);
#else	
	/* System controller PCMCLK_DIV set */
	data = pcm_inw(RALINK_SYSCTL_BASE+0x30);

#if defined(CONFIG_RALINK_PCMEXTCLK)
	data |= REGBIT(1, PCM_CLK_SEL);
#else
	data &= ~REGBIT(1, PCM_CLK_SEL);
#endif	
	data |= REGBIT(1, PCM_CLK_EN);	
	data &= 0xFFFFFFC0;
	data |= REGBIT(CONFIG_RALINK_PCMDIV, PCM_CLK_DIV);
	data |= 0x00000080;
	
	pcm_outw(RALINK_SYSCTL_BASE+0x30, data);
	MSG("RALINK_SYSCTL_BASE+0x30=0x%08X\n",(u32)data);	
#endif	
	
	/* set PCMCFG external PCMCLK control bit */
	data = pcm_inw(PCM_PCMCFG);
#if defined(CONFIG_RALINK_PCMEXTCLK)
	data |= REGBIT(1, PCM_EXT_CLK_EN);
#else
	data &= ~REGBIT(1, PCM_EXT_CLK_EN);
#endif	
	pcm_outw(PCM_PCMCFG, data);
	MSG("PCM_PCMCFG=0x%08X\n",(u32)data);	
	
	return 0;	
}

int pcm_clock_enable(void)
{
	unsigned long data;
	/* set PCMCFG clock out bit */
	data = pcm_inw(PCM_PCMCFG);	
	data |= REGBIT(1,  PCM_CLKOUT);
	pcm_outw(PCM_PCMCFG, data);
	MSG("PCM_PCMCFG=0x%08X\n",(u32)data);
	
	return 0;	
}

int pcm_clock_disable(void)
{
	unsigned long data;
	/* set PCMCFG clock out bit */
	data = pcm_inw(PCM_PCMCFG);	
	data &= ~REGBIT(1,  PCM_CLKOUT);
	pcm_outw(PCM_PCMCFG, data);
	MSG("PCM_PCMCFG=0x%08X\n",(u32)data);
	
	return 0;	
}
	
int pcm_close(void)
{
	int i;
		
	MSG("pcm_close\n");	

	for( i = 0 ; i < ppcm_config->pcm_ch_num ; i ++ )
		pcm_disable(i, ppcm_config);	

	kfree(ppcm_config);
	ppcm_config = NULL;
	
	return PCM_OK;
}

int pcm_enable(unsigned int chid, pcm_config_type* ptrpcm_config)
{
	unsigned int GLBCFG_Data=0, int_en;
	
	if(ptrpcm_config->nch_active>=ptrpcm_config->pcm_ch_num)
	{
		MSG("There are %d channels already enabled\n",ptrpcm_config->nch_active);
		return PCM_OK;
	}
	int_en = pcm_inw(PCM_INT_EN);
	GLBCFG_Data = pcm_inw(PCM_GLBCFG);

	pcm_outw(PCM_INT_STATUS, 0x0);
	
	if(ptrpcm_config->nch_active==0)
		GLBCFG_Data |= REGBIT(0x1, DMA_EN);
#if defined (CONFIG_RALINK_MT7620) || defined(CONFIG_RALINK_MT7621) || defined (CONFIG_RALINK_MT7628)
	ptrpcm_config->nch_active++;
	GLBCFG_Data |= REGBIT(0x1, CH_EN+chid);
#else
	switch(chid)
	{
		case 0:
			MSG("PCM:enable CH0\n");
			GLBCFG_Data |= REGBIT(0x1, CH0_TX_EN);
			GLBCFG_Data |= REGBIT(0x1, CH0_RX_EN);
			
			int_en |= REGBIT(0x1, CH0T_DMA_FAULT);
			int_en |= REGBIT(0x1, CH0R_DMA_FAULT);
			 
			ptrpcm_config->nch_active++;
			break;
		case 1:
			MSG("PCM:enable CH1\n");
			GLBCFG_Data |= REGBIT(0x1, CH1_TX_EN);
			GLBCFG_Data |= REGBIT(0x1, CH1_RX_EN);
			
			int_en |= REGBIT(0x1, CH1T_DMA_FAULT);
			int_en |= REGBIT(0x1, CH1R_DMA_FAULT);
 
			ptrpcm_config->nch_active++;
			break;
		default:
			break;
	}
#endif	

	//GLBCFG_Data |= REGBIT(0x1, PCM_EN);
	pcm_outw(PCM_INT_EN, int_en);
	pcm_outw(PCM_GLBCFG, GLBCFG_Data);
	
	return PCM_OK;
}

int pcm_disable(unsigned int chid, pcm_config_type* ptrpcm_config)
{
	unsigned int data, int_en;

	if(ptrpcm_config->nch_active<=0)
	{ 
		MSG("No channels needed to disable\n");
		return PCM_OK;
	}
	ppcm_config->txfifo_rd_idx[chid] = 0;
	ppcm_config->txfifo_wt_idx[chid] = 0;
	ppcm_config->rxfifo_rd_idx[chid] = 0;
	ppcm_config->rxfifo_wt_idx[chid] = 0;
	ppcm_config->bsfifo_rd_idx[chid] = 0;
	ppcm_config->bsfifo_wt_idx[chid] = 0;
	
	int_en = pcm_inw(PCM_INT_EN);
	data = pcm_inw(PCM_GLBCFG);
	
#if defined (CONFIG_RALINK_MT7620) || defined(CONFIG_RALINK_MT7621) || defined (CONFIG_RALINK_MT7628)
	data &= ~REGBIT(0x1, CH_EN+chid);
	ptrpcm_config->nch_active--;
#else
	switch(chid)
	{
		case 0:
			MSG("PCM:disable CH0\n");
			data &= ~REGBIT(0x1, CH0_TX_EN);
			data &= ~REGBIT(0x1, CH0_RX_EN);
			int_en &= ~REGBIT(0x1, CH0T_DMA_FAULT);
			int_en &= ~REGBIT(0x1, CH0R_DMA_FAULT);
			pcm_outw(PCM_INT_EN, int_en);
			ptrpcm_config->nch_active--;

			break;
		case 1:
			MSG("PCM:disable CH1\n");
			data &= ~REGBIT(0x1, CH1_TX_EN);
			data &= ~REGBIT(0x1, CH1_RX_EN);
			int_en &= ~REGBIT(0x1, CH1T_DMA_FAULT);
			int_en &= ~REGBIT(0x1, CH1R_DMA_FAULT);
			pcm_outw(PCM_INT_EN, int_en);
			ptrpcm_config->nch_active--;

			break;
		default:
			break;
	}
#endif	
	if(ptrpcm_config->nch_active<=0)
	{
		//data &= ~REGBIT(0x1, PCM_EN);
		data &= ~REGBIT(0x1, DMA_EN);
	}
	pcm_outw(PCM_GLBCFG, data);
	return PCM_OK;
}

void dma_finish(u32 dma_ch)
{
	//printk("dma_finish %d %d %d %d\n", playbackr, playbackw, dma_ch, ev_dma_t);
}

void pcm_dma_tx_finish(u32 dma_ch)
{
	playbackr = (playbackr + PLAYBACK_PERIOD_SIZE) % PLAYBACK_BUF_SIZE;
	//printk("pcm_dma_tx_finish %d %d %d %d\n", playbackr, playbackw, dma_ch, ev_dma_t);

	if(isplaybackStart == 1)
	{
		if(!is_playbackbuf_empty())
		{
			//printk("GdmaPcmTx %d %d %d %d\n", playbackr, playbackw, dma_ch, ev_dma_t);
			GdmaPcmTx((u32)(playbackbuf+playbackr), (u32)PCM_CH_FIFO(0), 0, 0, PLAYBACK_PERIOD_SIZE, pcm_dma_tx_finish, dma_finish);
			GdmaUnMaskChannel(GDMA_PCM_TX(0,0));
		}
		else
		{
			//printk("playbackbuf_empty %d %d %d %d\n", playbackr, playbackw, dma_ch, ev_dma_t);
			isplaybackStart=0;
		}
	}

	if(ev_dma_t == 0)
	{
		//printk("pcm_dma_tx_finish wake_up_interruptible %d\n", isplaybackStart);
		ev_dma_t = 1;
		wake_up_interruptible(&dma_waitq_t);/* 唤醒休眠的进程 */
	}
}

void pcm_dma_rx_finish(u32 dma_ch)
{
	//printk("pcm_dma_rx_finish[%d]\n", dma_ch);//dma_ch=4	
	recordw = (recordw + RECORD_PERIOD_SIZE) % RECORD_BUF_SIZE;

	if(isrecordStart)
	{
		if(is_recordbuf_full())
		{
			//printk("recordbuf_full\n");
			recordr = (recordr + RECORD_PERIOD_SIZE) % RECORD_BUF_SIZE;
		}
		
		//printk("GdmaPcmRx %d %d %d %d\n", recordr, recordw, dma_ch, ev_dma_r);
		GdmaPcmRx((u32)PCM_CH_FIFO(0), (u32)recordbuf+recordw, 0, 0, RECORD_PERIOD_SIZE, pcm_dma_rx_finish, dma_finish);
		GdmaUnMaskChannel(GDMA_PCM_RX(0,0));
	}

	if(ev_dma_r == 0)
	{
		//printk("pcm_dma_rx_finish wake_up_interruptible\n");
		ev_dma_r = 1;
		wake_up_interruptible(&dma_waitq_r);/* 唤醒休眠的进程 */
	}
}

/**
 * @brief PCM interrupt handler 
 *
 * When PCM interrupt happened , call related handler 
 * to do the remain job.
 *
 */
irqreturn_t pcm_irq_isr(int irq, void *irqaction)
{
	u32 pcm_status;
	pcm_status=pcm_inw(PCM_INT_STATUS);
	MSG("SR=%08X\n",pcm_status);
	pcm_outw(PCM_INT_STATUS, 0xFFFF);
	return IRQ_HANDLED;
}

int pcm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int i, Ret;
	unsigned long flags;
	unsigned long data;
	
	pcm_config_type* ptrpcm_config = ppcm_config;	
	switch(cmd)
	{
		case PCM_OPEN:
			MSG("iocmd=PCM_OPEN\n");
			*(u32*)arg = 0;
			data = pcm_inw(RALINK_REG_GPIOMODE);
			data &= 0xFFFFFFE1;
			data |= 0x00000004;
			pcm_outw(RALINK_REG_GPIOMODE, data);
			break;
		case PCM_CLOSE:
			MSG("iocmd=PCM_CLOSE\n");
			break;
		case PCM_SET_RECORD:
			MSG("iocmd=PCM_SET_RECORD\n");				
			setrecord();
			break;
		case PCM_SET_UNRECORD:
			MSG("iocmd=PCM_SET_UNRECORD\n");
			break;
		case PCM_SET_PLAYBACK:
			MSG("iocmd=PCM_SET_RECORD\n");
			setplayback();
			break;
		case PCM_SET_UNPLAYBACK:
			MSG("iocmd=PCM_SET_UNPLAYBACK\n");
			break;
		case PCM_READ_PCM:
			if(is_recordbuf_empty())
			{
				if(isrecordStart == 0)
				{
					printk("start GdmaPcmRx %d %d\n", recordr, recordw);
					GdmaPcmRx((u32)PCM_CH_FIFO(0), (u32)recordbuf+recordw, 0, 0, RECORD_PERIOD_SIZE, pcm_dma_rx_finish, dma_finish);
					GdmaUnMaskChannel(GDMA_PCM_RX(0,0));
					isrecordStart = 1;
				}
			
				ev_dma_r = 0;
				wait_event_interruptible(dma_waitq_r, ev_dma_r);//休眠		
			}
			copy_to_user(arg, recordbuf+recordr, RECORD_PERIOD_SIZE);
			recordr = (recordr + RECORD_PERIOD_SIZE) % RECORD_BUF_SIZE;
			break;
		case PCM_WRITE_PCM:
			if(is_playbackbuf_full())
			{
				ev_dma_t = 0;
				wait_event_interruptible(dma_waitq_t, ev_dma_t);//休眠		
			}
			
			copy_from_user(playbackbuf+playbackw, arg, PLAYBACK_PERIOD_SIZE);
			playbackw = (playbackw + PLAYBACK_PERIOD_SIZE) % PLAYBACK_BUF_SIZE;
			
			if(isplaybackStart == 0)
			{
				printk("start GdmaPcmTx %d %d\n", playbackr, playbackw);
				GdmaPcmTx((u32)(playbackbuf+playbackr), (u32)PCM_CH_FIFO(0), 0, 0, PLAYBACK_PERIOD_SIZE, pcm_dma_tx_finish, dma_finish);
				GdmaUnMaskChannel(GDMA_PCM_TX(0,0));
				isplaybackStart = 1;
			}
			break;				
		case PCM_START:
			MSG("iocmd=PCM_START\n");
			Ret = request_irq(SURFBOARDINT_PCM, pcm_irq_isr, IRQF_DISABLED, "Ralink_PCM", NULL);
			if(Ret){
				MSG("PCM: IRQ %d is not free.\n", SURFBOARDINT_PCM);
				return PCM_REQUEST_IRQ_FAILED;
			}
			
			//使能pcm中断，时钟
			for ( i = 0 ; i < ptrpcm_config->pcm_ch_num ; i ++ )
				pcm_enable(i, ptrpcm_config);
			/* enable system interrupt for PCM */
			data = pcm_inw(RALINK_REG_INTENA);
			data |=0x010;
    		pcm_outw(RALINK_REG_INTENA, data);
			pcm_dump_reg();			
			break;
		case PCM_STOP:
			stopplayback();
			stoprecord();
			spin_lock_irqsave(&ptrpcm_config->lock, flags);
			/* disable system interrupt for PCM */
			data = pcm_inw(RALINK_REG_INTENA);
			data &=~0x010;
    		pcm_outw(RALINK_REG_INTENA, data);
			synchronize_irq(SURFBOARDINT_PCM);
			free_irq(SURFBOARDINT_PCM, NULL);
			for ( i = 0 ; i < ptrpcm_config->pcm_ch_num ; i ++ )
				pcm_disable(i, ptrpcm_config);			
			spin_unlock_irqrestore(&ptrpcm_config->lock, flags);
			break;
		case PCM_EXT_LOOPBACK_ON:
			MSG("external loopback on\n");
			data = pcm_inw(PCM_CH0_CFG);
			data |= 0x40000000;
			pcm_outw(PCM_CH0_CFG, data);
			for ( i = 0 ; i < ptrpcm_config->pcm_ch_num ; i ++ )
				pcm_enable(i, ptrpcm_config);
			break;
		case PCM_EXT_LOOPBACK_OFF:
			MSG("external loopback off\n");
			for ( i = 0 ; i < ptrpcm_config->pcm_ch_num ; i ++ )
				pcm_disable(i, ptrpcm_config);
			data = pcm_inw(PCM_CH0_CFG);
			data &= ~0x40000000;
			pcm_outw(PCM_CH0_CFG, data);
			break;
		default:
			break;
	}
	
	return 0;
}

void pcm_dump_reg (void)
{
	int i;
	MSG("[0x%08X]RALINK_REG_GPIOMODE=0x%08X\n", RALINK_REG_GPIOMODE, pcm_inw(RALINK_REG_GPIOMODE));
	MSG("[0x%08X]PCM_GLBCFG=0x%08X\n", PCM_GLBCFG, pcm_inw(PCM_GLBCFG));
	MSG("[0x%08X]PCM_PCMCFG=0x%08X\n", PCM_PCMCFG, pcm_inw(PCM_PCMCFG));
	MSG("[0x%08X]PCM_INT_STATUS=0x%08X\n", PCM_INT_STATUS, pcm_inw(PCM_INT_STATUS));
	MSG("[0x%08X]PCM_INT_EN=0x%08X\n", PCM_INT_EN, pcm_inw(PCM_INT_EN));
	MSG("[0x%08X]PCM_FF_STATUS=0x%08X\n", PCM_FF_STATUS, pcm_inw(PCM_FF_STATUS));
#if defined (CONFIG_RALINK_MT7620) || defined (CONFIG_RALINK_MT7621) || defined (CONFIG_RALINK_MT7628)
	for (i = 0; i < CONFIG_PCM_CH; i++) {
		MSG("[0x%08X]PCM_CH_CFG(%d)=0x%08X\n", PCM_CH_CFG(i), i,  pcm_inw(PCM_CH_CFG(i)));
		MSG("[0x%08X]PCM_CH_FIFO(%d)=0x%08X\n", PCM_CH_FIFO(i), i, pcm_inw(PCM_CH_FIFO(i)));
	}
#else	
	MSG("[0x%08X]PCM_CH0_CFG=0x%08X\n", PCM_CH0_CFG, pcm_inw(PCM_CH0_CFG));
	MSG("[0x%08X]PCM_CH1_CFG=0x%08X\n", PCM_CH1_CFG, pcm_inw(PCM_CH1_CFG));
	MSG("[0x%08X]PCM_CH0_FIFO=0x%08X\n", PCM_CH0_FIFO,pcm_inw(PCM_CH0_FIFO));
	MSG("[0x%08X]PCM_CH1_FIFO=0x%08X\n", PCM_CH1_FIFO,pcm_inw(PCM_CH1_FIFO));
#endif
#if defined(CONFIG_RALINK_RT3883)||defined(CONFIG_RALINK_RT3352)||defined(CONFIG_RALINK_RT5350) \
	|| defined (CONFIG_RALINK_RT6855A) || defined(CONFIG_RALINK_MT7620) || defined (CONFIG_RALINK_MT7621) \
	|| defined (CONFIG_RALINK_MT7628)
	MSG("[0x%08X]PCM_FSYNC_CFG=0x%08X\n", PCM_FSYNC_CFG, pcm_inw(PCM_FSYNC_CFG));
	MSG("[0x%08X]PCM_CH_CFG2=0x%08X\n", PCM_CH_CFG2, pcm_inw(PCM_CH_CFG2));
	MSG("[0x%08X]PCM_DIVCOMP_CFG=0x%08X\n", PCM_DIVCOMP_CFG, pcm_inw(PCM_DIVCOMP_CFG));
	MSG("[0x%08X]PCM_DIVINT_CFG=0x%08X\n", PCM_DIVINT_CFG, pcm_inw(PCM_DIVINT_CFG));
#endif	
}	

static int pcmdrv_major =  233;
static struct class *pcmmodule_class;
int __init pcm_init(void)
{
	u32	data;

    int result=0;
    result = register_chrdev(pcmdrv_major, PCMDRV_DEVNAME, &pcmdrv_fops);
    if (result < 0) {
		printk(KERN_WARNING "pcm: can't get major %d\n",pcmdrv_major);
        return result;
    }

    if (pcmdrv_major == 0) {
		pcmdrv_major = result; /* dynamic */
    }
	
	pcmmodule_class=class_create(THIS_MODULE, PCMDRV_DEVNAME);
	if (IS_ERR(pcmmodule_class)) 
		return -EFAULT;
	device_create(pcmmodule_class, NULL, MKDEV(pcmdrv_major, 0), NULL, PCMDRV_DEVNAME);

	MSG("PCMRST map to GPIO%d\n", CONFIG_RALINK_PCMRST_GPIO);
	MSG("Total %d PCM channel number supported\n", MAX_PCM_CH);
#if defined(CONFIG_RALINK_PCMEXTCLK) 	
	MSG("PCMCLK clock source from SoC external OSC\n");
#else
	MSG("PCMCLK clock source from SoC internal clock\n");	
#endif

#if defined(CONFIG_RALINK_PCMFRACDIV)	
	MSG("PCMCLK clock dividor Int[%d], Comp[%d]\n", CONFIG_RALINK_PCMINTDIV, CONFIG_RALINK_PCMCOMPDIV);
#else
	MSG("PCMCLK clock dividor [%d]\n", CONFIG_RALINK_PCMDIV);	
#endif	
	MSG("PCM slot mode is %d\n", CONFIG_RALINK_PCMSLOTMODE);

	pcm_open();
	recordbuf = kmalloc(RECORD_BUF_SIZE, GFP_KERNEL);
	if(recordbuf == NULL)
	{
		MSG("recordbuf kmalloc error\n");
		return -1;
	}
	
	playbackbuf = kmalloc(PLAYBACK_BUF_SIZE, GFP_KERNEL);
	if(playbackbuf == NULL)
	{
		MSG("playbackbuf kmalloc error\n");
		return -1;
	}
	
	return 0;
}

void pcm_exit(void)
{
	kfree(recordbuf);
	kfree(playbackbuf);
	pcm_close();
	
    unregister_chrdev(pcmdrv_major, PCMDRV_DEVNAME);
	device_destroy(pcmmodule_class,MKDEV(pcmdrv_major, 0));
	class_destroy(pcmmodule_class); 
	return ;
}

module_init(pcm_init);
module_exit(pcm_exit);
MODULE_LICENSE("GPL");

