#KERN_DIR = /home/weiqing/7620/openwrt/openwrt-sdk-ramips/build_dir/target-mipsel_24kec+dsp_uClibc-0.9.33.2/linux-ramips_mt7620/linux-3.10.14-p112871

obj-m +=ralink_pcm.o
ralink_pcm-objs := 	pcm_ctrl.o

EXTRA_CFLAGS += -DCONFIG_RALINK_PCM_SPICH=1
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMRST_GPIO=0
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMCHNUM=1
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMINTDIV=117
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMCOMPDIV=48
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMSLOTMODE=0
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMUARTF
