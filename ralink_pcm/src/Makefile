#KERN_DIR = /home/weiqing/7620/openwrt/openwrt-sdk-ramips/build_dir/target-mipsel_24kec+dsp_uClibc-0.9.33.2/linux-ramips_mt7620/linux-3.10.14-p112871

PKG_DIR = /home/weiqing/7620/openwrt/openwrt-sdk-ramips/package/ralink_pcm/src
obj-m +=ralink_pcm.o
ralink_pcm-objs := 	pcm_ctrl.o \
					codec_lib/codec_api.o \
					codec_lib/lib_g711/g711.o  \
					proslic_api/slic_init.o	\
					proslic_api/src/si_voice_version.o \
					proslic_api/src/vdaa.o \
					proslic_api/src/proslic.o \
					proslic_api/src/si_voice.o \
					proslic_api/src/si3226x_intf.o \
					proslic_api/patch_files/si3226x_patch_C_TSS_ISO_2012DEC06.o \
					proslic_api/config_src/si3226x_PBB_VDC_4P5_16P0_constants.o \
					proslic_api/config_src/vdaa_constants.o \
					proslic_api/example_drivers/ralink/proslic_ppt_mb/si3226/proslic_spi_api.o \
					proslic_api/example_drivers/ralink/timer/proslic_timer_intf_linux.o

EXTRA_CFLAGS += -I$(PKG_DIR)/proslic_api/inc
EXTRA_CFLAGS += -I$(PKG_DIR)/proslic_api/config_inc
EXTRA_CFLAGS += -I$(PKG_DIR)/proslic_api/example_drivers/ralink/proslic_ppt_mb
EXTRA_CFLAGS += -I$(PKG_DIR)/proslic_api/example_drivers/ralink/timer
EXTRA_CFLAGS += -DSI3226X
EXTRA_CFLAGS += -DSLIC_SILICON
EXTRA_CFLAGS += -DCONFIG_RALINK_PCM_SPICH=1
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMRST_GPIO=0
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMCHNUM=1
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMINTDIV=117
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMCOMPDIV=48
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMSLOTMODE=3
EXTRA_CFLAGS += -DCONFIG_RALINK_PCMUARTF
#all:
#	make ARCH=mips CROSS_COMPILE=mipsel-openwrt-linux-uclibc- -C $(KERN_DIR) SUBDIRS=$(shell pwd) modules

#clean:
#	make ARCH=mips CROSS_COMPILE=mipsel-openwrt-linux-uclibc- -C $(KERN_DIR) SUBDIRS=$(shell pwd) modules clean


