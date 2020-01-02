include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=satfi
PKG_RELEASE:=1

PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define Package/satfi
	SECTION:=utils
	CATEGORY:=Utilities
	TITLE:=Cjb -- Inmabeidou Chuanjiabao Service
	DEPENDS:=libpthread
#	DEPENDS+=libasound
endef

define Package/satfi/description
	if you can't figure out this program does,you're probably
	brain-dead and need immediate medical attention.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Package/satfi/install
	$(INSTALL_DIR) $(1)/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/satfi $(1)/bin/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/httpdown $(1)/bin/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/power_mode $(1)/bin/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/3g_manage $(1)/bin/
#	$(INSTALL_BIN) $(PKG_BUILD_DIR)/getip $(1)/bin/
#	$(INSTALL_BIN) $(PKG_BUILD_DIR)/checkgprs.sh $(1)/bin/
#	$(INSTALL_BIN) $(PKG_BUILD_DIR)/clientapp $(1)/bin/

#	$(INSTALL_DIR) $(1)/usr/lib
#	$(CP) $(STAGING_DIR)/usr/lib/libasound.so.2 $(1)/usr/lib

	$(INSTALL_DIR) $(1)/etc
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/config.ini $(1)/etc/
#	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/sysctl.conf $(1)/etc/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/rc.local $(1)/etc/
#	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/hosts $(1)/etc/
	
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/config/network $(1)/etc/config/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/config/firewall $(1)/etc/config/
#	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/config/system $(1)/etc/config/
#	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/config/dhcp $(1)/etc/config/
#	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/config/wireless $(1)/etc/config/
	
	$(INSTALL_DIR) $(1)/etc/chatscripts
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/chatscripts/3g.chat $(1)/etc/chatscripts/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/chatscripts/sat-sm2700.chat $(1)/etc/chatscripts/
	
	$(INSTALL_DIR) $(1)/lib/netifd/proto
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/lib/netifd/proto/3g.sh $(1)/lib/netifd/proto/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/lib/netifd/proto/ppp.sh $(1)/lib/netifd/proto/
		
	$(INSTALL_DIR) $(1)/etc/hotplug.d/usb
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/hotplug.d/usb/30-gprs $(1)/etc/hotplug.d/usb/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/hotplug.d/usb/40-gprs $(1)/etc/hotplug.d/usb/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/hotplug.d/usb/50-gprs $(1)/etc/hotplug.d/usb/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/hotplug.d/usb/60-gprs $(1)/etc/hotplug.d/usb/
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/hotplug.d/usb/40-sat-sm2700 $(1)/etc/hotplug.d/usb/
#	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/hotplug.d/usb/70-usbstm32 $(1)/etc/hotplug.d/usb/

#	$(INSTALL_DIR) $(1)/etc/syslog-ng
#	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/syslog-ng/syslog-ng.conf $(1)/etc/syslog-ng/
	
	$(INSTALL_DIR) $(1)/etc/rc.d
	$(CP) $(PKG_BUILD_DIR)/etc/rc.d/S01satfi $(1)/etc/rc.d/
	
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/etc/init.d/satfi $(1)/etc/init.d/
	
#	$(INSTALL_DIR) $(1)/lib/modules/
#	$(INSTALL_BIN) $(KERNEL_BUILD_DIR)/leds_driver/leds_driver.ko $(1)/$(TARGET_MODULES_DIR)/
#	$(INSTALL_BIN) $(KERNEL_BUILD_DIR)/leds_driver/leds_driver.ko $(1)/lib/modules/
endef

$(eval $(call BuildPackage,satfi))
