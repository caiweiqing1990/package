#
# Copyright (C) 2008-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=power_mode
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/power_mode
  SUBMENU:=Other modules
  # DEPENDS:=@!LINUX_3_3
  TITLE:=power_mode
  FILES:=$(PKG_BUILD_DIR)/power_mode.ko
  AUTOLOAD:=$(call AutoLoad,40,power_mode,1)
  KCONFIG:=
endef

define KernelPackage/power_mode/description
 This is a power_mode drivers
 endef

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)"

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,power_mode))
