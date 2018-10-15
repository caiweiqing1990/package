#
# Copyright (C) 2008-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=ralink_gdma
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/ralink_gdma
  SUBMENU:=Other modules
  # DEPENDS:=@!LINUX_3_3
  TITLE:=Motor driver
  FILES:=$(PKG_BUILD_DIR)/ralink_gdma.ko
  AUTOLOAD:=$(call AutoLoad,38,ralink_gdma,1)
  KCONFIG:=
endef

define KernelPackage/ralink_gdma/description
 This is a ralink_gdma drivers
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

$(eval $(call KernelPackage,ralink_gdma))
