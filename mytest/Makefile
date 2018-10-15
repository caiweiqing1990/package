#
# Top level makefile for mytest application
#

include $(TOPDIR)/rules.mk

PKG_NAME:=mytest
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define Package/mytest
 SECTION:=utils
 CATEGORY:=Utilities
 TITLE:=prints a message
 DEPENDS:=+libpthread
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)
endef

define Build/Configure
endef

TARGET_CFLAGS += $(FPIC)

define Package/mytest/install
	$(INSTALL_DIR) $(1)/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/mytest $(1)/bin/
#	$(INSTALL_BIN) $(PKG_BUILD_DIR)/msgtest $(1)/bin/
endef

$(eval $(call BuildPackage,mytest))
