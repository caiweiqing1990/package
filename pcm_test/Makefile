#
# Top level makefile for pcm_test application
#

include $(TOPDIR)/rules.mk

PKG_NAME:=pcm_test
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define Package/pcm_test
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

define Package/pcm_test/install
	$(INSTALL_DIR) $(1)/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/pcm_test $(1)/bin/
endef

$(eval $(call BuildPackage,pcm_test))
