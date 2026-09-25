include $(TOPDIR)/rules.mk

PKG_NAME:=tas5805m
PKG_VERSION:=1.0
PKG_RELEASE:=1

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/jpvc36/TAS5805m.git
PKG_SOURCE_VERSION:=main
PKG_MIRROR_HASH:=skip

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define KernelPackage/tas5805m
  SUBMENU:=Sound Support
  TITLE:=TAS58xx driver
  DEPENDS:= kmod-i2c-bcm2835 kmod-regmap-i2c kmod-sound-soc-bcm2835-i2s kmod-sound-soc-core
  FILES:=$(PKG_BUILD_DIR)/tas5805m.ko \
        $(PKG_BUILD_DIR)/simple-card.ko \
        $(PKG_BUILD_DIR)/simple-card-utils.ko
  AUTOLOAD:=$(call AutoLoad,68,tas5805m simple-card simple-card-utils)
endef

EXTRA_KCONFIG:=

EXTRA_CFLAGS += \
    -DCONFIG_AP_MODE \
    -I$(PKG_BUILD_DIR)/include

define Build/Compile
        $(KERNEL_MAKE) \
                M="$(PKG_BUILD_DIR)" \
                EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
                modules
endef

$(eval $(call KernelPackage,tas5805m))
