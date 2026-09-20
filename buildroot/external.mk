TAS5805M_VERSION = main
TAS5805M_SITE = https://githubusercontent.com
TAS5805M_SOURCE = tas5805m.c
TAS5805M_SITE_METHOD = wget

# Tell Buildroot not to expect a compressed archive
TAS5805M_EXTRA_DOWNLOAD_OPTS = 

TAS5805M_DEPENDENCIES = alsa-utils
TAS5805M_MODULE_MAKE_OPTS = CONFIG_SND_SOC_TAS5805M=m

define TAS5805M_EXTRACT_CMDS
	cp $(TAS5805M_DL_DIR)/$(TAS5805M_SOURCE) $(@D)/
	echo "obj-m += tas5805m.o" > $(@D)/Makefile
endef

$(eval $(kernel-module))

