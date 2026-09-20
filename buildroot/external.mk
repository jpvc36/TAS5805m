TAS5805M_VERSION = main
TAS5805M_SITE = https://github.com/jpvc36/TAS5805m/raw/refs/heads/main
TAS5805M_SOURCE = tas5805m.c
TAS5805M_SITE_METHOD = wget
TAS5805M_DEPENDENCIES = alsa-utils

define TAS5805M_EXTRACT_CMDS
	cp $(TAS5805M_DL_DIR)/$(TAS5805M_SOURCE) $(@D)/
	echo "obj-m += tas5805m.o" > $(@D)/Makefile
endef

$(eval $(kernel-module))
