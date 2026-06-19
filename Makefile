obj-m += tas5805m.o

KERNEL := /lib/modules/$(shell uname -r)/build
CROSS_COMPILE :=
TARGET_ARCH := $(shell uname -m)

ifeq ($(ARCH),arm64)
    CROSS_COMPILE := aarch64-linux-gnu-
    TARGET_ARCH := arm64
else ifeq ($(ARCH),aarch64)
    CROSS_COMPILE := aarch64-linux-gnu-
    TARGET_ARCH := arm64
else ifeq ($(ARCH),arm)
    CROSS_COMPILE := arm-linux-gnueabihf-
    TARGET_ARCH := arm
endif

all:
	make -j$(shell nproc) ARCH=$(TARGET_ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(shell pwd) modules
clean:
	make -j$(shell nproc) ARCH=$(TARGET_ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(shell pwd) clean
