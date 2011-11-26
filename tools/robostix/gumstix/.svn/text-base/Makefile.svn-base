#############################################################################
#
#	Makefile for building the robostix module
#
#############################################################################

ifeq ($(DEBUG),y)
	CFLAGS += -O -g		# -O is need to expand inlines
else
	CFLAGS += -O2
endif

#----------------------------------------------------------------------------
											
ifeq ($(MAKE_SVN_VERSION),y)

all : svn-version.h

include ../Rules.mk
include ../svn-version.mk

else

#----------------------------------------------------------------------------
											
ifneq ($(KERNELRELEASE),)
# call from kernel build system

obj-m	:= robostix_drv.o # robostix-spi.o

else

PWD       := $(shell pwd)

GUMSTIX_BUILDROOT	= $(PWD)/../../gumstix-buildroot
BUILD_ARM		= $(GUMSTIX_BUILDROOT)/build_arm_nofpu
KERNELDIR 		?= $(wildcard $(BUILD_ARM)/linux-*)
CROSS_COMPILE		= $(BUILD_ARM)/staging_dir/bin/arm-linux-

default: svn-version.h user-app kernel-module

svn-version.h:
	$(MAKE) MAKE_SVN_VERSION=y svn-version.h

.PHONY: user-app kernel-module

user-app: robostix sertest

TARGET_ARCH=-Os -march=armv5te -mtune=xscale -Wa,-mcpu=xscale
CC = $(CROSS_COMPILE)gcc

robostix: robostix.c

sertest: LDLIBS = -lpthread
sertest: sertest.c

kernel-module:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) modules

endif	# KERNELRELEASE

#----------------------------------------------------------------------------

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions robostix sertest svn-version.h

depend .depend dep:
	$(CC) $(CFLAGS) -M *.c > .depend

ifeq (.depend,$(wildcard .depend))
include .depend
endif

endif 	# MAKE_SVN_VERSION
