#############################################################################
#
#	Makefile for building the i2c-test program
#
#############################################################################

ifeq ($(DEBUG),y)
	CFLAGS += -O -g		# -O is need to expand inlines
else
	CFLAGS += -O2
endif

PWD       := $(shell pwd)

GUMSTIX_BUILDROOT	= $(PWD)/../../../gumstix-buildroot
BUILD_ARM			= $(GUMSTIX_BUILDROOT)/build_arm_nofpu
CROSS_COMPILE		= $(BUILD_ARM)/staging_dir/bin/arm-linux-
COMMON				= ../Common
SHARED				= ../../Shared

vpath %.c $(COMMON) $(SHARED)

CPPFLAGS += -I . -I $(COMMON) -I $(SHARED)
CFLAGS	 += -Wall
LDFLAGS  += -lpthread

TARGET_ARCH=-Os -march=armv5te -mtune=xscale -Wa,-mcpu=xscale
CC = $(CROSS_COMPILE)gcc

OBJS = \
	i2c-test.o 			\
	Log.o				\
	SerialLog.o

all: i2c-test

i2c-test: svn-version.h $(OBJS)

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions i2c-test

depend .depend dep: svn-version.h
	@echo "Creating dependencies ..."
	$(CC) $(CFLAGS) $(CPPFLAGS) -M *.c $(COMMON)/*.c > .depend

FORCE:

.PHONY: FORCE

PREPROCESS.c = $(CC) $(CPPFLAGS) $(TARGET_ARCH) -E -Wp,-C,-dD,-dI

%.pp : %.c FORCE
	$(PREPROCESS.c) $< > $@
	
include ../../svn-version.mk

ifeq ($(strip $(filter clean, $(MAKECMDGOALS))),)
-include .depend
endif

