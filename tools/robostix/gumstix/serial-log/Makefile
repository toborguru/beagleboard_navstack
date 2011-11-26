#############################################################################
#
#	Makefile for building the serial-log program
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
KERNELDIR 		   ?= $(BUILD_ARM)/linux-2.6.11gum
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
	serial-log.o		\
	Log.o				\
	SerialLog.o

all: serial-log

serial-log: $(OBJS)

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions serial-log

depend .depend dep:
	@echo "Creating dependencies ..."
	$(CC) $(CFLAGS) $(CPPFLAGS) -M *.c $(COMMON)/*.c > .depend

FORCE:

.PHONY: FORCE

PREPROCESS.c = $(CC) $(CPPFLAGS) $(TARGET_ARCH) -E -Wp,-C,-dD,-dI

%.pp : %.c FORCE
	$(PREPROCESS.c) $< > $@
	
ifeq ($(strip $(filter clean, $(MAKECMDGOALS))),)
-include .depend
endif

