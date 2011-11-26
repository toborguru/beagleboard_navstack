###########################################################################
#
# i2c-io Makefile
#
###########################################################################

# CPU_MCU is the flavor of atmega that we're using.

CPU_MCU  ?= 128
CPU_FREQ ?= 16

AVR_MCU = atmega$(CPU_MCU)
CPPFLAGS += -DCFG_CPU_CLOCK=$(CPU_FREQ)000000

MAIN_OBJS = i2c-io.o

COMMON_OBJS = \
	i2c-slave-boot.o \
	Delay.o 	\
	Timer.o 	\
	Hardware.o

#COMMON_OBJS += \
#	UART.o 		\
#	Log.o

ifeq ($(CPU_MCU),128)

TARGET	= i2c-io
COMMON_OBJS	+= memcpy_EP.o

else

TARGET	= i2c-io-m$(CPU_MCU)

endif


all: svn-version.h

include ../Rules.mk
include ../svn-version.mk

