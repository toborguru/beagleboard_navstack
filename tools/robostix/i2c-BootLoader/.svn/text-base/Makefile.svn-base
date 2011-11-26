###########################################################################
#
#	Makefile to build i2c-Bootloader
#
###########################################################################

# CPU_MCU is the flavor of atmega that we're using.

CPU_MCU  ?= 128
CPU_FREQ ?= 16

# Set STANDALONE to y in order to build a standalone, non-bootloader
# version for testing.

STANDALONE ?= n

STANDALONE_n_suffix =
STANDALONE_y_suffix = -sa

# The Bootloader needs 4K

BOOT_START_m8   = 0x1800
BOOT_START_m32  = 0x7000
BOOT_START_m128 = 0x1F000

TARGET 	= i2c-Boot-m$(CPU_MCU)-$(CPU_FREQ)MHz$(STANDALONE_$(STANDALONE)_suffix)

BOOT_START = $(BOOT_START_m$(CPU_MCU))

CPPFLAGS += -DCFG_CPU_CLOCK=$(CPU_FREQ)000000

CFLAGS += -gstabs

AVR_MCU = atmega$(CPU_MCU)

MAIN_OBJS = BootLoader.o

COMMON_OBJS = \
	i2c-slave.o	\
	Crc8.o		\
	Timer.o		\
	eeprom-init.o

ifeq ($(CPU_MCU),128)
COMMON_OBJS += memcpy_EP.o
endif

#LINK.o += --verbose	

ifeq ($(STANDALONE),y)
#--------------------------------------------------------------------------					   

CPPFLAGS += -DSTANDALONE=1

#COMMON_OBJS += Hardware.o Log.o UART.o

#--------------------------------------------------------------------------					   
else
#--------------------------------------------------------------------------					   

MAIN_OBJS := $(MAIN_OBJS) vectors.o

CFLAGS += -mshort-calls

LDFLAGS += -Wl,--section-start=.text=$(BOOT_START),-T,BootLoader.lds,--entry=start,--defsym,zero_addr=0 -nostdlib
LDLIBS  += -lc

#--------------------------------------------------------------------------					   
endif

include ../Rules.mk

$(TARGET) : $(TARGET).hex $(TARGET)-eep.hex
$(TARGET).hex : $(TARGET).elf
$(TARGET)-eep.hex : $(TARGET).elf

# The following target build eeprom-init.hex which can be used with uisp 
# to reset the EEPROM back to it's default state										

eeprom-init.hex : eeprom-init.o
	$(ECHO) "Creating $@ ..."
	avr-objcopy --change-section-address .eeprom=0xFEC $<
	avr-objcopy -j .eeprom -O ihex $< $@
