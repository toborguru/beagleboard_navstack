###########################################################################
#
# Top level Makefile - builds all of the sample projects
#
###########################################################################

TARGETS = \
   ArgTest/	     \
   Flash-LED/        \
   i2c-BootLoader/   \
   i2c-io/           \
   i2c-test/         \
   int6/	     \
   LCD-Test/         \
   QD-Test/          \
   RCInput/          \
   Simple-Flasher/   \
   Simple-Servo/     \
   Simple-Servo-2/   \
   Tachometer/

SUBDIRS := $(filter %/,$(TARGETS))

all: $(TARGETS)

.PHONY: all FORCE

FORCE:

#--------------------------------------------------------------------------
#
# 	Run make with v=1 or verbose=1 to get verbose output
#

ifeq ($(v),)
export verbose = 0
else
export verbose = 1
endif

ifeq ($(verbose),)
export verbose = 0
endif

ifeq ($(verbose),0)
	Q = @
	MAKEFLAGS += -s
else
	Q =
endif
export Q

#--------------------------------------------------------------------------
#
# 	Rules to clean subdirectories
#

clean-dirs := $(addprefix _clean_,$(SUBDIRS))

.PHONY: clean $(clean-dirs)

clean: $(clean-dirs)

$(clean-dirs):
	@echo "Cleaning $(patsubst _clean_%,%,$@) ..."
	$(Q)$(MAKE) -C $(patsubst _clean_%,%,$@) clean

#--------------------------------------------------------------------------
#
# 	Rule to build subdirectories
#

%/: FORCE
	@echo "Building $@ ..."
	$(Q)$(MAKE) -C $@ $(filter clean% exec print-%,$(MAKECMDGOALS))


