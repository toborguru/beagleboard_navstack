PROGRAMMER=stk200
MCU=atmega8
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-g -mmcu=$(MCU) -Wall -Wstrict-prototypes -Os -mcall-prologues -W

#- default LED-PIN
LED=PD4

#- ... mangle things a bit.
PORT=`echo ${LED} | cut -c2`
PIN=`echo ${LED} | cut -c3`
LEDPIN=-DLEDDDR=DDR${PORT} -DLEDPORT=PORT${PORT} -DLEDPIN=P${PORT}${PIN}

all: i2c-bootloader-1024w.hex i2c-bootloader-512w.hex i2c-test i2cloader 

i2c-bootloader-1024w.hex i2c-bootloader-512w.hex: %.hex: %
	$(OBJCOPY) -R .eeprom -O ihex $< $@

i2c-bootloader-1024w: i2c-bootloader-led.o
	$(CC) $(CFLAGS) -o $@ -Wl,--section-start=.text=0x1800 \
		-Wl,-Map,i2c-bootloader-1024w.map $<

i2c-bootloader-512w: i2c-bootloader.o
	$(CC) $(CFLAGS) -o $@ -Wl,--section-start=.text=0x1C00 \
		-Wl,-Map,i2c-bootloader-512w.map $<

i2c-bootloader-led.o: i2c-bootloader.c
	$(CC) -o $@ -c $< -DMITLED $(CFLAGS) $(LEDPIN)

i2c-bootloader.s: i2c-bootloader.c
	$(CC) $(CFLAGS) -S -DMITLED $<

i2c-test: i2c-test.c i2c-host-routines.c i2c-host-routines.h 
	gcc -Wall -ggdb -o i2c-test i2c-host-routines.c i2c-test.c

i2cloader: i2cloader.c i2c-host-routines.c i2c-host-routines.h 
	gcc -Wall -ggdb -o i2cloader i2c-host-routines.c i2cloader.c

clean:
	rm -f *.map *.hex i2c-bootloader-*w *.o *.s *~ i2c-test i2cloader

load-1024w: i2c-bootloader-1024w.hex
	avrdude -p $(MCU) -U flash:w:$< -E vcc,noreset

load-512w: i2c-bootloader-512w.hex
	avrdude -p $(MCU) -U flash:w:$< -E vcc,noreset


fuse-1024w: #resetvector auf application und bootloaderspeicher auf 0C00
	echo ":01000000D926" | avrdude -p m8 -U hfuse:w:-:i -E vcc,noreset

fuse-512w: #resetvector auf application und bootloaderspeicher auf 0E00
	echo ":01000000DB24" | avrdude -p m8 -U hfuse:w:-:i -E vcc,noreset

fuse-reset-1024w: #resetvector auf bootloader und bootloaderspeicher auf 0C00
	echo ":01000000D827" | avrdude -p m8 -U hfuse:w:-:i -E vcc,noreset

fuse-reset-512w: #resetvector auf bootloader und bootloaderspeicher auf 0E00
	echo ":01000000DA25" | avrdude -p m8 -U hfuse:w:-:i -E vcc,noreset

loaduisp-1024w: i2c-bootloader-1024w.hex
	uisp --erase  -dprog=$(PROGRAMMER)
	uisp --upload if=$< -dprog=$(PROGRAMMER)  -v=3 --hash=32

loaduisp-512w: i2c-bootloader-512w.hex
	uisp --erase  -dprog=$(PROGRAMMER)
	uisp --upload if=$< -dprog=$(PROGRAMMER)  -v=3 --hash=32

loadadd-1024w: i2c-bootloader-1024w.hex
	avrdude -p $(MCU) -D -U flash:w:$< -E vcc,noreset

loadadd-512w: i2c-bootloader-512w.hex
	avrdude -p $(MCU) -D -U flash:w:$< -E vcc,noreset

install: i2cloader
	install i2cloader /usr/local/bin/
