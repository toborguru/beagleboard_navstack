/****************************************************************************
*
*   Copyright (c) 2006 Dave Hylands     <dhylands@gmail.com>
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation.
*
*   Alternatively, this software may be distributed under the terms of BSD
*   license.
*
*   See README and COPYING for more details.
*
****************************************************************************/

#include <avr/io.h>

#if __AVR_MEGA__
  #define XJMP jmp
  #define XCALL call
#else
  #define XJMP rjmp
  #define XCALL rcall
#endif

	.macro	vector name
	.if (. - boot_vectors < _VECTORS_SIZE)
	.weak	\name
	.set	\name, start
	XJMP	\name
	.endif
	.endm

	.section .boot_vectors,"ax",@progbits
	.global	boot_vectors
	.func	boot_vectors
boot_vectors:
	XJMP	start

        .global jump_table

        .word   0xB007                  // Magic number
        .word   pm( jump_table )        // Address of Jump Table
        
        .global AppBoot
        .global I2C_SlaveHandler
        .global BootLoaderProcessCommand

jump_table:
        .word   3               // Number of entries in the table
        .word   pm( AppBoot )
        .word   pm( I2C_SlaveHandler )
        .word   pm( BootLoaderProcessCommand )

#if __AVR_MEGA__

        // For the ATMega16 and above, each vector is 4 bytes        

        .word   0               // pad (vector 4)
        .word   0               // pad

        .word   0               // pad (vector 5)
        .word   0               // pad

        .word   0               // pad (vector 6)
        .word   0               // pad

        .word   0               // pad (vector 7)
        .word   0               // pad

        .word   0               // pad (vector 8)
        .word   0               // pad

#else
        // For the ATMega8 and smaller, each vector is only 2 bytes                                                 
                                                 
        .word   0               // pad (vector 7)
        .word   0               // pad (vector 8)

#endif  // __AVR_MEGA__

        // We only use SIG_OVERFLOW0 and SIG_2WIRE_SERIAL

        // Processor  Overflow0 TWI
        // ---------  --------- ---
        // ATMegax8     16      24
        // ATMega8       9      17
        // ATMega16      9      17
        // ATMega32     11      19
        // ATMega64     16      33
        // ATMega128    16      33

        
	vector	__vector_9      // OverFlow0
	vector	__vector_10
	vector	__vector_11     // OverFlow0
	vector	__vector_12
	vector	__vector_13
	vector	__vector_14
	vector	__vector_15
	vector	__vector_16     // OverFlow0
	vector	__vector_17     // TWI
	vector	__vector_18
	vector	__vector_19     // TWI
	vector	__vector_20
	vector	__vector_21
	vector	__vector_22
	vector	__vector_23
	vector	__vector_24     // TWI
	vector	__vector_25
	vector	__vector_26
	vector	__vector_27
	vector	__vector_28
	vector	__vector_29
	vector	__vector_30
	vector	__vector_31
	vector	__vector_32
	vector	__vector_33     // TWI

        // We don't need any vectors higher than 33, so don't waste the flash

	.endfunc

#define __zero_reg__    r1

        .global __stack
	.set	__stack, RAMEND

    .text

        .global start
        .global BootLoader

start:
	clr	__zero_reg__
	out	_SFR_IO_ADDR(SREG), __zero_reg__
	ldi	r28,lo8(__stack)
	ldi	r29,hi8(__stack)
	out	_SFR_IO_ADDR(SPH), r29
	out	_SFR_IO_ADDR(SPL), r28

        ldi     r24,lo8(0)
        ldi     r25,hi8(0)
                       
	XJMP	BootLoader
                          
