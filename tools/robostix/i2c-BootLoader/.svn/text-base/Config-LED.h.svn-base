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
/**
*
*   @file   Config-LED.h
*
*   @brief  Contains definitions for the LEDs used by the booloader. This
*           was split out so that it could be shared with i2c-io
*
****************************************************************************/

#if !defined( CONFIG_LED_H )
#   define CONFIG_LED_H

#if defined (__AVR_ATmega8__) \
 || defined (__AVR_ATmega48__) \
 || defined (__AVR_ATmega88__) \
 || defined (__AVR_ATmega168__)

#define CFG_BOOTLOADER_BEAT_PORT    PORTB
#define CFG_BOOTLOADER_BEAT_DDR     DDRB
#define CFG_BOOTLOADER_BEAT_MASK    ( 1 << 4 )

#elif defined (__AVR_ATmega16__) \
   || defined (__AVR_ATmega32__)

// On my i2c test setup, PD6 (pin 20) is red and PD6 (pin 21) is green

#define CFG_I2C_LED_PORT            PORTD
#define CFG_I2C_LED_DDR             DDRD
#define CFG_I2C_LED_MASK            ( 1 << 7 )

#define CFG_BOOTLOADER_BEAT_PORT    PORTD
#define CFG_BOOTLOADER_BEAT_DDR     DDRD
#define CFG_BOOTLOADER_BEAT_MASK    ( 1 << 6 )

#elif defined (__AVR_ATmega64__) \
   || defined (__AVR_ATmega128__)

// On the Robostix, Red is PG4, Blue is PG3

//#define CFG_I2C_LED_PORT            PORTG
//#define CFG_I2C_LED_DDR             DDRG
//#define CFG_I2C_LED_MASK            ( 1 << 3 )

#define CFG_BOOTLOADER_BEAT_PORT    PORTG
#define CFG_BOOTLOADER_BEAT_DDR     DDRG
#define CFG_BOOTLOADER_BEAT_MASK    ( 1 << 4 )

#else
#   error Unrecognized processor in Config-LED.h for CFG_BOOTLOADER_BEAT_xxx
#endif

#endif  // CONFIG_LED_H
