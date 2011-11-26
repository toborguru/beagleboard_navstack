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
*   @file   Config.h
*
*   @brief  Global Configuration information.
*
****************************************************************************/

#if !defined( CONFIG_H )
#define CONFIG_H              /**< Include Guard                           */

/* ---- Include Files ---------------------------------------------------- */

#include "Config-LED.h"

/* ---- Constants and Types ---------------------------------------------- */

/**
 *  Defines the clock frequency, in Hz.
 */

#if !defined( CFG_CPU_CLOCK )
#define CFG_CPU_CLOCK   8000000
#endif

#define CFG_TIMER_8_BIT_TICK            1
#define CFG_TIMER_MICRO_TICK            0
#define CFG_TIMER_WAITFORTIMER0ROLLOVER 0

/**
 *  Sets Logging parameters
 */

#define CFG_LOG_TO_BUFFER   1
#define CFG_LOG_NUM_BUFFER_ENTRIES  128

#if STANDALONE

#define CFG_USE_UART        1

/**
 *  Sets the size of the Uart Rx buffer
 */

#define CFG_UART_RX_BUFFER_SIZE     16
#define CFG_UART_TX_BUFFER_SIZE     16
#define CFG_UART_LF_TO_CRLF         1

#else

#define CFG_USE_UART        0

/**
 *  Sets the size of the Uart Rx buffer
 */

#define CFG_UART_RX_BUFFER_SIZE     0
#define CFG_UART_TX_BUFFER_SIZE     0   
#define CFG_UART_LF_TO_CRLF         1

#endif  // STANDALONE

#define CFG_I2C_USE_CRC     1

//---------------------------------------------------------------------------
//
// The FORCE pin, when connected to ground, causes the bootloader to be
// forced (i.e. the user program will not be run)
//
// It's convenient to use the MISO pin, which has a ground opposite from it
// on the 10 pin ISP connector.

#if defined (__AVR_ATmega8__) \
 || defined (__AVR_ATmega48__) \
 || defined (__AVR_ATmega88__) \
 || defined (__AVR_ATmega168__)

// PortB4 is MISO during ISP programming

#define CFG_FORCE_PORT  PORTB
#define CFG_FORCE_PIN   PINB
#define CFG_FORCE_DDR   DDRB
#define CFG_FORCE_MASK  ( 1 << 4 )

#elif defined (__AVR_ATmega16__) \
   || defined (__AVR_ATmega32__)
                                                          
// PortB6 is MISO during ISP programming

#define CFG_FORCE_PORT  PORTB
#define CFG_FORCE_PIN   PINB
#define CFG_FORCE_DDR   DDRB
#define CFG_FORCE_MASK  ( 1 << 6 )

#elif defined (__AVR_ATmega64__) \
   || defined (__AVR_ATmega128__)

// PortE1 is TxD for UART0, or MISO during ISP programming
                            
#define CFG_FORCE_PORT  PORTE
#define CFG_FORCE_PIN   PINE
#define CFG_FORCE_DDR   DDRE
#define CFG_FORCE_MASK  ( 1 << 1 )

#else
#   error Unrecognized processor in Config.h for CFG_FORCE_xxx
#endif

/** @} */

#endif // CONFIG_H

