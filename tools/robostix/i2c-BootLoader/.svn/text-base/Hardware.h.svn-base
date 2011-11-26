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
*   @file   Hardware.h
*
*   @brief  Defines all of the hardware definitions for the chip.
*
****************************************************************************/

#if !defined( HARDWARE_H )
#define HARDWARE_H            /**< Include Guard                           */

/* ---- Include Files ---------------------------------------------------- */

#include "Config.h"

#include <avr/io.h>

/* ---- Constants and Types ---------------------------------------------- */

//--------------------------------------------------------------------------
// UART settings

#define UART_BAUD_RATE   38400

#define UART_DATA_BIT_8  ( 1 << UCSZ1 ) | ( 1 << UCSZ0 )
#define UART_PARITY_NONE ( 0 << UPM1 )  | ( 0 << UPM0 )
#define UART_STOP_BIT_1  ( 1 << USBS )

#define UBRR_INIT   (( CFG_CPU_CLOCK / 16 / UART_BAUD_RATE ) - 1 )
#define UCSRA_INIT  0

#if ( CFG_UART_RX_BUFFER_SIZE > 0 )
#   define UCSRB_INIT  ( 1 << RXCIE ) | ( 1 << RXEN ) | ( 1 << TXEN )
#else
#   define UCSRB_INIT   ( 1 << RXEN ) | ( 1 << TXEN )
#endif

#if defined( URSEL )
#   define  URSEL_INIT  ( 1 << URSEL )
#else
#   define  URSEL_INIT  0
#endif

#define UCSRC_INIT  URSEL_INIT | UART_DATA_BIT_8 | UART_PARITY_NONE | UART_STOP_BIT_1

/* ---- Variable Externs ------------------------------------------------- */

/**
 *  Description of variable.
 */

/* ---- Function Prototypes ---------------------------------------------- */

void InitHardware( void );
 

/** @} */

#endif // HARDWARE_H

