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
*   @file   int6.c 
*
*   @brief  Example which shows how to use INT6 and INT7.
*
*****************************************************************************/

/* ---- Include Files ----------------------------------------------------- */

#include <avr/io.h>
#if defined( __AVR_LIBC_VERSION__ )
#   include <avr/interrupt.h>
#else
#   include <avr/signal.h>
#endif
#include <stdio.h>
#include <inttypes.h>

#include "Hardware.h"
#include "Log.h"
#include "Delay.h"
#include "Timer.h"
#include "UART.h"
#include "Robostix.h"

/* ---- Public Variables -------------------------------------------------- */

/* ---- Private Constants and Types --------------------------------------- */

/* ---- Private Variables ------------------------------------------------- */

uint8_t gInt6Timer = 0; // For debounce
uint8_t gInt7Timer = 0; // For debounce

/* ---- Private Function Prototypes --------------------------------------- */


#define LOG_ENABLED  1

#if LOG_ENABLED
#   define  LOG0( fmt )                     LogBuf0( fmt )
#   define  LOG1( fmt, arg1 )               LogBuf1( fmt, arg1 )
#   define  LOG2( fmt, arg1, arg2 )         LogBuf2( fmt, arg1, arg2 )
#   define  LOG3( fmt, arg1, arg2, arg3 )   LogBuf3( fmt, arg1, arg2, arg3 )
#else
#   define  LOG0( fmt )
#   define  LOG1( fmt, arg1 )
#   define  LOG2( fmt, arg1, arg2 )
#   define  LOG3( fmt, arg1, arg2, arg3 )
#endif

/* ---- Functions --------------------------------------------------------- */

//***************************************************************************
/**
*   Main loop for the I2C I/O program.
*/

int main(void)
{
    int     count;

    InitHardware();

    // The first handle opened for read goes to stdin, and the first handle
    // opened for write goes to stdout. So u0 is stdin, stdout, and stderr

#if LOG_ENABLED

#if defined( __AVR_LIBC_VERSION__ )
    fdevopen( UART0_PutCharStdio, UART0_GetCharStdio );
#else
    fdevopen( UART0_PutCharStdio, UART0_GetCharStdio, 0 );
#endif
    LogInit( stdout );

    Log( "*****\n" );
    Log( "***** INT6/7 Sample Program\n" );
    Log( "*****\n" );

#endif // LOG_ENABLED

    // Grab the rising edge.

    EICRB |= (( 1 << ISC71 ) | ( 1 << ISC70 ) | ( 1 << ISC61 ) | ( 1 << ISC60 ));

    EIFR   = (( 1 << INTF7 ) | ( 1 << INTF6 ));
    EIMSK |= (( 1 << INT7 )  | ( 1 << INT6 ));

    DDRE &= ~(( 1 << 6 ) | ( 1 << 7 ));
    PORTE |= (( 1 << 6 ) | ( 1 << 7 ));

    BLUE_LED_DDR   = BLUE_LED_MASK;
    YELLOW_LED_DDR = YELLOW_LED_MASK;

    LED_OFF( YELLOW );
    LED_OFF( BLUE );

    sei();

    // The main loop just does an interesting heartbeat with a two pulses
    // close together, followed by a longer pause.

    count = 0;

    LOG0( "Testing\n" );

    while ( 1 )
    {
        tick_t prevTick;

        switch ( count )
        {
            case   0:   LED_ON(  RED ); break;
            case  20:   LED_OFF( RED ); break;
            case  60:   count = -1;     break;
        }
        count++;

        if ( gInt6Timer > 0 )
        {
            if ( --gInt6Timer == 0 )
            {
                // Re-enable IRQ6, clearing any bounces

                EIFR   = ( 1 << INTF6 );
                EIMSK |= ( 1 << INT6 );
            }
        }

        if ( gInt7Timer > 0 )
        {
            if ( --gInt7Timer == 0 )
            {
                // Re-enable IRQ7, clearing any bounces

                EIFR   = ( 1 << INTF7 );
                EIMSK |= ( 1 << INT7 );
            }
        }

        prevTick = gTickCount;
        while ( gTickCount == prevTick )
        {
#if LOG_ENABLED
            LogBufDump();
#endif
        }
    }

    return 0;

} // main

//***************************************************************************
/**
*   Interrupt 6 handler
*/

SIGNAL(SIG_INTERRUPT6)
{
    LOG0( "INT6\n" );

    LED_TOGGLE( BLUE );

    EIMSK &= ~( 1 << INT6 );

    gInt6Timer = 2; // Irq 6 stays disabled for 2 x 10ms ticks

} // SIG_INTERRUPT6

//***************************************************************************
/**
*   Interrupt 7 handler
*/

SIGNAL(SIG_INTERRUPT7)
{
    LOG0( "INT7\n" );

    LED_TOGGLE( YELLOW );

    EIMSK &= ~( 1 << INT7 );

    gInt7Timer = 2; // Irq 6 stays disabled for 2 x 10ms ticks

} // SIG_INTERRUPT7

