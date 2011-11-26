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
#include <stdio.h>

#include "Hardware.h"
#include "Timer.h"
#include "UART.h"

uint8_t gADC[ 8 ];

int main(void)
{
    int     i;
    int     led = 0;
    FILE   *u0;
    uint8_t prevPinD;
    uint8_t forcePrint;
    uint8_t ddrd;
    uint8_t pind;
    uint8_t portd;

    InitHardware();

    // The first handle opened for read goes to stdin, and the first handle
    // opened for write goes to stdout. So u0 is stdin, stdout, and stderr

#if defined( __AVR_LIBC_VERSION__ )
    u0 = fdevopen( UART0_PutCharStdio, UART0_GetCharStdio );
#else
    u0 = fdevopen( UART0_PutCharStdio, UART0_GetCharStdio, 0 );
#endif

    printf( "*****\n" );
    printf( "***** i2c-test program\n" );
    printf( "*****\n" );

    // Set SDA and SCL as input

    PORTD &= ~3;    // Turn off pullups
    DDRD &= ~3;     // Make inputs

    // Disable TWI

    TWCR = 0;

    prevPinD = 0;
    forcePrint = 0;

    while( 1 )
    {
        // Turn all of the LEDs off

        LED_OFF( RED );
        LED_OFF( BLUE );
        LED_OFF( YELLOW );

        switch ( led )
        {
            case    0:  LED_ON( BLUE );     break;
            case    1:  LED_OFF( BLUE );    break;
        }

        if ( ++led >= 2 )
        {
            led = 0;
        }

        // Tick rate is 100/sec so waiting for 100 waits for 1 sec

        for ( i = 0; i < 100; i++ ) 
        {
            WaitForTimer0Rollover();

            if ( UART0_IsCharAvailable() )
            {
                char    ch = getchar();

                if (( ch == 'c' ) || ( ch == 'C' ))
                {
                    // Toggle SCL (aka PD0)

                    if ((( DDRD & 1 ) == 1 ) && (( PORTD & 1 ) == 0 ))
                    {
                        // Currently output, driving low, change to input

                        DDRD &= ~1;
                    }
                    else
                    if ((( DDRD & 1 ) == 1 ) && (( PORTD & 1 ) == 1 ))
                    {
                        // Currently output, driving high, change to low

                        PORTD &= ~1;
                    }
                    else
                    {
                        // Currently input, change to output, driving high

                        DDRD  |= 1;
                        PORTD |= 1;
                    }
                }
                else
                if (( ch == 'd' ) || ( ch == 'D' ))
                {
                    // Toggle SDA (aka PD1)

                    if ((( DDRD & 2 ) == 2 ) && (( PORTD & 2 ) == 0 ))
                    {
                        // Currently output, driving low, change to input

                        DDRD &= ~2;
                    }
                    else
                    if ((( DDRD & 2 ) == 2 ) && (( PORTD & 2 ) == 2 ))
                    {
                        // Currently output, driving high, change to low

                        PORTD &= ~2;
                    }
                    else
                    {
                        // Currently input, change to output, driving high

                        DDRD  |= 2;
                        PORTD |= 2;
                    }
                }
                forcePrint = 1;
            }

            portd = PORTD & 3;
            ddrd  = DDRD  & 3;
            pind  = PIND  & 3;

            if ( pind != prevPinD )
            {
                forcePrint = 1;
                prevPinD = pind;
            }

            if ( forcePrint )
            {
                int     sda_w;
                int     scl_w;
                int     sda_r;
                int     scl_r;
                int     scl_z;
                int     sda_z;

                scl_z = (( ddrd & 1 ) == 0 );
                scl_w = (( portd & 1 ) != 0 );

                sda_z = (( ddrd & 2 ) == 0 );
                sda_w = (( portd & 2 ) != 0 );

                scl_r = ( pind & 1 ) != 0;
                sda_r = ( pind & 2 ) != 0;

                printf( "robostix: SCLw: %c SCLr: %d SDAw: %c SDAr: %d DDRD: %d PIND: %d, PORTD: %d\n\n", 
                        ( scl_z ? 'Z' : ( scl_w ? '1' : '0' )), scl_r, 
                        ( sda_z ? 'Z' : ( sda_w ? '1' : '0' )), sda_r, ddrd, pind, portd );

                forcePrint = 0;
            }
        }
    }

    return 0;
}

