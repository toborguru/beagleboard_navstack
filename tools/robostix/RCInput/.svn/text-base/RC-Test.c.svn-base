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
#include <string.h>

#include "Hardware.h"
#include "Timer.h"
#include "UART.h"

#include "lcd.h"
#include "Log.h"

#include "Delay.h"
#include "RCInput.h"

#define NUM_CHANNELS    2
#define NUM_AVG_SAMPLES 8

typedef struct
{
    uint16_t   channel[NUM_CHANNELS];

} Sample_t;

volatile Sample_t   gPulseWidth[ NUM_AVG_SAMPLES ];
volatile Sample_t   gPulseAggr;
volatile uint8_t    gPulseIdx;

static void PulseDetected( uint8_t channel, uint16_t pulseWidth )
{
    if (( channel >= 1 ) && ( channel <= 2 ))
    {
        uint8_t chanIdx = channel - 1;

        gPulseAggr.channel[ chanIdx ] -= gPulseWidth[ gPulseIdx ].channel[ chanIdx ];

        gPulseWidth[ gPulseIdx ].channel[ chanIdx ] = pulseWidth;

        gPulseAggr.channel[ chanIdx ] += pulseWidth;

        if ( channel == 2 )
        {
            gPulseIdx++;
            gPulseIdx %= NUM_AVG_SAMPLES;
        }
    }

} // PulseDetected

static void ClearMem( volatile void *ptr, uint8_t numBytes )
{
    volatile uint8_t    *dst = (volatile uint8_t *)ptr;

    while ( numBytes > 0 )
    {
        *dst++ = 0;
        numBytes--;
    }
}

static void MissingPulse( void )
{
    ClearMem( gPulseWidth, sizeof( gPulseWidth ));
    ClearMem( &gPulseAggr, sizeof( gPulseAggr ));

} // MissingPulse

int main(void)
{
    int     i;
    int     led = 0;
    FILE   *lcd;
    char   *spinChar = " .oO";
    uint8_t spinIdx = 0;

    InitHardware();

    // The first handle opened for read goes to stdin, and the first handle
    // opened for write goes to stdout. So u0 is stdin, stdout, and stderr

    fdevopen( UART0_PutCharStdio, UART0_GetCharStdio );

    LogInit( stdout );

    lcd = fdevopen( LCD_PutCharStdio, NULL );

    printf( "*****\n" );
    printf( "***** RCI-Test program\n" );
    printf( "*****\n" );

    LCD_Init( 4, 20 );

    fprintf( lcd, "**** R/C Input ****\n" );

    RCI_SetPulseCallback( PulseDetected );
    RCI_SetMissingPulseCallback( MissingPulse );
    
    RCI_Init();

    sei();

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

        if ( ++led > 1 )
        {
            led = 0;
        }

        spinIdx++;
        spinIdx &= 3;

        LCD_MoveTo( 0, 1 );
        fprintf( lcd, "%c", spinChar[ spinIdx ] );

        // Tick rate is 100/sec so waiting for 100 waits for 1 sec

        for ( i = 0; i < 10; i++ ) 
        {
            int j;
            int channel;

            LCD_MoveTo( 0, 2 );

            for ( channel = 0; channel < 2; channel++ ) 
            {
                if ( gPulseAggr.channel[ channel ] == 0 )
                {
                    fprintf( lcd, "%d: No Pulse\n", channel + 1 );
                }
                else
                {
                    uint16_t pulseWidth = gPulseAggr.channel[ channel ] / NUM_AVG_SAMPLES;
                    pulseWidth += 5;
                    pulseWidth /= 10;

                    fprintf( lcd, "%d: %4d    \n", channel + 1, pulseWidth );
                }

            }

            for ( j = 0; j < 10; j++  ) 
            {
                LogBufDump();
                WaitForTimer0Rollover();

                if ( UART0_IsCharAvailable() )
                {
                    char    ch = getchar();

                    printf( "Read: '%c'\n", ch );

                    if ( ch == ' ' )
                    {
                        printf( "*** Press a key to continue\n" );
                        ch = getchar();
                        printf( "*** Continuing...\n" );
                    }
                }
            }
        }
    }

    return 0;
}

