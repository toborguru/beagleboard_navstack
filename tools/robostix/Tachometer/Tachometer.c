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
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "a2d.h"
#include "Hardware.h"
#include "Tach.h"
#include "Timer.h"
#include "UART.h"

#define SAMPLE_SIZE 4

typedef struct
{
    uint16_t    dummyLocZero;
    uint16_t    cyclesPerRev;
    uint8_t     ocr2;
    
} EE_Data_t;


EE_Data_t __attribute__((section (".eeprom"))) eeprom =
{
    0,
    1,
    255
};

EE_Data_t   eeRam;

char        gLine[ 20 ];

#if 0
void SetTop( uint16_t newTop )
{
    uint32_t    oldTop = TOP;

    if ( newTop < 10 )
    {
        return;
    }

    if ( newTop != oldTop )
    {
        uint32_t    oldPwm = PWM;
        uint32_t    newPwm;

        // Adjust the PWM so that the ration stays the same. Since top and
        // oldPwm are both 16 bit numbers, the result will fit in a 32 bit
        // answer.

        newPwm = (uint32_t)newTop * oldPwm / oldTop;

        TOP = newTop;
        PWM = newPwm;
    }
}
#endif

int main(void)
{
    int         i;
    uint32_t    prevEndTime = 0;
    char       *spinner = "/-\\|";
    int         spinnerIdx = 0;

    InitHardware();

    // The first handle opened for read goes to stdin, and the first handle
    // opened for write goes to stdout. So u0 is stdin, stdout, and stderr

#if defined( __AVR_LIBC_VERSION__ )
    fdevopen( UART0_PutCharStdio, UART0_GetCharStdio );
#else
    fdevopen( UART0_PutCharStdio, UART0_GetCharStdio, 0 );
#endif

    printf( "*****\n" );
    printf( "***** Tachometer/Motor Controller program\n" );
    printf( "*****\n" );

    // Read in the data from EEPROM

    eeprom_read_block( &eeRam, &eeprom, sizeof( eeprom ));

    if (( eeRam.cyclesPerRev == 0 ) || ( eeRam.cyclesPerRev == 0xFFFF ))
    {
        eeRam.cyclesPerRev = 1;
    }

    printf( "Cycles/Rec = %d\n", eeRam.cyclesPerRev );
    printf( "ocr2       = %d\n", eeRam.ocr2 );

    OCR2 = eeRam.ocr2;

    while( 1 )
    {
        uint16_t    top;
        uint16_t    adcTop;
        uint16_t    adcPwm;
        uint32_t    pwm;

        // Use the Red LED like a heartbeat

        LED_TOGGLE( RED );

        // Read the ADC

        adcTop = a2d_8( 0 );
        adcPwm = a2d_8( 1 );

        top = (( (uint16_t)adcTop + 1u ) * 64u ) - 1u;
        pwm = ( (uint32_t)adcPwm * (uint32_t)top ) / 255u;

        TOP = top;
        PWM = pwm;

        printf( "%c ADC: %02x %02x TOP: %04x PWM: %04lx PCT: %3ld Enable:%d\r", 
                spinner[ spinnerIdx ], adcTop, adcPwm, top, pwm, pwm * 100 / top,
                ( ENA_PORT & ENA_MASK ) != 0 );
        
        spinnerIdx++;
        spinnerIdx &= 0x03;

        if ( CBUF_Len( TACH_gBuffer ) > SAMPLE_SIZE )
        {
            uint32_t    startTime;
            uint32_t    endTime;
            uint32_t    deltaTime;
            uint32_t    rpm;

            cli();
            startTime = CBUF_GetEnd( TACH_gBuffer, SAMPLE_SIZE );
            endTime   = CBUF_GetEnd( TACH_gBuffer, 0 );
            sei();

            deltaTime = endTime - startTime;

            if ( deltaTime > 0 )
            {
                if ( endTime == prevEndTime )
                {
                    printf( "RPM: no signal\r" );
                }
                else
                {
                    rpm = SAMPLE_SIZE * TIMER3_FREQ * 60 / ( deltaTime * eeRam.cyclesPerRev );

                    printf( "RPM: %5ld     \r", rpm );
                }
            }
            else
            {
                printf( "deltaTime = %ld\n", deltaTime );
            }

            prevEndTime = endTime;
        }

        // Tick rate is 100/sec so waiting for 25 waits for 1/4 sec

        for ( i = 0; i < 25; i++ ) 
        {
            WaitForTimer0Rollover();

            if ( UART0_IsCharAvailable() )
            {
                char    ch = getchar();

                ch = tolower( ch );

                switch ( ch )
                {
                    case 'c':
                    {
                        printf( "\nCycles/Rev = %d\n", eeRam.cyclesPerRev );
                        printf( "Enter new cycles/rev: " );
                        if ( fgets( gLine, sizeof( gLine ), stdin ) != NULL )
                        {
                            eeRam.cyclesPerRev = atoi( gLine );
                            if (( eeRam.cyclesPerRev == 0 ) || ( eeRam.cyclesPerRev == 0xFFFF ))
                            {
                                eeRam.cyclesPerRev = 1;
                            }
                            eeprom_write_word( &eeprom.cyclesPerRev, eeRam.cyclesPerRev );
                        }
                        break;
                    }

                    case 'd':
                    {
                        printf( "\nDisable Motor\n" );
                        ENA_PORT &= ~ENA_MASK;
                        break;
                    }

                    case 'e':
                    {
                        printf( "\nEnable Motor\n" );
                        ENA_PORT |= ENA_MASK;
                        break;
                    }

                    case 'o':
                    {
                        printf( "\nOCR2 = %d\n", eeRam.ocr2 );
                        printf( "Enter OCR2: " );
                        if ( fgets( gLine, sizeof( gLine ), stdin ) != NULL )
                        {
                            eeRam.ocr2 = atoi( gLine );
                            OCR2 = eeRam.ocr2;
                            eeprom_write_byte( &eeprom.ocr2, eeRam.ocr2 );
                        }
                        break;
                    }
                }
            }
        }
    }

    return 0;
}

