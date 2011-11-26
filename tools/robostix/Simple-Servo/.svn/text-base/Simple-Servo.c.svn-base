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

#include "a2d.h"
#include "Hardware.h"
#include "Delay.h"
#include "Timer.h"
#include "UART.h"

//--------------------------------------------------------------------------
//
// Servo pins (relies on definitions from Timer.h)
//

#define SERVO_1 3A
#define SERVO_2 3B

#define SERVO__( ocr, reg )     TIMER_ ## ocr ## _ ## reg
#define SERVO_( ocr, reg )      SERVO__( ocr, reg )
#define SERVO( num, reg )       SERVO_( SERVO_ ## num, reg )

uint8_t gADC[ 8 ];

int main(void)
{
    int     i;
    char   *spinner = "/-\\|";
    int     spinnerIdx = 0;

    InitHardware();

    // The first handle opened for read goes to stdin, and the first handle
    // opened for write goes to stdout.

#if defined( __AVR_LIBC_VERSION__ )
    fdevopen( UART0_PutCharStdio, UART0_GetCharStdio );
#else
    fdevopen( UART0_PutCharStdio, UART0_GetCharStdio, 0 );
#endif

    printf( "\n\n*****\n" );
    printf( "***** Simple-Servo program\n" );
    printf( "*****\n\n" );

    // If we run Timer 3 (a 16 bit timer) at 2 MHz (divde by 8 prescalar)
    // then it will overflow every 32.7 msec which is suitable for driving
    // R/C servos.
    //
    // We use WGM mode 14, Fast PWM with the TOP value supplied by ICR3.
    // We use COM mode 2, which causes the OC3x pin to be cleared on a match
    // with OCR3x and set when the timer reaches TOP.
    //
    // With TOP set to 40,000, we get 40,000 1/2 usec counts or about 20 msec
    // between pulses (50 Hz) which is a suitable pulse rate for R/C servos.

    ICR3 = 40000u;
    SERVO( 1, OCR ) = 1500u * 2;   // 1500 usec is servo mid point
    SERVO( 2, OCR ) = 1500u * 2;
    TCNT3 = 0;

    // Set the WGM mode & prescalar

    TCCR3A = ( 1 << WGM31 ) | ( 0 << WGM30 );
    TCCR3B = ( 1 << WGM33 )  | ( 1 << WGM32 ) | TIMER3_CLOCK_SEL_DIV_8;

    // Set the servo pins to COM mode 2 (COM_1 = 1, COM_0 = 0)
    // This causes the OCR pin to fall which the counter reaches the value
    // stored in the OCR register and causes the OCR pin to rise when the 
    // counter reaches TOP (gets reset to zero).

    SERVO( 1, TCCRA ) |= ( 1 << SERVO( 1, COM_1 ));
    SERVO( 2, TCCRA ) |= ( 1 << SERVO( 2, COM_1 ));
                 
    // Set the servo pins to be outputs

    SERVO( 1, DDR ) |= SERVO( 1, MASK );
    SERVO( 2, DDR ) |= SERVO( 2, MASK );

    while( 1 )
    {
        uint16_t    pulse1_usec;
        uint16_t    pulse2_usec;

        // Read the ADC's

        gADC[ 0 ] = a2d_8( 0 );
        gADC[ 1 ] = a2d_8( 1 );

        // "nominal" servo pulses are between 1 ms and 2 ms (1000 usec and 2000 usec)
        //  Some servos have a wider range, so I used 500usec thru 2500 usec range
        // and the user can use this to find the limits.
        //
        // The "nominal" formula would be:
        //
        //  pulse (in usec) = ( adc / 255 ) * (pulse_max - pulse_min) + pulse_min;
        //
        // Since we're using integer arithmetic, we need to rearrange things:
        //
        //  pulse = ( adc * 2000 ) / 255 + 500
        //
        // However, adc * 2000 exceeds 16 bits. 2000 / 255 is close enough to 8
        // for our purposes, so we rewrite it to get:
        //
        //  pulse = ( adc * 8 ) + 500
        //
        // The actual range of this formula is 500 - 2540
        //
        // We then need to multiply the pulse in usec by 2, since out timer 
        // runs with 1/2 usec ticks.

        pulse1_usec = ( (uint16_t)gADC[ 0 ] * 8u ) + 500u;
        pulse2_usec = ( (uint16_t)gADC[ 1 ] * 8u ) + 500u;

        // The OCR3x registers are double buffered so we can set them at any 
        // time.

        SERVO( 1, OCR ) = pulse1_usec * 2;    // * 2 is because timer ticks every 1/2 usec.
        SERVO( 2, OCR ) = pulse2_usec * 2;

        // We put a delay in here so we aren't trying to update the OCR 
        // registers too frequently. Waiting to 2 ticks coincides with the
        // pulse rate.

        for ( i = 0; i < 2; i++ ) 
        {
            // Toggle our heartbeat LED and update the display 4 times a second.
            // gTickCount increments every 10 msec

            WaitForTimer0Rollover();

            if (( gTickCount % 25 ) == 0 )
            {
                // The main loop runs once every 30 msec, and we toggle every 8th
                // time through, so we toggle about 4 times/sec.

                LED_TOGGLE( RED );

                printf( "%c Servo1 adc: %3d pulse: %4d  Servo2 adc: %3d pulse: %4d    \r",
                        spinner[ spinnerIdx ], gADC[ 0 ], pulse1_usec, gADC[ 1 ], pulse2_usec );

                spinnerIdx++;
                spinnerIdx &= 0x03;
            }
        }
    }

    return 0;
}

