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

#include "Delay.h"
#include "Timer.h"

#define RED_LED_PIN     4
#define RED_LED_MASK    ( 1 << RED_LED_PIN )
#define RED_LED_DDR     DDRG
#define RED_LED_PORT    PORTG

#define BLUE_LED_PIN    3
#define BLUE_LED_MASK   ( 1 << BLUE_LED_PIN )
#define BLUE_LED_DDR    DDRG
#define BLUE_LED_PORT   PORTG

#define YELLOW_LED_PIN  4
#define YELLOW_LED_MASK ( 1 << YELLOW_LED_PIN )
#define YELLOW_LED_DDR  DDRB
#define YELLOW_LED_PORT PORTB

// Setting the pin to 0 turns the LED on

#define LED_ON( color )     color ## _LED_PORT &= ~color ## _LED_MASK
#define LED_OFF( color )    color ## _LED_PORT |= color ## _LED_MASK

int main(void)
{
    int led;

    InitTimer();

    ASSR &= ~( 1 << AS0 );  // Make sure Port G LED pins are setup for I/O

    RED_LED_DDR     |= RED_LED_MASK;
    YELLOW_LED_DDR  |= YELLOW_LED_MASK;
    BLUE_LED_DDR    |= BLUE_LED_MASK;

    led = 0;

    while( 1 )
    {
        // Turn all of the LEDs off

        LED_OFF( RED );
        LED_OFF( BLUE );
        LED_OFF( YELLOW );

        switch ( led )
        {
            case    0:  LED_ON( RED );      break;
            case    1:  LED_ON( BLUE );     break;
            case    2:  LED_ON( YELLOW );   break;
        }

        if ( ++led > 2 )
        {
            led = 0;
        }

        {
            int i;

            // Tick rate is 100/sec so waiting for 100 waits for 1 sec

            for ( i = 0; i < 100; i++ ) 
            {
                    WaitForTimer0Rollover();
            }
        }
    }

    return 0;
}
