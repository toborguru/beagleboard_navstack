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

// ---- Include Files -------------------------------------------------------

#include <avr/io.h>
#if defined( __AVR_LIBC_VERSION__ )
#   include <avr/interrupt.h>
#else
#include <avr/signal.h>
#endif
#include <inttypes.h>
#include <stdio.h>

#include "Tach.h"
#include "Hardware.h"

// ---- Public Variables ----------------------------------------------------

volatile    TACH_Buffer_t   TACH_gBuffer;

// ---- Private Constants and Types -----------------------------------------

typedef struct
{
    union
    {
        uint32_t    count32;

        struct
        {
            uint16_t    countLow16;
            uint16_t    countHigh16;

        };
    };

} TimerCount_t;

// ---- Private Variables ---------------------------------------------------

static  uint16_t    gTimer3High16;

// ---- Private Function Prototypes -----------------------------------------
// ---- Functions -----------------------------------------------------------

/****************************************************************************
*
*   Interrupt handler for Timer3 overflow.
*
*   This allows us to extend the timer to become a 32 bit timer.
*
****************************************************************************/

SIGNAL( SIG_OVERFLOW3 )
{
    gTimer3High16++;

} // SIG_OVERFLOW3

/****************************************************************************
*
*   Interrupt handler for the Timer3 input capture.
*
*   This interrupt fires whenever a rising edge (or falling depending on
*   the ICES3 bit in TCCR3B) is detected.
*
****************************************************************************/

SIGNAL( SIG_INPUT_CAPTURE3 )
{
    TimerCount_t    t;
    
    // Read the captured time value. Must read the low byte first

    t.countLow16 = TCNT3;
    t.countHigh16 = gTimer3High16;

    LED_TOGGLE( BLUE );

    // Stash it in the circular buffer

    if ( CBUF_Len( TACH_gBuffer ) >= TACH_NUM_ENTRIES )
    {
        CBUF_Pop( TACH_gBuffer );
    }
    CBUF_Push( TACH_gBuffer, t.count32 );

} // SIG_INPUT_CAPTURE3

