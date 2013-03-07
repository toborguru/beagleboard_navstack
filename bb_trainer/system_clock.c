#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "macros.h"

#include "system_clock.h"

#define SYSTEM_CLOCK_RATE_HZ    1000

volatile SYSTEM_CLOCK_T g_system_clock = 0;

/*
 * Setup timer 2 compare match to generate a tick interrupt.
 */
void System_Clock_Init( void )
{
    uint8_t compare_match;
    uint8_t byte;

	compare_match = 124; // 1000 Hz

	/* Setup compare match value for compare match A.  Interrupts are disabled 
	before this is called so we need not worry here. */
	byte = ( unsigned char ) ( compare_match & ( unsigned long ) 0xff );
	OCR2A = byte;

	/* Setup clock source and compare match behaviour. */
	TCCR2A = _BV(WGM21); // CTC
    TCCR2B = _BV(CS22);  // 64 Prescale
    BIT_CLEAR ( PRR, PRTIM2 );

	/* Enable the interrupt - this is okay as interrupt are currently globally
	disabled. */
	byte = TIMSK2;
	byte |= _BV(OCIE2A); // Enable Timer Interupt
	TIMSK2 = byte;
}

/* This function returns the value of time1-time2 corrected for roll over at
 * SYSTEM_CLOCK_MAX. Differences over SYSTEM_CLOCK_MAX/2 are considered roll-over.
 */
int32_t Clock_Diff(SYSTEM_CLOCK_T time1, SYSTEM_CLOCK_T time2)
{
    int32_t time_diff;

    time_diff = (int32_t)time1 - (int32_t)time2;

    if ( time_diff > (SYSTEM_CLOCK_MAX / 2) )
    {
        time_diff -= SYSTEM_CLOCK_MAX;
    }
    else if ( time_diff < (-1 * SYSTEM_CLOCK_MAX / 2) )
    {
        time_diff += SYSTEM_CLOCK_MAX;
    }

    return time_diff;
}

ISR( TIMER2_COMPA_vect )
{
    ++g_system_clock;

    if ( g_system_clock >= SYSTEM_CLOCK_MAX )
    {
      g_system_clock = 0;
    }
}

