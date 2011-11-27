/************************************************
  Sawyer Larkin
  D.A.T.A.

  ShaftEncoders.c
  Routines for monitoring the shaft encoders
 *************************************************/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "../Avr.h"
#include "../macros.h"

#include "motors.h"
#include "shaft_encoders.h"

#define STATIS_WHEEL_EN 0

#define ENCODERS_LEFT_TICKS_PORT    D
#define ENCODERS_LEFT_TICKS_PIN     3
#define ENCODERS_LEFT_DIR_PORT      D
#define ENCODERS_LEFT_DIR_PIN       5
#define ENCODERS_LEFT_INTERRUPT     INT1
#define ENCODERS_LEFT_INT_VECT      INT1_vect

#define ENCODERS_RIGHT_TICKS_PORT   D
#define ENCODERS_RIGHT_TICKS_PIN    2
#define ENCODERS_RIGHT_DIR_PORT     D
#define ENCODERS_RIGHT_DIR_PIN      4
#define ENCODERS_RIGHT_INTERRUPT    INT0
#define ENCODERS_RIGHT_INT_VECT     INT0_vect

#define ENCODERS_EICRA_MASK         BIT( ISC00 ) | BIT( ISC10 ) 

#if ENCODERS_STATIS_WHEEL_EN
#define STASIS_EICRB_MASK       BIT( ISC50 ) | BIT( ISC51 )
#endif
     
#if STATIS_WHEEL_EN
#define ENCODERS_STASIS_TICKS_PORT  E
#define ENCODERS_STASIS_TICKS_PIN   5
#define ENCODERS_STASIS_DIR_PORT    A
#define ENCODERS_STASIS_DIR_PIN     7
#endif

#define ENCODERS_RISING_EDGE    0
#define ENCODERS_FALLING_EDGE   1

/* Global variable definitions */
volatile int16_t g_shaft_encoders_left_count = 0;
volatile int16_t g_shaft_encoders_right_count = 0;
volatile int16_t g_shaft_encoders_stasis_count = 0;

/** Initializes the AVR
*/
void Shaft_Encoders_Init( void  )
{
    // Set pins as input
    INPUT_PIN( ENCODERS_LEFT_TICKS_PORT, ENCODERS_LEFT_TICKS_PIN );
    INPUT_PIN( ENCODERS_LEFT_DIR_PORT, ENCODERS_LEFT_DIR_PIN );

    INPUT_PIN( ENCODERS_RIGHT_TICKS_PORT, ENCODERS_RIGHT_TICKS_PIN );
    INPUT_PIN( ENCODERS_RIGHT_DIR_PORT, ENCODERS_RIGHT_DIR_PIN );

#if STATIS_WHEEL_EN
    INPUT_PIN( ENCODERS_STASIS_TICKS_PORT, ENCODERS_STASIS_TICKS_PIN );
    INPUT_PIN( ENCODERS_STASIS_DIR_PORT, ENCODERS_STASIS_DIR_PIN );
#endif

    // Disable pull-up
    DISABLE_PULLUP( ENCODERS_LEFT_TICKS_PORT, ENCODERS_LEFT_TICKS_PIN );
    DISABLE_PULLUP( ENCODERS_RIGHT_TICKS_PORT, ENCODERS_RIGHT_TICKS_PIN );

    DISABLE_PULLUP( ENCODERS_LEFT_DIR_PORT, ENCODERS_LEFT_DIR_PIN );
    DISABLE_PULLUP( ENCODERS_RIGHT_DIR_PORT, ENCODERS_RIGHT_DIR_PIN );

    // Set interupts as rising edge triggered
    EICRA |= ENCODERS_EICRA_MASK;


#if STATIS_WHEEL_EN
    EICRB |= STASIS_EICRB_MASK;  
#endif

    // Enable the interupts - Interrupts should be globally disabled
    EIMSK |= BIT( ENCODERS_LEFT_INTERRUPT ) | BIT( ENCODERS_RIGHT_INTERRUPT );

#if STATIS_WHEEL_EN
    EIMSK |= BIT ( STASIS_WHEEL_INTERRUPT );
#endif
}
 
ISR( ENCODERS_LEFT_INT_VECT )
{ 
    if ( READ_PIN( ENCODERS_LEFT_DIR_PORT, ENCODERS_LEFT_DIR_PIN ) )
    {
        --g_shaft_encoders_left_count;
    }
    else
    {
        ++g_shaft_encoders_left_count;
    }
}

ISR( ENCODERS_RIGHT_INT_VECT )
{
    if ( READ_PIN( ENCODERS_RIGHT_DIR_PORT, ENCODERS_RIGHT_DIR_PIN ) )
    {
        ++g_shaft_encoders_right_count;
    }
    else
    {
        --g_shaft_encoders_right_count;
    }
}  

#if STATIS_WHEEL_EN
ISR( INT5_vect )
{
    uint8_t eimsk, eicrb;

    eicrb = EICRB;

    if ( BIT_TEST( eicrb, ISC50 ) )  // Just caught the rising edge
    {
        if ( READ_PIN( ENCODERS_STASIS_DIR_PORT, ENCODERS_STASIS_DIR_PIN ) )
        {
            --g_shaft_encoders_stasis_count;
        }
        else
        {
            ++g_shaft_encoders_stasis_count;
        }

        eimsk = EIMSK;
        EIMSK = BIT_CLEAR( eimsk, INT5 );   // Disable the interrupt to clear it
        EICRB = BIT_CLEAR( eicrb , ISC50 ); // Set to falling edge
        EIMSK = BIT_SET( eimsk , INT5 );    // Enable the interrupt
    }
    else // Just caught rising edge
    {
        if ( _READ_PIN( ENCODERS_STASIS_DIR_PORT, ENCODERS_STASIS_DIR_PIN ) )
        {
            ++g_shaft_encoders_stasis_count;
        }
        else
        {
            --g_shaft_encoders_stasis_count;
        }

        eimsk = EIMSK;
        EIMSK = BIT_CLEAR( eimsk, INT5 );   // Disable the interrupt to clear it
        EICRB = BIT_SET( eicrb , ISC50 );   // Set to rising edge
        EIMSK = BIT_SET( eimsk , INT5 );    // Enable the interrupt
    }
}  
#endif
