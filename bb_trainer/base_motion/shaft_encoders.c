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

#include "../util/QD.h"

#include "motors.h"
#include "shaft_encoders.h"

#define ENCODERS_STASIS_WHEEL_ENABLE  1

#define ENCODERS_LEFT_TICKS_PORT    D
#define ENCODERS_LEFT_TICKS_PIN     2
#define ENCODERS_LEFT_DIR_PORT      D
#define ENCODERS_LEFT_DIR_PIN       4
#define ENCODERS_LEFT_INTERRUPT     INT0
#define ENCODERS_LEFT_INT_VECT      INT0_vect

#define ENCODERS_RIGHT_TICKS_PORT   D
#define ENCODERS_RIGHT_TICKS_PIN    3
#define ENCODERS_RIGHT_DIR_PORT     D
#define ENCODERS_RIGHT_DIR_PIN      5
#define ENCODERS_RIGHT_INTERRUPT    INT1
#define ENCODERS_RIGHT_INT_VECT     INT1_vect
#define ENCODERS_EICRA_MASK         BIT( ISC00 ) | BIT( ISC10 ) 

#if ENCODERS_STASIS_WHEEL_ENABLE
#define ENCODERS_STASIS_A_PORT  D
#define ENCODERS_STASIS_A_PIN   6
#define ENCODERS_STASIS_B_PORT  D
#define ENCODERS_STASIS_B_PIN   7
#define ENCODERS_STASIS_A_INTERRUPT   PCINT22
#define ENCODERS_STASIS_B_INTERRUPT   PCINT23
#define ENCODERS_STASIS_INT_VECT    PCINT2_vect
#define ENCODERS_STASIS_PCICR_MASK  BIT( PCIE2 )
#endif

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

#if ENCODERS_STASIS_WHEEL_ENABLE
    INPUT_PIN( ENCODERS_STASIS_A_PORT, ENCODERS_STASIS_A_PIN );
    INPUT_PIN( ENCODERS_STASIS_B_PORT, ENCODERS_STASIS_B_PIN );
#endif

    // Disable pull-up
    DISABLE_PULLUP( ENCODERS_LEFT_TICKS_PORT, ENCODERS_LEFT_TICKS_PIN );
    DISABLE_PULLUP( ENCODERS_RIGHT_TICKS_PORT, ENCODERS_RIGHT_TICKS_PIN );

    DISABLE_PULLUP( ENCODERS_LEFT_DIR_PORT, ENCODERS_LEFT_DIR_PIN );
    DISABLE_PULLUP( ENCODERS_RIGHT_DIR_PORT, ENCODERS_RIGHT_DIR_PIN );

#if ENCODERS_STASIS_WHEEL_ENABLE
    DISABLE_PULLUP( ENCODERS_STASIS_A_PORT, ENCODERS_STASIS_A_PIN );
    DISABLE_PULLUP( ENCODERS_STASIS_B_PORT, ENCODERS_STASIS_B_PIN );
#endif

    // Set interupts trigger
    EICRA |= ENCODERS_EICRA_MASK;

    // Enable the interupts - Interrupts should be globally disabled
    EIMSK |= BIT( ENCODERS_LEFT_INTERRUPT ) | BIT( ENCODERS_RIGHT_INTERRUPT );

    // Setup stasis wheel interrupts
#if ENCODERS_STASIS_WHEEL_ENABLE
    PCMSK2 = BIT( ENCODERS_STASIS_A_INTERRUPT ) | BIT( ENCODERS_STASIS_B_INTERRUPT );

    // Enable the interrupt
    PCICR |= ENCODERS_STASIS_PCICR_MASK;  
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

#if ENCODERS_STASIS_WHEEL_ENABLE
ISR( ENCODERS_STASIS_INT_VECT )
{
  static uint_fast8_t prev_state = 0;
  uint_fast8_t state;

  state = READ_PIN( ENCODERS_STASIS_A_PORT, ENCODERS_STASIS_A_PIN ) << 1 
          | READ_PIN( ENCODERS_STASIS_B_PORT, ENCODERS_STASIS_B_PIN );

  g_shaft_encoders_stasis_count += QD_gIncrement[ prev_state ].increment[ state ];
  prev_state = state; 
}  
#endif
