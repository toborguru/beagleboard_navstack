/************************************************
  Sawyer Larkin
  D.A.T.A.

  Routines for monitoring the shaft encoders
************************************************/

#ifndef __SHAFT_ENCODERS_H
#define __SHAFT_ENCODERS_H

extern volatile int16_t g_shaft_encoders_left_count;
extern volatile int16_t g_shaft_encoders_right_count;
extern volatile int16_t g_shaft_encoders_stasis_count;

// Initialize shaft encoder hardware
void Shaft_Encoders_Init( void );

#endif
