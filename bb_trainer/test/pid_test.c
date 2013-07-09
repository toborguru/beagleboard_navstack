/** @file checksum_test.c
 *  This program will run a unit test on the checksum module.
 *
 *  Created:    03/10/2011 12:27:19 AM
 *  Compiler:   gcc
 *  @author:    Sawyer Larkin (SJL), DATAProject@therobotguy.com
 *
 *  Last Changed:   $id$
 *
 */

#include <inttypes.h>
#include <cstdio>
#include <gtest/gtest.h>

#include "pid.h"

TEST(PidTest, staticData )
{
  int16_t input = 0;
  int16_t output;

  Pid_State_t pid_state;

  pid_state.kp = 5.0;
  pid_state.kd = 0.0;
  pid_state.ki = 1.0;

  pid_state.setpoint = 6;

  for ( int_fast8_t i = 0; i < 100; i++)
  {
    output = Pid_Compute_Output( input, &pid_state );
    printf( "Input: %d, output: %d\n", input, output );

    input = output / 20;
  }

  EXPECT_EQ( 1, 1 );
}
