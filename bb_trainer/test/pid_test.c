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
  int16_t output;
  int16_t num_iterations = 250;
  int16_t num_steps = 10;
  int16_t starting_setpoint = 1;
  int16_t ending_setpoint = 12;
  int16_t target_setpoint = 6;
  int16_t max_output = 255;
  int16_t min_output = -255;

  Pid_State_t pid_state;

  memset(&pid_state, 0, sizeof(pid_state));

  pid_state.kp = 5.0;
  pid_state.kd = 0.0;
  pid_state.ki = 1.0;

  pid_state.max_output = max_output;
  pid_state.min_output = min_output;

  int16_t input = starting_setpoint;
  int16_t step_size = (ending_setpoint - starting_setpoint) / num_steps;
  int16_t step_length = num_iterations / num_steps;

  pid_state.setpoint = target_setpoint;

  for ( int i = 0; i < num_iterations; i++)
  {
    output = Pid_Compute_Output( input, &pid_state );
    printf( "Input: %d, output: %d\n", input, output );

    input = starting_setpoint + ( step_size * (i / step_length) );

    ASSERT_GE(output, min_output);
    ASSERT_LE(output, max_output);
  }

  EXPECT_EQ( 1, 1 );
}
