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

#include "motion_control.h"

TEST(MotionControlTest, staticData )
{
  Motion_Control_Init();

  EXPECT_EQ( 1, 1 );
}
