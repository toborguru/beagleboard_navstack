#include <gtest/gtest.h>

#include "BumpersProcessor.hpp"

namespace data_robot_test_core
{
// Define the unit test to verify ability to differentiate around roll-overs
TEST(BumpersProcessorTests, canSetAndClearBumps)
{
  uint8_t new_bumps;
  uint8_t bumps_mask;
  
  uint8_t bumps1;
  uint8_t bumps2;
  uint8_t bumps3;
  uint8_t bumps4;
  uint8_t bumps5;
  uint8_t bumps6;
  uint8_t bumps7;

  data_robot_core::BumpersProcessor bumpers;

  // Establish Context

  // Act
  bumps1 = bumpers.GetCurrentBumps();

  // Assert

  EXPECT_EQ( 0, bumps1 );
}

}
