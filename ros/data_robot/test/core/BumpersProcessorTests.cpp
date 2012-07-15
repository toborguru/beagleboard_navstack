#include <gtest/gtest.h>

#include "BumpersProcessor.hpp"

namespace data_robot_test_core
{
// Define the unit test to verify ability to set and clear the bumper bit-field
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
  uint8_t bumps8;

  data_robot_core::BumpersProcessor bumpers;

  // Establish Context

  // Act
  bumps1 = bumpers.GetCurrentBumps();

  // All new data
  bumps_mask = 0xFF;
  new_bumps = 0xFF;
  bumpers.AddNewData( bumps_mask, new_bumps );
  bumps2 = bumpers.GetCurrentBumps();

  new_bumps = 0xAA;
  bumpers.AddNewData( bumps_mask, new_bumps );
  bumps3 = bumpers.GetCurrentBumps();

  // Masked Data
  bumps_mask = 0x0F;
  new_bumps = 0xFF;
  bumpers.AddNewData( bumps_mask, new_bumps );
  bumps4 = bumpers.GetCurrentBumps();

  bumps_mask = 0xF0;
  new_bumps = 0x00;
  bumpers.AddNewData( bumps_mask, new_bumps );
  bumps5 = bumpers.GetCurrentBumps();

  // Getting silly
  bumps_mask = 0xF5;
  new_bumps = 0xA0;
  bumpers.AddNewData( bumps_mask, new_bumps );
  bumps6 = bumpers.GetCurrentBumps();

  bumps_mask = 0xAA;
  new_bumps = 0xFF;
  bumpers.AddNewData( bumps_mask, new_bumps );
  bumps7 = bumpers.GetCurrentBumps();

  bumps_mask = 0xAA;
  new_bumps = 0x55;
  bumpers.AddNewData( bumps_mask, new_bumps );
  bumps8 = bumpers.GetCurrentBumps();

  // Assert

  EXPECT_EQ( 0,     bumps1 );
  EXPECT_EQ( 0xFF,  bumps2 );
  EXPECT_EQ( 0xAA,  bumps3 );
  EXPECT_EQ( 0xAF,  bumps4 );
  EXPECT_EQ( 0x0F,  bumps5 );
  EXPECT_EQ( 0xAA,  bumps6 );
  EXPECT_EQ( 0xAA,  bumps7 );
  EXPECT_EQ( 0,     bumps8 );
}

// Define the unit test to verify ability to set and clear the bumper bit-field
TEST(BumpersProcessorTests, canSetBumperMaskesAndCalculateDirection)
{
  uint8_t new_bumps;
  uint8_t bumps_mask;

  uint32_t mask1;

  uint8_t dir1; 
  uint8_t dir2; 
  uint8_t dir3; 
  uint8_t dir4; 
  uint8_t dir5; 
  uint8_t dir6; 
  uint8_t dir7; 
  uint8_t dir8; 
  uint8_t dir9; 
  uint8_t dir10;
  uint8_t dir11; 
  uint8_t dir12; 
  uint8_t dir13; 
  uint8_t dir14; 
  uint8_t dir15; 
  uint8_t dir16; 
  uint8_t dir17; 
  uint8_t dir18; 
  
  data_robot_core::BumpersProcessor bumpers;

  // Establish Context
  new_bumps = 0xFF;
  bumps_mask = 0xFF;
  bumpers.AddNewData( bumps_mask, new_bumps );

  // Act
  // No bumpers defined
  dir1 = bumpers.GetBumpDirection();

  // Mask 0=>0,  Mask 1=>1
  new_bumps = 0x01;
  bumps_mask = 0xFF;
  bumpers.AddNewData( bumps_mask, new_bumps );

  bumpers.AddFrontBumperIndex(0);
  mask1 = bumpers.GetFrontBumperMask();

  // Set and change front bumper
  bumpers.SetFrontBumperMask(1);
  dir2 = bumpers.GetBumpDirection();

  bumpers.SetFrontBumperMask(2);
  dir3 = bumpers.GetBumpDirection();

  // Set and test front and left
  bumpers.SetFrontBumperMask(0xFF);
  bumpers.SetFrontLeftBumperMask(0);
  dir4 = bumpers.GetBumpDirection();

  bumpers.SetFrontBumperMask(0xFF00);
  bumpers.SetFrontLeftBumperMask(1);
  dir5 = bumpers.GetBumpDirection();

  bumpers.SetFrontBumperMask(1);
  bumpers.SetFrontLeftBumperMask(1);
  dir6 = bumpers.GetBumpDirection();

  // Set and test front and right
  bumpers.SetFrontBumperMask(1);
  bumpers.SetFrontLeftBumperMask(0);
  bumpers.SetFrontRightBumperMask(0);
  dir7 = bumpers.GetBumpDirection();

  bumpers.SetFrontBumperMask(0);
  bumpers.SetFrontRightBumperMask(1);
  dir8 = bumpers.GetBumpDirection();

  bumpers.SetFrontBumperMask(1);
  bumpers.SetFrontRightBumperMask(1);
  dir9 = bumpers.GetBumpDirection();

  // Set and test all front bumpers
  bumpers.SetFrontBumperMask(1);
  bumpers.SetFrontLeftBumperMask(1);
  bumpers.SetFrontRightBumperMask(1);
  dir10 = bumpers.GetBumpDirection();

  bumpers.SetFrontBumperMask(0);
  bumpers.SetFrontLeftBumperMask(1);
  bumpers.SetFrontRightBumperMask(1);
  dir11 = bumpers.GetBumpDirection();

  bumpers.SetFrontBumperMask(0);
  bumpers.SetFrontLeftBumperMask(1);
  bumpers.SetFrontRightBumperMask(0);
  dir12 = bumpers.GetBumpDirection();

  bumpers.SetFrontBumperMask(0);
  bumpers.SetFrontLeftBumperMask(0);
  bumpers.SetFrontRightBumperMask(1);
  dir13 = bumpers.GetBumpDirection();

  bumpers.SetFrontBumperMask(0);
  bumpers.SetFrontLeftBumperMask(0);
  bumpers.SetFrontRightBumperMask(0);
  dir14 = bumpers.GetBumpDirection();

  // Test the rear bumpers
  bumpers.SetRearBumperMask(1);
  bumpers.SetRearLeftBumperMask(0);
  bumpers.SetRearRightBumperMask(0);
  dir15 = bumpers.GetBumpDirection();

  bumpers.SetRearBumperMask(0);
  bumpers.SetRearLeftBumperMask(1);
  bumpers.SetRearRightBumperMask(0);
  dir16 = bumpers.GetBumpDirection();

  bumpers.SetRearBumperMask(0);
  bumpers.SetRearLeftBumperMask(0);
  bumpers.SetRearRightBumperMask(1);
  dir17 = bumpers.GetBumpDirection();

  bumpers.SetFrontBumperMask(1);
  dir18 = bumpers.GetBumpDirection();

  // Assert

  EXPECT_EQ( 1, mask1);
  EXPECT_EQ( data_robot::Bumpers::NONE,         dir1 );
  EXPECT_EQ( data_robot::Bumpers::FRONT,        dir2 );
  EXPECT_EQ( data_robot::Bumpers::NONE,         dir3 );
  EXPECT_EQ( data_robot::Bumpers::FRONT,        dir4 );
  EXPECT_EQ( data_robot::Bumpers::FRONT_LEFT,   dir5 );
  EXPECT_EQ( data_robot::Bumpers::FRONT,        dir6 );
  EXPECT_EQ( data_robot::Bumpers::FRONT,        dir7 );
  EXPECT_EQ( data_robot::Bumpers::FRONT_RIGHT,  dir8 );
  EXPECT_EQ( data_robot::Bumpers::FRONT,        dir9 );
  EXPECT_EQ( data_robot::Bumpers::FRONT,        dir11 );
  EXPECT_EQ( data_robot::Bumpers::FRONT_LEFT,   dir12 );
  EXPECT_EQ( data_robot::Bumpers::FRONT_RIGHT,  dir13 );
  EXPECT_EQ( data_robot::Bumpers::NONE,         dir14 );
  EXPECT_EQ( data_robot::Bumpers::REAR,         dir15 );
  EXPECT_EQ( data_robot::Bumpers::REAR_LEFT,    dir16 );
  EXPECT_EQ( data_robot::Bumpers::REAR_RIGHT,   dir17 );
  EXPECT_EQ( data_robot::Bumpers::WEDGED,       dir18 );
}

}
