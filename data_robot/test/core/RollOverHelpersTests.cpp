#include <gtest/gtest.h>

extern "C"
{
  #include "RollOverHelpers.h"
}

namespace data_robot_test_core
{
// Define the unit test to verify ability to differentiate around roll-overs
TEST(RollOverHelperTests, canDifferentiateSignedValues)
{
  int32_t i32_old_value;
  int32_t i32_new_value;
  int32_t i32diff1;
  int32_t i32diff2;
  int32_t i32diff3;
  int32_t i32diff4;
  int32_t i32diff5;
 
  int16_t i16_old_value;
  int16_t i16_new_value;
  int16_t i16diff1;
  int16_t i16diff2;
  int16_t i16diff3;
  int16_t i16diff4;
  int16_t i16diff5;
 
  int8_t i8_old_value;
  int8_t i8_new_value;
  int8_t i8diff1;
  int8_t i8diff2;
  int8_t i8diff3;
  int8_t i8diff4;
  int8_t i8diff5;
 
  // Establish Context

  // Act
  i8_old_value = 0;
  i8_new_value = 45;
  i8diff1 = DifferentiateInt8RollOver( i8_old_value, i8_new_value );

  i8_old_value = 0;
  i8_new_value = -45;
  i8diff2 = DifferentiateInt8RollOver( i8_old_value, i8_new_value );

  i8_old_value = 45;
  i8_new_value = -45;
  i8diff3 = DifferentiateInt8RollOver( i8_old_value, i8_new_value );

  i8_old_value = 120;
  i8_new_value = 120 + 20;
  i8diff4 = DifferentiateInt8RollOver( i8_old_value, i8_new_value );

  i8_old_value = -120;
  i8_new_value = -120 - 20;
  i8diff5 = DifferentiateInt8RollOver( i8_old_value, i8_new_value );

  // -----------------------------
  i16_old_value = 0;
  i16_new_value = 3E4;
  i16diff1 = DifferentiateInt16RollOver( i16_old_value, i16_new_value );

  i16_old_value = 0;
  i16_new_value = -3E4;
  i16diff2 = DifferentiateInt16RollOver( i16_old_value, i16_new_value );

  i16_old_value = -1000;
  i16_new_value = 1000;
  i16diff3 = DifferentiateInt16RollOver( i16_old_value, i16_new_value );

  i16_old_value = 3E4;
  i16_new_value =(int32_t) 3E4 + 5000;
  i16diff4 = DifferentiateInt16RollOver( i16_old_value, i16_new_value );

  i16_old_value = -3E4;
  i16_new_value = (int32_t) -3E4 - 5000;
  i16diff5 = DifferentiateInt16RollOver( i16_old_value, i16_new_value );

  // ------------------------------
  i32_old_value = 0;
  i32_new_value = 2E9;
  i32diff1 = DifferentiateInt32RollOver( i32_old_value, i32_new_value );

  i32_old_value = 0;
  i32_new_value = -2E9;
  i32diff2 = DifferentiateInt32RollOver( i32_old_value, i32_new_value );

  i32_old_value = -2E8;
  i32_new_value = 2E8;
  i32diff3 = DifferentiateInt32RollOver( i32_old_value, i32_new_value );

  i32_old_value = 2E9;
  i32_new_value = (long) 2E9 + (long)3E8;
  i32diff4 = DifferentiateInt32RollOver( i32_old_value, i32_new_value );

  i32_old_value = -2E9;
  i32_new_value = (long) -2E9 - (long)3E8;
  i32diff5 = DifferentiateInt32RollOver( i32_old_value, i32_new_value );

  // Assert

  EXPECT_EQ( 45, (int)i8diff1 );
  EXPECT_EQ( -45, (int)i8diff2 );
  EXPECT_EQ( -90, (int)i8diff3 );
  EXPECT_EQ( 20, (int)i8diff4 );
  EXPECT_EQ( -20, (int)i8diff5 );

  EXPECT_EQ( 3E4, i16diff1 );
  EXPECT_EQ( -3E4, i16diff2 );
  EXPECT_EQ( 2000, i16diff3 );
  EXPECT_EQ( 5000, i16diff4 );
  EXPECT_EQ( -5000, i16diff5 );

  EXPECT_EQ( 2E9, i32diff1 );
  EXPECT_EQ( -2E9, i32diff2 );
  EXPECT_EQ( 4E8, i32diff3 );
  EXPECT_EQ( 3E8, i32diff4 );
  EXPECT_EQ( -3E8, i32diff5 );
}

// Define the unit test to verify ability to differentiate around roll-overs
TEST(RollOverHelperTests, canDifferentiateUnsignedValues)
{
  uint32_t ui32_old_value;
  uint32_t ui32_new_value;
  uint32_t ui32diff1;
  uint32_t ui32diff2;
 
  uint16_t ui16_old_value;
  uint16_t ui16_new_value;
  uint16_t ui16diff1;
  uint16_t ui16diff2;
 
  uint8_t ui8_old_value;
  uint8_t ui8_new_value;
  uint8_t ui8diff1;
  uint8_t ui8diff2;
  uint8_t ui8diff3;
 
  // Establish Context

  // Act
  ui8_old_value = 0;
  ui8_new_value = 200;
  ui8diff1 = DifferentiateInt8RollOver( ui8_old_value, ui8_new_value );

  ui8_old_value = 250;
  ui8_new_value = 250 + 20;
  ui8diff2 = DifferentiateInt8RollOver( ui8_old_value, ui8_new_value );

  ui8_old_value = 200;
  ui8_new_value = 200 + 100;
  ui8diff3 = DifferentiateInt8RollOver( ui8_old_value, ui8_new_value );

  // -----------------------------
  ui16_old_value = 0;
  ui16_new_value = 6E4;
  ui16diff1 = DifferentiateInt16RollOver( ui16_old_value, ui16_new_value );

  ui16_old_value = 6E4;
  ui16_new_value = (uint32_t)6E4 + (uint32_t)1E4;
  ui16diff2 = DifferentiateInt16RollOver( ui16_old_value, ui16_new_value );

  // ------------------------------
  ui32_old_value = 0;
  ui32_new_value = 4E9;
  ui32diff1 = DifferentiateInt32RollOver( ui32_old_value, ui32_new_value );

  ui32_old_value = 4E9;
  ui32_new_value = (unsigned long) 4E9 + (unsigned long)1E9;
  ui32diff2 = DifferentiateInt32RollOver( ui32_old_value, ui32_new_value );

  // Assert

  EXPECT_EQ( 200, (int)ui8diff1 );
  EXPECT_EQ( 20, (int)ui8diff2 );
  EXPECT_EQ( 100, (int)ui8diff3 );

  EXPECT_EQ( 6E4, ui16diff1 );
  EXPECT_EQ( 1E4, ui16diff2 );

  EXPECT_EQ( 4E9, ui32diff1 );
  EXPECT_EQ( 1E9, ui32diff2 );
}
}
