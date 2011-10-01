/* Unit test for EncoderCountsReader class.
 *
 * @author Sawyer Larkin (SJL toborguru)
 */

#include <gtest/gtest.h>

#include "diff_drive/EncoderCounts.h"

#include "BaseModel.hpp"
 
using namespace diff_drive_core;
 
namespace diff_drive_core_test_core
{
// Define the unit test to verify Base Model calculated geometry properties
TEST( BaseModelTests, canCalculateGeometryProperties )
{
  // Establish Context
  BaseModel base_model;

  // Act
  base_model.SetWheelRadius( 0.5 / M_PI );
  base_model.SetWheelTicks( 100 );
  base_model.SetWheelBase( 0.5 );

  // Assert
  ASSERT_FLOAT_EQ( 100.0, base_model.GetTicksPerMeter() );
  ASSERT_FLOAT_EQ( 50.0, base_model.GetTicksPerRadian() );
  ASSERT_FLOAT_EQ( 0.0, base_model.GetStasisTicksPerMeter() );

  // Act
  base_model.SetStasisTicks( 10 );
  base_model.SetStasisRadius( 0.5 / M_PI );

  // Assert
  ASSERT_FLOAT_EQ( 10.0, base_model.GetStasisTicksPerMeter() );
}

// Define the unit test to verify Base Model calculated dead reckoning
TEST( BaseModelTests, canCalculateUncalibratedDeadReckoning )
{
  // Establish Context
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );
  diff_drive::EncoderCounts counts; 

  // Act
  counts.left_count = 100;
  counts.right_count = 100;
  counts.dt_ms = 100;

  base_model.NewEncoderCounts( counts );

  // Assert
  ASSERT_FLOAT_EQ( 1.0, base_model.GetDeltaX() );
  ASSERT_FLOAT_EQ( 0.0, base_model.GetDeltaY() );
  ASSERT_FLOAT_EQ( 0.0, base_model.GetDeltaTheta() );
  ASSERT_FLOAT_EQ( 10.0, base_model.GetLinearVelocity() );
  ASSERT_FLOAT_EQ( 0.0, base_model.GetAngularVelocity() );

  // Act
  counts.left_count = -25;
  counts.right_count = 25;
  counts.dt_ms = 100;

  base_model.NewEncoderCounts( counts );

  // Assert
  ASSERT_FLOAT_EQ( 0.0, base_model.GetDeltaX() );
  ASSERT_FLOAT_EQ( 0.0, base_model.GetDeltaY() );
  ASSERT_FLOAT_EQ( 1.0, base_model.GetDeltaTheta() );
  ASSERT_FLOAT_EQ( 0.0, base_model.GetLinearVelocity() );
  ASSERT_FLOAT_EQ( 10.0, base_model.GetAngularVelocity() );

  // Act
  counts.left_count = 75;
  counts.right_count = 125;
  counts.dt_ms = 100;

  base_model.NewEncoderCounts( counts );

  // Assert
  ASSERT_FLOAT_EQ( 1.0, base_model.GetDeltaTheta() );
  ASSERT_FLOAT_EQ( 10.0, base_model.GetLinearVelocity() );
  ASSERT_FLOAT_EQ( 10.0, base_model.GetAngularVelocity() );

  // Act
  counts.left_count = 25;
  counts.right_count = -25;
  counts.dt_ms = 100;

  base_model.NewEncoderCounts( counts );

  // Assert
  ASSERT_FLOAT_EQ( 0.0, base_model.GetDeltaX() );
  ASSERT_FLOAT_EQ( 0.0, base_model.GetDeltaY() );
  ASSERT_FLOAT_EQ( -1.0, base_model.GetDeltaTheta() );
  ASSERT_FLOAT_EQ( 0.0, base_model.GetLinearVelocity() );
  ASSERT_FLOAT_EQ( -10.0, base_model.GetAngularVelocity() );

  // Act
  counts.left_count = -75;
  counts.right_count = -125;
  counts.dt_ms = 100;

  base_model.NewEncoderCounts( counts );

  // Assert
  ASSERT_FLOAT_EQ( -1.0, base_model.GetDeltaTheta() );
  ASSERT_FLOAT_EQ( -10.0, base_model.GetLinearVelocity() );
  ASSERT_FLOAT_EQ( -10.0, base_model.GetAngularVelocity() );

  // Act
  counts.left_count = 205;
  counts.right_count = 195;
  counts.dt_ms = 100;

  base_model.NewEncoderCounts( counts );
}

// Define the unit test to verify Base Model calibrated dead reckoning
TEST( BaseModelTests, canCalculateCalibratedDeadReckoning )
{
  // Establish Context
  BaseModel base_model( 0.5 / M_PI, 100, 0.5, 0.95);
  diff_drive::EncoderCounts counts; 

  // Act
  counts.left_count = 205;
  counts.right_count = 195;
  counts.dt_ms = 100;

  base_model.NewEncoderCounts( counts );

#if 0
  std::cout << "X: " << base_model.GetDeltaX()
            << " Y: " << base_model.GetDeltaY()
            << " Theta: " << base_model.GetDeltaTheta()
            << std::endl;
#endif

  // Assert
  ASSERT_NEAR( 2.0, base_model.GetDeltaX(), 0.01 );
  ASSERT_NEAR( 0.0, base_model.GetDeltaY(), 0.01 );
  ASSERT_NEAR(0.0, base_model.GetDeltaTheta(), 0.01 );
}

// Define the unit test to verify Base Model tick velocities
TEST( BaseModelTests, canCalculateTickVelocities )
{
  // Establish Context
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );
  diff_drive::TickVelocity ticks; 

  // Act
  ticks = base_model.VelocityToTicks( 1.0, 0.0 );

  // Assert
  ASSERT_EQ( 100, ticks.linear_ticks_sec );
  ASSERT_EQ( 0.0, ticks.angular_ticks_sec );

  // Act
  ticks = base_model.VelocityToTicks( 0.0, 1.0 );

  // Assert
  ASSERT_EQ( 0, ticks.linear_ticks_sec );
  ASSERT_EQ( 50, ticks.angular_ticks_sec );
}

// Define the unit test to verify Base Model tick velocities
TEST( BaseModelTests, canCalculateCalibratedTickVelocities )
{
  // Establish Context
  BaseModel base_model( 0.5 / M_PI, 100, 0.5, 0.95 );
  diff_drive::TickVelocity ticks; 

  // Act
  ticks = base_model.VelocityToTicks( 1.0, 0.0 );

  // Assert
  ASSERT_EQ( 100, ticks.linear_ticks_sec );
  ASSERT_EQ( -5.0, ticks.angular_ticks_sec );

  // Act
  ticks = base_model.VelocityToTicks( 0.0, 1.0 );

  // Assert
  ASSERT_EQ( 0, ticks.linear_ticks_sec );
  ASSERT_EQ( 50, ticks.angular_ticks_sec );

}
}
