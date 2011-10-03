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
TEST( BaseModelTests, canCalculateDeadReckoning )
{
  // Establish Context
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );
  diff_drive::EncoderCounts counts; 

  // Act
  counts.left_count = 100;
  counts.right_count = 100;
  counts.dt_ms = 100;

  base_model.ConvertCounts( counts );

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

  base_model.ConvertCounts( counts );

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

  base_model.ConvertCounts( counts );

  // Assert
  ASSERT_FLOAT_EQ( 1.0, base_model.GetDeltaTheta() );
  ASSERT_FLOAT_EQ( 10.0, base_model.GetLinearVelocity() );
  ASSERT_FLOAT_EQ( 10.0, base_model.GetAngularVelocity() );

  // Act
  counts.left_count = 25;
  counts.right_count = -25;
  counts.dt_ms = 100;

  base_model.ConvertCounts( counts );

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

  base_model.ConvertCounts( counts );

  // Assert
  ASSERT_FLOAT_EQ( -1.0, base_model.GetDeltaTheta() );
  ASSERT_FLOAT_EQ( -10.0, base_model.GetLinearVelocity() );
  ASSERT_FLOAT_EQ( -10.0, base_model.GetAngularVelocity() );
}

// Define the unit test to verify Base Model calibrated dead reckoning
TEST( BaseModelTests, canCalculateCalibratedDeadReckoning )
{
  double x1, y1, theta1, x2, y2, theta2;
  // Establish Context
  BaseModel base_model( 0.5 / M_PI, 100, 0.5, 0.95);
  diff_drive::EncoderCounts counts; 

  // Act
  counts.left_count = 205;
  counts.right_count = 195;
  counts.dt_ms = 100;

  base_model.ConvertCounts( counts );
  
  x1 = base_model.GetDeltaX();
  y1 = base_model.GetDeltaY();
  theta1 = base_model.GetDeltaTheta();
  
  counts.left_count = -205;
  counts.right_count = 195;
  counts.dt_ms = 100;

  base_model.ConvertCounts( counts );
  
  x2 = base_model.GetDeltaX();
  y2 = base_model.GetDeltaY();
  theta2 = base_model.GetDeltaTheta();

#if 0
  std::cout << "X1: " << x1
            << " Y1: " << y1
            << " Theta1: " << theta1
            << " X2: " << x2
            << " Y2: " << y2
            << " Theta2: " << theta2
            << std::endl;
#endif

  // Assert
  ASSERT_NEAR( 2.0, x1, 0.01 );
  ASSERT_NEAR( 0.0, y1, 0.01 );
  ASSERT_NEAR( 0.0, theta1, 0.01 );

  ASSERT_NEAR( 0.0, x2, 0.01 );
  ASSERT_NEAR( 0.0, y2, 0.01 );
  ASSERT_NEAR( 8.0, theta2, 0.01 );
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
  ticks = base_model.VelocityToTicks( 0.0, 8.0 );

  // Assert
  ASSERT_EQ( -5, ticks.linear_ticks_sec );
  ASSERT_EQ( 400, ticks.angular_ticks_sec );
}
}
