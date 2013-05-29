/* Unit test for EncoderCountsReader class.
 *
 * @author Sawyer Larkin (SJL toborguru)
 */

#include <gtest/gtest.h>

#include "differential_drive/EncoderCounts.h"

#include "BaseModel.hpp"
 
using namespace differential_drive_core;
 
namespace differential_drive_core_test_core
{
// Define the unit test to verify Base Model calculated geometry properties
TEST( BaseModelTests, canCalculateTickRates )
{
  // Establish Context
  BaseModel base_model;

  // Act
  base_model.setWheelRadius( 0.5 / M_PI );
  base_model.setWheelTicks( 100 );
  base_model.setWheelBase( 0.5 );

  // Assert
  EXPECT_FLOAT_EQ( 100.0, base_model.getTicksPerMeter() );
  EXPECT_FLOAT_EQ( 0.01, base_model.getMetersPerTick() );
  EXPECT_FLOAT_EQ( 50.0, base_model.getTicksPerRadian() );
  EXPECT_FLOAT_EQ( 0.02, base_model.getRadiansPerTick() );
  EXPECT_FLOAT_EQ( 0.0, base_model.getStasisTicksPerMeter() );

  EXPECT_FLOAT_EQ( 1.0, base_model.getLeftInRightOutCorrection() );
  EXPECT_FLOAT_EQ( 1.0, base_model.getRightInLeftOutCorrection() );

  // Act
  base_model.setStasisTicks( 10 );
  base_model.setStasisRadius( 1.0 / M_PI );
  base_model.setWheelRatio( 0.95 );

  // Assert
  EXPECT_FLOAT_EQ( 5.0, base_model.getStasisTicksPerMeter() );
  EXPECT_FLOAT_EQ( 0.2, base_model.getMetersPerStasisTick() );

  EXPECT_FLOAT_EQ( 0.97435898, base_model.getLeftInRightOutCorrection() );
  EXPECT_FLOAT_EQ( 1.0256411, base_model.getRightInLeftOutCorrection() );
}

// Define the unit test to verify Base Model calculated dead reckoning
TEST( BaseModelTests, canCalculateDeadReckoning )
{
  // Establish Context
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );
  
  BaseDistance_T distance;
  BaseVelocities_T velocity;

  differential_drive::EncoderCounts counts; 

  // Act: 1m straight
  counts.left_count = 100;
  counts.right_count = 100;
  counts.dt_ms = 100;

  base_model.convertCounts( &distance, &velocity, counts );

  // Assert
  EXPECT_FLOAT_EQ( 1.0, distance.linear );
  EXPECT_FLOAT_EQ( 0.0, distance.theta );
  EXPECT_FLOAT_EQ( 10.0, velocity.linear );
  EXPECT_FLOAT_EQ( 0.0, velocity.angular );

  // Act: 1m straight, slow
  counts.left_count = 100;
  counts.right_count = 100;
  counts.dt_ms = 500;

  base_model.convertCounts( &distance, &velocity, counts );

  // Assert
  EXPECT_FLOAT_EQ( 1.0, distance.linear );
  EXPECT_FLOAT_EQ( 0.0, distance.theta );
  EXPECT_FLOAT_EQ( 2.0, velocity.linear );
  EXPECT_FLOAT_EQ( 0.0, velocity.angular );

  // Act: 1rad
  counts.left_count = -25;
  counts.right_count = 25;
  counts.dt_ms = 100;

  base_model.convertCounts( &distance, &velocity, counts );

  // Assert
  EXPECT_FLOAT_EQ( 0.0, distance.linear );
  EXPECT_FLOAT_EQ( 1.0, distance.theta );
  EXPECT_FLOAT_EQ( 0.0, velocity.linear );
  EXPECT_FLOAT_EQ( 10.0, velocity.angular );

  // Act
  counts.left_count = 75;
  counts.right_count = 125;
  counts.dt_ms = 100;

  base_model.convertCounts( &distance, &velocity, counts );

  // Assert
  EXPECT_FLOAT_EQ( 1.0, distance.theta );
  EXPECT_FLOAT_EQ( 10.0, velocity.linear );
  EXPECT_FLOAT_EQ( 10.0, velocity.angular );

  // Act: fast
  counts.left_count = 75;
  counts.right_count = 125;
  counts.dt_ms = 20;

  base_model.convertCounts( &distance, &velocity, counts );

  // Assert
  EXPECT_FLOAT_EQ( 1.0, distance.theta );
  EXPECT_FLOAT_EQ( 50.0, velocity.linear );
  EXPECT_FLOAT_EQ( 50.0, velocity.angular );

  // Act
  counts.left_count = 25;
  counts.right_count = -25;
  counts.dt_ms = 100;

  base_model.convertCounts( &distance, &velocity, counts );

  // Assert
  EXPECT_FLOAT_EQ( 0.0, distance.linear );
  EXPECT_FLOAT_EQ( -1.0, distance.theta );
  EXPECT_FLOAT_EQ( 0.0, velocity.linear );
  EXPECT_FLOAT_EQ( -10.0, velocity.angular );

  // Act
  counts.left_count = -75;
  counts.right_count = -125;
  counts.dt_ms = 100;

  base_model.convertCounts( &distance, &velocity, counts );

  // Assert
  EXPECT_FLOAT_EQ( -1.0, distance.theta );
  EXPECT_FLOAT_EQ( -10.0, velocity.linear );
  EXPECT_FLOAT_EQ( -10.0, velocity.angular );
}

// Define the unit test to verify Base Model calibrated dead reckoning
TEST( BaseModelTests, canCalculateDeadReckoningCalibrated )
{
  double x1, theta1, x2, theta2;
  // Establish Context
  BaseModel base_model( 0.5 / M_PI, 100, 0.5, 0.95);
  differential_drive::EncoderCounts counts; 

  BaseDistance_T distance;
  BaseVelocities_T velocity;

  // Act
  counts.left_count = 205;
  counts.right_count = 195;
  counts.dt_ms = 100;

  base_model.convertCounts( &distance, &velocity, counts );
  
  x1 = distance.linear;
  theta1 = distance.theta;
  
  counts.left_count = -205;
  counts.right_count = 195;
  counts.dt_ms = 100;

  base_model.convertCounts( &distance, &velocity, counts );
  
  x2 = distance.linear;
  theta2 = distance.theta;

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
  EXPECT_NEAR( 2.0, x1, 0.01 );
  EXPECT_NEAR( 0.0, theta1, 0.01 );

  EXPECT_NEAR( 0.0, x2, 0.01 );
  EXPECT_NEAR( 8.0, theta2, 0.01 );
}

// Define the unit test to verify Base Model tick velocities
TEST( BaseModelTests, canCalculateTickVelocities )
{
  // Establish Context
  BaseModel base_model( 0.5 / M_PI, 100, 0.5 );
  differential_drive::TickVelocity ticks; 

  // Act
  ticks = base_model.convertVelocity( 1.0, 0.0 );

  // Assert
  EXPECT_EQ( 100, ticks.linear_ticks_sec );
  EXPECT_EQ( 0.0, ticks.angular_ticks_sec );

  // Act
  ticks = base_model.convertVelocity( 0.0, 1.0 );

  // Assert
  EXPECT_EQ( 0, ticks.linear_ticks_sec );
  EXPECT_EQ( 50, ticks.angular_ticks_sec );
}

// Define the unit test to verify Base Model tick velocities
TEST( BaseModelTests, canCalculateTickVelocitiesCalibrated )
{
  // Establish Context
  BaseModel base_model( 0.5 / M_PI, 100, 0.5, 0.95 );
  differential_drive::TickVelocity ticks; 

  // Act
  ticks = base_model.convertVelocity( 1.0, 0.0 );

  // Assert
  EXPECT_EQ( 100, ticks.linear_ticks_sec );
  EXPECT_EQ( -5.0, ticks.angular_ticks_sec );

  // Act
  ticks = base_model.convertVelocity( 0.0, 8.0 );

  // Assert
  EXPECT_EQ( -5, ticks.linear_ticks_sec );
  EXPECT_EQ( 400, ticks.angular_ticks_sec );
}
}
