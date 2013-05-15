// DifferentialParametersRepositoryTests.cpp
 
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "DifferentialParametersRepository.hpp"
 
using namespace differential_drive_data_repositories;
using namespace differential_drive_core;
 
namespace differential_drive_test_data_repositories
{
TEST(DifferentialParametersRepositoryTests, canReadROSBaseParameters) 
{
  // Establish Context
  std::string name( "parameters_repository_tester" );
  int argc = 0;
  ros::init( argc, (char**)NULL, name );

  BaseGeometry_T base_geometry;
  BaseModel base_model(12.98);

  DifferentialParametersRepository parameters_repository;
  parameters_repository.SetBaseModel( &base_model );

  double radius1;
  double base1;
  double stasis_ticks1;
  double stasis_radius1;

  double radius2;
  double base2;
  double stasis_ticks2;
  double stasis_radius2;

  double radius3;
  double base3;
  double stasis_ticks3;
  double stasis_radius3;

  double radius4;
  double base4;
  double stasis_ticks4;
  double stasis_radius4;

  // Act
  // Establish all four parameters in case of a previous setting
  ros::param::set( "~drive_wheel_diameter", 0.5 );
  ros::param::set( "~drive_wheel_base", 1.0 );
  ros::param::set( "~stasis_wheel_diameter", 0.0 );
  ros::param::set( "~stasis_wheel_encoder_ticks", 100 );

  parameters_repository.QueryBaseParameters();
  base_geometry = base_model.GetBaseGeometry();

  radius1 = base_geometry.wheel_radius;
  base1 = base_geometry.wheel_base;
  stasis_ticks1 = base_geometry.stasis_ticks;
  stasis_radius1 = base_geometry.stasis_radius;

  ros::param::set( "~drive_wheel_diameter", 1.5 );
  ros::param::set( "~drive_wheel_base", 1.5 );
  ros::param::set( "~stasis_wheel_diameter", 0.5 );

  parameters_repository.QueryBaseParameters();
  base_geometry = base_model.GetBaseGeometry();

  radius2 = base_geometry.wheel_radius;
  base2 = base_geometry.wheel_base;
  stasis_ticks2 = base_geometry.stasis_ticks;
  stasis_radius2 = base_geometry.stasis_radius;

  ros::param::set( "~drive_wheel_diameter", 1.0 );
  ros::param::set( "~drive_wheel_base", 1.0 );
  ros::param::set( "~stasis_wheel_diameter", -0.5 );

  parameters_repository.QueryBaseParameters();
  base_geometry = base_model.GetBaseGeometry();

  radius3 = base_geometry.wheel_radius;
  base3 = base_geometry.wheel_base;
  stasis_ticks3 = base_geometry.stasis_ticks;
  stasis_radius3 = base_geometry.stasis_radius;

  ros::param::set( "~drive_wheel_diameter", 2.5 );
  ros::param::set( "~drive_wheel_base", 2.5 );
  ros::param::set( "~stasis_wheel_diameter", 1.5 );
  ros::param::set( "~stasis_wheel_encoder_ticks", 150 );

  parameters_repository.QueryBaseParameters();
  base_geometry = base_model.GetBaseGeometry();

  radius4 = base_geometry.wheel_radius;
  base4 = base_geometry.wheel_base;
  stasis_ticks4 = base_geometry.stasis_ticks;
  stasis_radius4 = base_geometry.stasis_radius;

  // Assert
  EXPECT_FLOAT_EQ( 0.25, radius1 );
  EXPECT_FLOAT_EQ( 1.0, base1 );
  EXPECT_FLOAT_EQ( 0.0, stasis_radius1 );
  EXPECT_EQ( 100, stasis_ticks1 );

  EXPECT_FLOAT_EQ( 0.75, radius2 );
  EXPECT_FLOAT_EQ( 1.5, base2 );
  EXPECT_FLOAT_EQ( 0.25, stasis_radius2 );
  EXPECT_EQ( 100, stasis_ticks2 );

  EXPECT_FLOAT_EQ( 0.5, radius3 );
  EXPECT_FLOAT_EQ( 1.0, base3 );
  EXPECT_FLOAT_EQ( 0.0, stasis_radius3 );
  EXPECT_EQ( 100, stasis_ticks3 );

  EXPECT_FLOAT_EQ( 1.25, radius4 );
  EXPECT_FLOAT_EQ( 2.5, base4 );
  EXPECT_FLOAT_EQ( 0.75, stasis_radius4 );
  EXPECT_EQ( 150, stasis_ticks4 );
}

TEST(DifferentialParametersRepositoryTests, canWriteROSBaseParameters) 
{
  // Establish Context
  std::string name( "parameters_repository_tester" );
  int argc = 0;
  ros::init( argc, (char**)NULL, name );

  BaseModel base_model;

  DifferentialParametersRepository parameters_repository;
  parameters_repository.SetBaseModel( &base_model );

  //OdometryIntegrator odometry_integrator;
  //parameters_repository.SetOdometryIntegrator( &odometry_integrator );

  double wheel_diameter1;
  double base1;
  int stasis_ticks1;
  double stasis_diameter1;

  double wheel_diameter2;
  double base2;
  int stasis_ticks2;
  double stasis_diameter2;

  double wheel_diameter3;
  double base3;
  int stasis_ticks3;
  double stasis_diameter3;

  double wheel_diameter4;
  double base4;
  int stasis_ticks4;
  double stasis_diameter4;

  // Act
  // Establish parameters in case of a previous setting
  parameters_repository.PersistBaseParameters();
  
  ros::param::param<double>( "~drive_wheel_diameter", wheel_diameter1, -10.0 );
  ros::param::param<double>( "~drive_wheel_base", base1, -10.0 );
  ros::param::param<double>( "~stasis_wheel_diameter", stasis_diameter1, -10.0 );
  ros::param::param<int>( "~stasis_wheel_encoder_ticks", stasis_ticks1, -10 );

  // Change the local copy
  base_model.SetWheelRadius( 3.14 );
  base_model.SetWheelBase( 6.28 );
  base_model.SetStasisTicks( 4096 );
  base_model.SetStasisRadius( 1.0 );

  // Pre-Persist
  ros::param::param<double>( "~drive_wheel_diameter", wheel_diameter2, -10.0 );
  ros::param::param<double>( "~drive_wheel_base", base2, -10.0 );
  ros::param::param<double>( "~stasis_wheel_diameter", stasis_diameter2, -10.0 );
  ros::param::param<int>( "~stasis_wheel_encoder_ticks", stasis_ticks2, -10 );

  parameters_repository.PersistBaseParameters();

  // Post-persist
  ros::param::param<double>( "~drive_wheel_diameter", wheel_diameter3, -10.0 );
  ros::param::param<double>( "~drive_wheel_base", base3, -10.0 );
  ros::param::param<double>( "~stasis_wheel_diameter", stasis_diameter3, -10.0 );
  ros::param::param<int>( "~stasis_wheel_encoder_ticks", stasis_ticks3, -10 );

  // Change the local copy
  base_model.SetWheelRadius( -1.0 );
  base_model.SetWheelBase( -1.0 );
  base_model.SetStasisTicks( -4096 );
  base_model.SetStasisRadius( -4.0 );

  parameters_repository.PersistBaseParameters();

  // Post-persist
  ros::param::param<double>( "~drive_wheel_diameter", wheel_diameter4, -10.0 );
  ros::param::param<double>( "~drive_wheel_base", base4, -10.0 );
  ros::param::param<double>( "~stasis_wheel_diameter", stasis_diameter4, -10.0 );
  ros::param::param<int>( "~stasis_wheel_encoder_ticks", stasis_ticks4, -10 );

  // Assert
  EXPECT_FLOAT_EQ( 0.0, wheel_diameter1 );
  EXPECT_FLOAT_EQ( 0.0, base1 );
  EXPECT_FLOAT_EQ( 0.0, stasis_diameter1 );
  EXPECT_EQ( -1, stasis_ticks1 );

  EXPECT_FLOAT_EQ( 0.0, wheel_diameter2 );
  EXPECT_FLOAT_EQ( 0.0, base2 );
  EXPECT_FLOAT_EQ( 0.0, stasis_diameter2 );
  EXPECT_EQ( -1, stasis_ticks2 );

  EXPECT_FLOAT_EQ( 6.28, wheel_diameter3 );
  EXPECT_FLOAT_EQ( 6.28, base3 );
  EXPECT_FLOAT_EQ( 2.0, stasis_diameter3 );
  EXPECT_EQ( 4096, stasis_ticks3 );

  EXPECT_FLOAT_EQ( 6.28, wheel_diameter4 );
  EXPECT_FLOAT_EQ( 6.28, base4 );
  EXPECT_FLOAT_EQ( 0.0, stasis_diameter4 );
  EXPECT_EQ( -1, stasis_ticks4 );
}

TEST(DifferentialParametersRepositoryTests, canReadROSOdometryParameters) 
{
  int pow2_readings1;
  int readings1;
  double percentage1;
  double limit1;
  
  int pow2_readings2;
  int readings2;
  double percentage2;
  double limit2;
  
  int pow2_readings3;
  int readings3;
  double percentage3;
  double limit3;
  
  int pow2_readings4;
  int readings4;
  double percentage4;
  double limit4;
  
  // Establish Context
  std::string name( "parameters_repository_tester" );
  int argc = 0;
  ros::init( argc, (char**)NULL, name );

  BaseModel base_model;

  DifferentialParametersRepository parameters_repository;

  OdometryIntegrator odometry_integrator;
  odometry_integrator.SetBaseModel( base_model );

  parameters_repository.SetOdometryIntegrator( &odometry_integrator );

  // Act
  // Establish all four parameters in case of a previous setting
  ros::param::set( "~average_2n_readings", 5 );
  ros::param::set( "~velocity_difference_percentage", 1.0 );
  ros::param::set( "~velocity_lower_limit", 0.1 );

  parameters_repository.QueryOdometryParameters();

  pow2_readings1 = odometry_integrator.GetAverage2nReadings();
  readings1 = odometry_integrator.GetAverageNumReadings();
  limit1 = odometry_integrator.GetVelocityLowerLimit();
  percentage1 = odometry_integrator.GetVelocityMatchPercentage();

  ros::param::set( "~average_2n_readings", 0 );
  ros::param::set( "~velocity_difference_percentage", 90.0 );
  ros::param::set( "~velocity_lower_limit", 1.5 );

  parameters_repository.QueryOdometryParameters();

  pow2_readings2 = odometry_integrator.GetAverage2nReadings();
  readings2 = odometry_integrator.GetAverageNumReadings();
  limit2 = odometry_integrator.GetVelocityLowerLimit();
  percentage2 = odometry_integrator.GetVelocityMatchPercentage();

  ros::param::set( "~average_2n_readings", -5 );
  ros::param::set( "~velocity_difference_percentage", -75.0 );
  ros::param::set( "~velocity_lower_limit", -0.18 );

  parameters_repository.QueryOdometryParameters();

  pow2_readings3 = odometry_integrator.GetAverage2nReadings();
  readings3 = odometry_integrator.GetAverageNumReadings();
  limit3 = odometry_integrator.GetVelocityLowerLimit();
  percentage3 = odometry_integrator.GetVelocityMatchPercentage();

  ros::param::set( "~average_2n_readings", 15 );
  ros::param::set( "~velocity_difference_percentage", 25.0 );
  ros::param::set( "~velocity_lower_limit", 1.0 );

  parameters_repository.QueryOdometryParameters();

  pow2_readings4 = odometry_integrator.GetAverage2nReadings();
  readings4 = odometry_integrator.GetAverageNumReadings();
  limit4 = odometry_integrator.GetVelocityLowerLimit();
  percentage4 = odometry_integrator.GetVelocityMatchPercentage();

  // Assert
  EXPECT_EQ( 5, pow2_readings1 );
  EXPECT_EQ( 32, readings1 );
  EXPECT_FLOAT_EQ( 1.0, percentage1 );
  EXPECT_FLOAT_EQ( 0.1, limit1 );

  EXPECT_EQ( 0, pow2_readings2 );
  EXPECT_EQ( 1, readings2 );
  EXPECT_FLOAT_EQ( 90.0, percentage2 );
  EXPECT_FLOAT_EQ( 1.5, limit2 );

  EXPECT_EQ( 0, pow2_readings3 );
  EXPECT_EQ( 1, readings3 );
  EXPECT_FLOAT_EQ( 90.0, percentage3 );
  EXPECT_FLOAT_EQ( 1.5, limit3 );

  EXPECT_EQ( 15, pow2_readings4 );
  EXPECT_EQ( 32768, readings4 );
  EXPECT_FLOAT_EQ( 25.0, percentage4 );
  EXPECT_FLOAT_EQ( 1.0, limit4 );
}

TEST(DifferentialParametersRepositoryTests, canWriteROSOdometryParameters) 
{
  int pow2_readings1;
  int readings1;
  double percentage1;
  double limit1;
  
  int pow2_readings2;
  int readings2;
  double percentage2;
  double limit2;
  
  int pow2_readings3;
  int readings3;
  double percentage3;
  double limit3;
  
  int pow2_readings4;
  int readings4;
  double percentage4;
  double limit4;
  
  // Establish Context
  std::string name( "parameters_repository_tester" );
  int argc = 0;
  ros::init( argc, (char**)NULL, name );

  BaseModel base_model;

  DifferentialParametersRepository parameters_repository;

  OdometryIntegrator odometry_integrator;
  odometry_integrator.SetBaseModel( base_model );

  parameters_repository.SetOdometryIntegrator( &odometry_integrator );

  // Act
  // Defaults
  parameters_repository.PersistOdometryParameters();
  
  ros::param::param<double>( "~velocity_difference_percentage", percentage1, -10.0 );
  ros::param::param<double>( "~velocity_lower_limit", limit1, -10.0 );
  ros::param::param<int>( "~average_2n_readings", pow2_readings1, -10 );
  ros::param::param<int>( "~average_num_readings", readings1, -10 );

  // Change the local copy
  odometry_integrator.SetAverage2nReadings( 8 );
  odometry_integrator.SetVelocityMatchPercentage( 15.0 );
  odometry_integrator.SetVelocityLowerLimit( 12.0 );

  // Pre-Persist
  ros::param::param<double>( "~velocity_difference_percentage", percentage2, -10.0 );
  ros::param::param<double>( "~velocity_lower_limit", limit2, -10.0 );
  ros::param::param<int>( "~average_2n_readings", pow2_readings2, -10.0 );
  ros::param::param<int>( "~average_num_readings", readings2, -10.0 );

  parameters_repository.PersistOdometryParameters();

  // Post-persist
  ros::param::param<double>( "~velocity_difference_percentage", percentage3, -10.0 );
  ros::param::param<double>( "~velocity_lower_limit", limit3, -10.0 );
  ros::param::param<int>( "~average_2n_readings", pow2_readings3, -10.0 );
  ros::param::param<int>( "~average_num_readings", readings3, -10.0 );

  // Change the local copy
  odometry_integrator.SetVelocityLowerLimit( 25.0 );

  parameters_repository.PersistOdometryParameters();

  // Post-persist
  ros::param::param<double>( "~velocity_difference_percentage", percentage4, -10.0 );
  ros::param::param<double>( "~velocity_lower_limit", limit4, -10.0 );
  ros::param::param<int>( "~average_2n_readings", pow2_readings4, -10.0 );
  ros::param::param<int>( "~average_num_readings", readings4, -10.0 );

  // Assert
  EXPECT_FLOAT_EQ( 10.0, percentage1 );
  EXPECT_FLOAT_EQ( 0.05, limit1 );
  EXPECT_FLOAT_EQ( 3, pow2_readings1 );
  EXPECT_EQ( 8, readings1 );

  EXPECT_FLOAT_EQ( 10.0, percentage2 );
  EXPECT_FLOAT_EQ( 0.05, limit2 );
  EXPECT_FLOAT_EQ( 3, pow2_readings2 );
  EXPECT_EQ( 8, readings2 );

  EXPECT_FLOAT_EQ( 15.0, percentage3 );
  EXPECT_FLOAT_EQ( 12.0, limit3 );
  EXPECT_FLOAT_EQ( 8, pow2_readings3 );
  EXPECT_EQ( 256, readings3 );

  EXPECT_FLOAT_EQ( 15.0, percentage4 );
  EXPECT_FLOAT_EQ( 25.0, limit4 );
  EXPECT_FLOAT_EQ( 8, pow2_readings4 );
  EXPECT_EQ( 256, readings4 );
}

TEST(DifferentialParametersRepositoryTests, canUseCallbackFunction) 
{
  int pow2_readings1;
  int readings1;
  double percentage1;
  double limit1;
  
  int pow2_readings2;
  int readings2;
  double percentage2;
  double limit2;
  
  int ros_pow2_readings1;
  int ros_readings1;
  double ros_percentage1;
  double ros_limit1;
  
  int ros_pow2_readings2;
  int ros_readings2;
  double ros_percentage2;
  double ros_limit2;
  
  double radius1;
  double base1;
  double stasis_ticks1;
  double stasis_radius1;

  double radius2;
  double base2;
  double stasis_ticks2;
  double stasis_radius2;

  double ros_wheel_diameter1;
  double ros_base1;
  int ros_stasis_ticks1;
  double ros_stasis_diameter1;

  double ros_wheel_diameter2;
  double ros_base2;
  int ros_stasis_ticks2;
  double ros_stasis_diameter2;

  // Establish Context
  std::string name( "parameters_repository_tester" );
  int argc = 0;
  ros::init( argc, (char**)NULL, name );

  BaseGeometry_T base_geometry;
  BaseModel base_model;

  DifferentialParametersRepository parameters_repository;
  parameters_repository.SetBaseModel( &base_model );

  OdometryIntegrator odometry_integrator;
  parameters_repository.SetOdometryIntegrator( &odometry_integrator );

  differential_drive::DifferentialParametersConfig new_config;

  // Clean-up ROS parameter server
  parameters_repository.PersistOdometryParameters();
  parameters_repository.PersistBaseParameters();

  // Record default values
  pow2_readings1 = odometry_integrator.GetAverage2nReadings();
  readings1 = odometry_integrator.GetAverageNumReadings();
  limit1 = odometry_integrator.GetVelocityLowerLimit();
  percentage1 = odometry_integrator.GetVelocityMatchPercentage();

  ros::param::param<double>( "~velocity_difference_percentage", ros_percentage1, -10.0 );
  ros::param::param<double>( "~velocity_lower_limit", ros_limit1, -10.0 );
  ros::param::param<int>( "~average_2n_readings", ros_pow2_readings1, -10 );
  ros::param::param<int>( "~average_num_readings", ros_readings1, -10 );

  base_geometry = base_model.GetBaseGeometry();

  radius1 = base_geometry.wheel_radius;
  base1 = base_geometry.wheel_base;
  stasis_ticks1 = base_geometry.stasis_ticks;
  stasis_radius1 = base_geometry.stasis_radius;

  ros::param::param<double>( "~drive_wheel_diameter", ros_wheel_diameter1, -10.0 );
  ros::param::param<double>( "~drive_wheel_base", ros_base1, -10.0 );
  ros::param::param<double>( "~stasis_wheel_diameter", ros_stasis_diameter1, -10.0 );
  ros::param::param<int>( "~stasis_wheel_encoder_ticks", ros_stasis_ticks1, -10 );

  // ACT
  new_config.drive_wheel_diameter           = 12.34;
  new_config.drive_wheel_base               = 5.67;
  new_config.drive_wheel_ratio              = 0.89;
  new_config.drive_wheel_encoder_ticks      = 10;
  
  new_config.stasis_wheel_diameter          = 1.112;
  new_config.stasis_wheel_encoder_ticks     = 0;

  new_config.average_2n_readings            = 13;
  new_config.velocity_difference_percentage = 14;
  new_config.velocity_lower_limit           = 15;

  parameters_repository.UpdateParametersCallBack(new_config, 0);

  // Verify changes
  pow2_readings2 = odometry_integrator.GetAverage2nReadings();
  readings2 = odometry_integrator.GetAverageNumReadings();
  limit2 = odometry_integrator.GetVelocityLowerLimit();
  percentage2 = odometry_integrator.GetVelocityMatchPercentage();

  ros::param::param<double>( "~velocity_difference_percentage", ros_percentage2, -10.0 );
  ros::param::param<double>( "~velocity_lower_limit", ros_limit2, -10.0 );
  ros::param::param<int>( "~average_2n_readings", ros_pow2_readings2, -10 );
  ros::param::param<int>( "~average_num_readings", ros_readings2, -10 );

  base_geometry = base_model.GetBaseGeometry();

  radius2 = base_geometry.wheel_radius;
  base2 = base_geometry.wheel_base;
  stasis_ticks2 = base_geometry.stasis_ticks;
  stasis_radius2 = base_geometry.stasis_radius;

  ros::param::param<double>( "~drive_wheel_diameter", ros_wheel_diameter2, -10.0 );
  ros::param::param<double>( "~drive_wheel_base", ros_base2, -10.0 );
  ros::param::param<double>( "~stasis_wheel_diameter", ros_stasis_diameter2, -10.0 );
  ros::param::param<int>( "~stasis_wheel_encoder_ticks", ros_stasis_ticks2, -10 );

  EXPECT_EQ( 3, pow2_readings1 );
  EXPECT_EQ( 8, readings1 );
  EXPECT_FLOAT_EQ( 10.0, percentage1 );
  EXPECT_FLOAT_EQ( 0.05, limit1 );

  EXPECT_EQ( 3, ros_pow2_readings1 );
  EXPECT_EQ( 8, ros_readings1 );
  EXPECT_FLOAT_EQ( 10.0, ros_percentage1 );
  EXPECT_FLOAT_EQ( 0.05, ros_limit1 );

  EXPECT_FLOAT_EQ( 0.0, radius1 );
  EXPECT_FLOAT_EQ( 0.0, base1 );
  EXPECT_FLOAT_EQ( 0.0, stasis_radius1 );
  EXPECT_EQ( -1, stasis_ticks1 );

  EXPECT_FLOAT_EQ( 0.0, ros_wheel_diameter1 );
  EXPECT_FLOAT_EQ( 0.0, ros_base1 );
  EXPECT_FLOAT_EQ( 0.0, ros_stasis_diameter1 );
  EXPECT_EQ( -1, ros_stasis_ticks1 );

  EXPECT_EQ( 13, pow2_readings2 );
  EXPECT_EQ( 8192, readings2 );
  EXPECT_FLOAT_EQ( 14, percentage2 );
  EXPECT_FLOAT_EQ( 15, limit2 );

  EXPECT_EQ( 13, ros_pow2_readings2 );
  EXPECT_EQ( 8192, ros_readings2 );
  EXPECT_FLOAT_EQ( 14, ros_percentage2 );
  EXPECT_FLOAT_EQ( 15, ros_limit2 );

  EXPECT_FLOAT_EQ( 12.34 / 2, radius2 );
  EXPECT_FLOAT_EQ( 5.67, base2 );
  EXPECT_FLOAT_EQ( 1.112 / 2, stasis_radius2 );
  EXPECT_EQ( -1, stasis_ticks2 );

  EXPECT_FLOAT_EQ( 12.34 , ros_wheel_diameter2 );
  EXPECT_FLOAT_EQ( 5.67, ros_base2 );
  EXPECT_FLOAT_EQ( 1.112, ros_stasis_diameter2 );
  EXPECT_EQ( -1, ros_stasis_ticks2 );
}
}
