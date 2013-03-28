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
  BaseModel base_model;

  DifferentialParametersRepository parameters_repository;
  parameters_repository.SetBaseModel( &base_model );

  //OdometryIntegrator odometry_integrator;
  //parameters_repository.SetOdometryIntegrator( &odometry_integrator );

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

  ros::param::set( "~drive_wheel_diameter", -1.5 );
  ros::param::set( "~drive_wheel_base", 1.5 );
  ros::param::set( "~stasis_wheel_diameter", 0.5 );

  parameters_repository.QueryBaseParameters();
  base_geometry = base_model.GetBaseGeometry();

  radius2 = base_geometry.wheel_radius;
  base2 = base_geometry.wheel_base;
  stasis_ticks2 = base_geometry.stasis_ticks;
  stasis_radius2 = base_geometry.stasis_radius;

  ros::param::set( "~drive_wheel_diameter", 1.0 );
  ros::param::set( "~drive_wheel_base", -1.0 );
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
  EXPECT_DOUBLE_EQ( 0.25, radius1 );
  EXPECT_DOUBLE_EQ( 1.0, base1 );
  EXPECT_DOUBLE_EQ( 1.0, stasis_radius1 );
  EXPECT_EQ( -1, stasis_ticks1 );

  EXPECT_DOUBLE_EQ( 0.75, radius2 );
  EXPECT_DOUBLE_EQ( 1.5, base2 );
  EXPECT_DOUBLE_EQ( 0.25, stasis_radius2 );
  EXPECT_EQ( 100, stasis_ticks2 );

  EXPECT_DOUBLE_EQ( 0.5, radius3 );
  EXPECT_DOUBLE_EQ( 1.0, base3 );
  EXPECT_DOUBLE_EQ( 1.0, stasis_radius3 );
  EXPECT_EQ( -1, stasis_ticks3 );

  EXPECT_DOUBLE_EQ( 1.25, radius4 );
  EXPECT_DOUBLE_EQ( 2.5, base4 );
  EXPECT_DOUBLE_EQ( 0.75, stasis_radius4 );
  EXPECT_EQ( 150, stasis_ticks4 );
}

TEST(DifferentialParametersRepositoryTests, canWriteROSBaseParameters) 
{
  // Establish Context
  std::string name( "parameters_repository_tester" );
  int argc = 0;
  ros::init( argc, (char**)NULL, name );

  BaseGeometry_T base_geometry;
  BaseModel base_model;

  DifferentialParametersRepository parameters_repository;
  parameters_repository.SetBaseModel( &base_model );

  //OdometryIntegrator odometry_integrator;
  //parameters_repository.SetOdometryIntegrator( &odometry_integrator );

  double wheel_diameter1;
  double base1;
  double stasis_ticks1;
  double stasis_diameter1;

  double wheel_diameter2;
  double base2;
  double stasis_ticks2;
  double stasis_diameter2;

  double wheel_diameter3;
  double base3;
  double stasis_ticks3;
  double stasis_diameter3;

  double wheel_diameter4;
  double base4;
  double stasis_ticks4;
  double stasis_diameter4;

  // Act
  // Establish parameters in case of a previous setting
  parameters_repository.PersistBaseParameters();
  
  ros::param::param<double>( "~drive_wheel_diameter", wheel_diameter1, -10.0 );
  ros::param::param<double>( "~drive_wheel_base", base1, -10.0 );
  ros::param::param<double>( "~stasis_wheel_diameter", stasis_diameter1, -10.0 );
  ros::param::param<double>( "~stasis_wheel_encoder_ticks", stasis_ticks1, -10 );

  // Change the local copy
  base_model.SetWheelRadius( 3.14 );
  base_model.SetWheelBase( 6.28 );
  base_model.SetStasisTicks( 4096 );
  base_model.SetStasisRadius( 1.0 );

  // Pre-Persist
  ros::param::param<double>( "~drive_wheel_diameter", wheel_diameter2, -10.0 );
  ros::param::param<double>( "~drive_wheel_base", base2, -10.0 );
  ros::param::param<double>( "~stasis_wheel_diameter", stasis_diameter2, -10.0 );
  ros::param::param<double>( "~stasis_wheel_encoder_ticks", stasis_ticks2, -10 );

  parameters_repository.PersistBaseParameters();

  // Post-persist
  ros::param::param<double>( "~drive_wheel_diameter", wheel_diameter3, -10.0 );
  ros::param::param<double>( "~drive_wheel_base", base3, -10.0 );
  ros::param::param<double>( "~stasis_wheel_diameter", stasis_diameter3, -10.0 );
  ros::param::param<double>( "~stasis_wheel_encoder_ticks", stasis_ticks3, -10 );

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
  ros::param::param<double>( "~stasis_wheel_encoder_ticks", stasis_ticks4, -10 );

  // Assert
  EXPECT_DOUBLE_EQ( 0.0, wheel_diameter1 );
  EXPECT_DOUBLE_EQ( 0.0, base1 );
  EXPECT_DOUBLE_EQ( 0.0, stasis_diameter1 );
  EXPECT_EQ( -1, stasis_ticks1 );

  EXPECT_DOUBLE_EQ( 0.0, wheel_diameter2 );
  EXPECT_DOUBLE_EQ( 0.0, base2 );
  EXPECT_DOUBLE_EQ( 0.0, stasis_diameter2 );
  EXPECT_EQ( -1, stasis_ticks2 );

  EXPECT_DOUBLE_EQ( 6.28, wheel_diameter3 );
  EXPECT_DOUBLE_EQ( 6.28, base3 );
  EXPECT_DOUBLE_EQ( 2.0, stasis_diameter3 );
  EXPECT_EQ( 4096, stasis_ticks3 );

  EXPECT_DOUBLE_EQ( 6.28, wheel_diameter4 );
  EXPECT_DOUBLE_EQ( 6.28, base4 );
  EXPECT_DOUBLE_EQ( 0.0, stasis_diameter4 );
  EXPECT_EQ( -1, stasis_ticks4 );
}
}
