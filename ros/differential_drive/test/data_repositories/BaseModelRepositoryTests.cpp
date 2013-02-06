// BaseModelRepositoryTests.cpp
 
#include <gtest/gtest.h>
#include <ros/ros.h>


#include "BaseModelRepository.hpp"
 
using namespace differential_drive_data_repositories;
//using namespace nav_msgs_application_services;
 
namespace differential_drive_test_data_repositories
{
TEST(BaseModelRepositoryTests, canReadROSBaseModelParameters) 
{
  // Establish Context
  std::string name( "base_model_repository_tester" );
  int argc = 0;
  ros::init( argc, (char**)NULL, name );

  differential_drive_core::BaseGeometry_T base_geometry;
  differential_drive_core::BaseModel base_model;

  BaseModelRepository base_model_repository;
  base_model_repository.SetBaseModel( &base_model );

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

  base_model_repository.QueryBaseGeometry();
  base_geometry = base_model.GetBaseGeometry();

  radius1 = base_geometry.wheel_radius;
  base1 = base_geometry.wheel_base;
  stasis_ticks1 = base_geometry.stasis_ticks;
  stasis_radius1 = base_geometry.stasis_radius;

  ros::param::set( "~drive_wheel_diameter", -1.5 );
  ros::param::set( "~drive_wheel_base", 1.5 );
  ros::param::set( "~stasis_wheel_diameter", 0.5 );

  base_model_repository.QueryBaseGeometry();
  base_geometry = base_model.GetBaseGeometry();

  radius2 = base_geometry.wheel_radius;
  base2 = base_geometry.wheel_base;
  stasis_ticks2 = base_geometry.stasis_ticks;
  stasis_radius2 = base_geometry.stasis_radius;

  ros::param::set( "~drive_wheel_diameter", 1.0 );
  ros::param::set( "~drive_wheel_base", -1.0 );
  ros::param::set( "~stasis_wheel_diameter", -0.5 );

  base_model_repository.QueryBaseGeometry();
  base_geometry = base_model.GetBaseGeometry();

  radius3 = base_geometry.wheel_radius;
  base3 = base_geometry.wheel_base;
  stasis_ticks3 = base_geometry.stasis_ticks;
  stasis_radius3 = base_geometry.stasis_radius;

  ros::param::set( "~drive_wheel_diameter", 2.5 );
  ros::param::set( "~drive_wheel_base", 2.5 );
  ros::param::set( "~stasis_wheel_diameter", 1.5 );
  ros::param::set( "~stasis_wheel_encoder_ticks", 150 );

  base_model_repository.QueryBaseGeometry();
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
}
