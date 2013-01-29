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
    differential_drive_core::BaseGeometry_T base_geometry;

    std::string name( "base_model_repository_tester" );
    int argc = 0;
    ros::init( argc, (char**)NULL, name );

    BaseModelRepository base_model_repository;

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
    ros::param::set( "~drive_wheel_radius", 0.5 );
    ros::param::set( "~drive_wheel_base", 1.0 );
    ros::param::set( "~stasis_wheel_radius", 0.0 );
    ros::param::set( "~stasis_wheel_encoder_ticks", 100 );

    base_geometry = base_model_repository.QueryBaseGeometry();
    radius1 = base_geometry.wheel_radius;
    base1 = base_geometry.wheel_base;
    stasis_ticks1 = base_geometry.stasis_ticks;
    stasis_radius1 = base_geometry.stasis_radius;
  
    ros::param::set( "~drive_wheel_radius", -1.5 );
    ros::param::set( "~drive_wheel_base", 1.5 );
    ros::param::set( "~stasis_wheel_radius", 0.5 );

    base_geometry = base_model_repository.QueryBaseGeometry();
    radius2 = base_geometry.wheel_radius;
    base2 = base_geometry.wheel_base;
    stasis_ticks2 = base_geometry.stasis_ticks;
    stasis_radius2 = base_geometry.stasis_radius;
  
    ros::param::set( "~drive_wheel_radius", 1.0 );
    ros::param::set( "~drive_wheel_base", -1.0 );
    ros::param::set( "~stasis_wheel_radius", -0.5 );

    base_geometry = base_model_repository.QueryBaseGeometry();
    radius3 = base_geometry.wheel_radius;
    base3 = base_geometry.wheel_base;
    stasis_ticks3 = base_geometry.stasis_ticks;
    stasis_radius3 = base_geometry.stasis_radius;
  
    ros::param::set( "~drive_wheel_radius", 2.5 );
    ros::param::set( "~drive_wheel_base", 2.5 );
    ros::param::set( "~stasis_wheel_radius", 1.5 );
    ros::param::set( "~stasis_wheel_encoder_ticks", 150 );

    base_geometry = base_model_repository.QueryBaseGeometry();
    radius4 = base_geometry.wheel_radius;
    base4 = base_geometry.wheel_base;
    stasis_ticks4 = base_geometry.stasis_ticks;
    stasis_radius4 = base_geometry.stasis_radius;
  
    // Assert
    EXPECT_DOUBLE_EQ( 0.5, radius1 );
    EXPECT_DOUBLE_EQ( 1.0, base1 );
    EXPECT_DOUBLE_EQ( 1.0, stasis_radius1 );
    EXPECT_EQ( -1, stasis_ticks1 );

    EXPECT_DOUBLE_EQ( 1.5, radius2 );
    EXPECT_DOUBLE_EQ( 1.5, base2 );
    EXPECT_DOUBLE_EQ( 0.5, stasis_radius2 );
    EXPECT_EQ( 100, stasis_ticks2 );

    EXPECT_DOUBLE_EQ( 1.0, radius3 );
    EXPECT_DOUBLE_EQ( 1.0, base3 );
    EXPECT_DOUBLE_EQ( 1.0, stasis_radius3 );
    EXPECT_EQ( -1, stasis_ticks3 );

    EXPECT_DOUBLE_EQ( 2.5, radius4 );
    EXPECT_DOUBLE_EQ( 2.5, base4 );
    EXPECT_DOUBLE_EQ( 1.5, stasis_radius4 );
    EXPECT_EQ( 150, stasis_ticks4 );
  }

#if 0 
  // Define unit test to verify ability to leverage the reporting 
  // service using the concrete message endpoint. This is more of a 
  // package integration test than a unit test, making sure that all 
  // of the pieces are playing together nicely within the package.
  TEST(BaseModelRepositoryTests, canStartAndStopOdometryReportingServiceWithEndpoint) {
    // Establish Context
    boost::shared_ptr<BaseModelRepository> base_model_repository =
      boost::shared_ptr<BaseModelRepository>(new BaseModelRepository());

    EncoderCountEndpoint* encoderCountEndpoint = new EncoderCountEndpoint();
    OdometryReportingService odometryReportingService(base_model_repository, *encoderCountEndpoint);
 
    // Give ROS time to fully initialize and for the base_model_repository to advertise
    sleep(1);
 
    // Act
    odometryReportingService.beginReporting();
    sleep(4);
    odometryReportingService.stopReporting();
 
    // Assert
    // See assertion note above from 
    // BaseModelRepositoryTests.canPublishOdometryWithEndpoint
  }
#endif
}
