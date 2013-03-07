// TickVelocityPublisherEndpointTests.cpp
 
#include <gtest/gtest.h>

#include "differential_drive/TickVelocity.h"

#include "TickVelocityPublisherEndpoint.hpp"
 
using namespace differential_drive_message_endpoints;
//using namespace differential_drive_application_services;
 
namespace differential_drive_test_message_endpoints
{
  static int _count;
  static differential_drive::TickVelocity _tick_velocity;

  void IncrementCount( const differential_drive::TickVelocity::ConstPtr& msg )
  {
    ++_count;
    _tick_velocity.linear_ticks_sec = msg->linear_ticks_sec;
    _tick_velocity.angular_ticks_sec = msg->angular_ticks_sec;
  }

  // Define unit test to verify ability to publish laser scans 
  // to ROS using the concrete message endpoint.
  TEST(TickVelocityPublisherEndpointTests, canPublishTickVelocityWithEndpoint) 
  {
    // Establish Context
    std::string name("tick_velocity_endpoint_tester");
    int argc = 0;
    ros::init( argc, (char**)NULL, name );

    int count1;
    int linear1;
    int angular1;

    int count2;
    int linear2;
    int angular2;

    TickVelocityPublisherEndpoint tick_velocity_endpoint;
    differential_drive::TickVelocity tick_velocity;

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe( "tick_velocity", 32, IncrementCount );

    _count = 0;
 
    // Give ROS time to fully initialize and for the tick_velocity_endpoint to advertise
    sleep(1);
 
    // Act
    tick_velocity.linear_ticks_sec = 10;
    tick_velocity.angular_ticks_sec = 5;
    tick_velocity_endpoint.Publish(tick_velocity);
    tick_velocity_endpoint.Publish(tick_velocity);
    tick_velocity_endpoint.Publish(tick_velocity);
    tick_velocity_endpoint.Publish(tick_velocity);
    usleep( 25000 );
    ros::spinOnce();

    count1 = _count;
    linear1 = _tick_velocity.linear_ticks_sec;
    angular1 = _tick_velocity.angular_ticks_sec;

    tick_velocity.linear_ticks_sec = 15;
    tick_velocity.angular_ticks_sec = 25;
    tick_velocity_endpoint.Publish(tick_velocity);
    tick_velocity_endpoint.Publish(tick_velocity);
    usleep( 25000 );
    ros::spinOnce();
 
    count2 = _count;
    linear2 = _tick_velocity.linear_ticks_sec;
    angular2 = _tick_velocity.angular_ticks_sec;

    // Assert
    EXPECT_EQ( 4, count1 );
    EXPECT_EQ( 10, linear1 );
    EXPECT_EQ( 5, angular1 );

    EXPECT_EQ( 6, count2 );
    EXPECT_EQ( 15, linear2 );
    EXPECT_EQ( 25, angular2 );
  }
#if 0 
  // Define unit test to verify ability to leverage the reporting 
  // service using the concrete message endpoint. This is more of a 
  // package integration test than a unit test, making sure that all 
  // of the pieces are playing together nicely within the package.
  TEST(TickVelocityPublisherEndpointTests, canStartAndStopTickVelocityReportingServiceWithEndpoint) {
    // Establish Context
    boost::shared_ptr<TickVelocityPublisherEndpoint> tick_velocity_endpoint =
      boost::shared_ptr<TickVelocityPublisherEndpoint>(new TickVelocityPublisherEndpoint());

    EncoderCountEndpoint* encoderCountEndpoint = new EncoderCountEndpoint();
    TickVelocityReportingService tick_velocityReportingService(tick_velocity_endpoint, *encoderCountEndpoint);
 
    // Give ROS time to fully initialize and for the tick_velocity_endpoint to advertise
    sleep(1);
 
    // Act
    tick_velocityReportingService.beginReporting();
    sleep(4);
    tick_velocityReportingService.stopReporting();
 
    // Assert
    // See assertion note above from 
    // TickVelocityPublisherEndpointTests.canPublishTickVelocityWithEndpoint
  }
#endif
}
