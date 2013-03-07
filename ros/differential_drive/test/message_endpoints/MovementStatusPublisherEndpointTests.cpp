// MovementStatusPublisherEndpointTests.cpp
 
#include <gtest/gtest.h>

#include "differential_drive/MovementStatus.h"

#include "MovementStatusPublisherEndpoint.hpp"
 
using namespace differential_drive_message_endpoints;
//using namespace differential_drive_application_services;
 
namespace differential_drive_test_message_endpoints
{
  static int _count;
  static differential_drive::MovementStatus _movement_status;

  void IncrementCount( const differential_drive::MovementStatus::ConstPtr& msg )
  {
    ++_count;
    _movement_status.linear_velocity = msg->linear_velocity;
    _movement_status.stasis_velocity = msg->stasis_velocity;
  }

  // Define unit test to verify ability to publish laser scans 
  // to ROS using the concrete message endpoint.
  TEST(MovementStatusPublisherEndpointTests, canPublishMovementStatusWithEndpoint) 
  {
    // Establish Context
    std::string name("movement_status_endpoint_tester");
    int argc = 0;
    ros::init( argc, (char**)NULL, name );

    int count1;
    int x1;
    int y1;

    int count2;
    int x2;
    int y2;

    MovementStatusPublisherEndpoint movement_status_endpoint;
    differential_drive::MovementStatus movement_status;

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe( "movement_status", 32, IncrementCount );

    _count = 0;
 
    // Give ROS time to fully initialize and for the movement_status_endpoint to advertise
    sleep(1);
 
    // Act
    movement_status.linear_velocity = 10;
    movement_status.stasis_velocity = 5;
    movement_status_endpoint.Publish(movement_status);
    movement_status_endpoint.Publish(movement_status);
    movement_status_endpoint.Publish(movement_status);
    movement_status_endpoint.Publish(movement_status);
    usleep( 25000 );
    ros::spinOnce();

    count1 = _count;
    x1 = _movement_status.linear_velocity;
    y1 = _movement_status.stasis_velocity;

    movement_status.linear_velocity = 15;
    movement_status.stasis_velocity = 25;
    movement_status_endpoint.Publish(movement_status);
    movement_status_endpoint.Publish(movement_status);
    usleep( 25000 );
    ros::spinOnce();
 
    count2 = _count;
    x2 = _movement_status.linear_velocity;
    y2 = _movement_status.stasis_velocity;

    // Assert
    // Nothing to assert other than using terminal windows to 
    // watch publication activity. Alternatively, for better testing, 
    // you could create a subscriber and subscribe to the reports 
    // You could then track how many reports were received and 
    // assert checks, accordingly.
    EXPECT_EQ( 4, count1 );
    EXPECT_EQ( 10, x1 );
    EXPECT_EQ( 5, y1 );

    EXPECT_EQ( 6, count2 );
    EXPECT_EQ( 15, x2 );
    EXPECT_EQ( 25, y2 );
  }
#if 0 
  // Define unit test to verify ability to leverage the reporting 
  // service using the concrete message endpoint. This is more of a 
  // package integration test than a unit test, making sure that all 
  // of the pieces are playing together nicely within the package.
  TEST(MovementStatusPublisherEndpointTests, canStartAndStopMovementStatusReportingServiceWithEndpoint) {
    // Establish Context
    boost::shared_ptr<MovementStatusPublisherEndpoint> movement_status_endpoint =
      boost::shared_ptr<MovementStatusPublisherEndpoint>(new MovementStatusPublisherEndpoint());

    EncoderCountEndpoint* encoderCountEndpoint = new EncoderCountEndpoint();
    MovementStatusReportingService movement_statusReportingService(movement_status_endpoint, *encoderCountEndpoint);
 
    // Give ROS time to fully initialize and for the movement_status_endpoint to advertise
    sleep(1);
 
    // Act
    movement_statusReportingService.beginReporting();
    sleep(4);
    movement_statusReportingService.stopReporting();
 
    // Assert
    // See assertion note above from 
    // MovementStatusPublisherEndpointTests.canPublishMovementStatusWithEndpoint
  }
#endif
}
