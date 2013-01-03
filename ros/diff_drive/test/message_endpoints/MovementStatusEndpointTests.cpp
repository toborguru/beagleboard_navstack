// MovementStatusEndpointTests.cpp
 
#include <gtest/gtest.h>

#include "diff_drive/MovementStatus.h"

#include "MovementStatusEndpoint.hpp"
 
using namespace diff_drive_message_endpoints;
//using namespace diff_drive_application_services;
 
namespace diff_drive_test_message_endpoints
{
  static int _count;
  static diff_drive::MovementStatus _movement_status;

  void IncrementCount( const diff_drive::MovementStatus::ConstPtr& msg )
  {
    _count++;
    _movement_status.linear_velocity = msg->linear_velocity;
  }

  // Define unit test to verify ability to publish laser scans 
  // to ROS using the concrete message endpoint.
  TEST(MovementStatusEndpointTests, canPublishMovementStatusWithEndpoint) 
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

    MovementStatusEndpoint movement_status_endpoint;
    diff_drive::MovementStatus movement_status;

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
    x1 = _movement_status.pose.pose.position.x;
    y1 = _movement_status.pose.pose.position.y;

    movement_status.pose.pose.position.x = 15;
    movement_status.pose.pose.position.y = 25;
    movement_status_endpoint.Publish(movement_status);
    movement_status_endpoint.Publish(movement_status);
    usleep( 25000 );
    ros::spinOnce();
 
    count2 = _count;
    x2 = _movement_status.pose.pose.position.x;
    y2 = _movement_status.pose.pose.position.y;

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
  TEST(MovementStatusEndpointTests, canStartAndStopMovementStatusReportingServiceWithEndpoint) {
    // Establish Context
    boost::shared_ptr<MovementStatusEndpoint> movement_status_endpoint =
      boost::shared_ptr<MovementStatusEndpoint>(new MovementStatusEndpoint());

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
    // MovementStatusEndpointTests.canPublishMovementStatusWithEndpoint
  }
#endif
}
