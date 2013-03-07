// OdometryPublisherEndpointTests.cpp
 
#include <gtest/gtest.h>

#include "nav_msgs/Odometry.h"

#include "OdometryPublisherEndpoint.hpp"
 
using namespace differential_drive_message_endpoints;
//using namespace nav_msgs_application_services;
 
namespace differential_drive_test_message_endpoints
{
  static int _count;
  static nav_msgs::Odometry _odometry;

  void IncrementCount( const nav_msgs::Odometry::ConstPtr& msg )
  {
    ++_count;
    _odometry.pose.pose.position.x = msg->pose.pose.position.x;
    _odometry.pose.pose.position.y = msg->pose.pose.position.y;
  }

  // Define unit test to verify ability to publish laser scans 
  // to ROS using the concrete message endpoint.
  TEST(OdometryPublisherEndpointTests, canPublishOdometryWithEndpoint) 
  {
    // Establish Context
    std::string name("odometry_endpoint_tester");
    int argc = 0;
    ros::init( argc, (char**)NULL, name );

    int count1;
    int x1;
    int y1;

    int count2;
    int x2;
    int y2;

    OdometryPublisherEndpoint odometry_endpoint;
    nav_msgs::Odometry odometry;

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe( "odometry", 32, IncrementCount );

    _count = 0;
 
    // Give ROS time to fully initialize and for the odometry_endpoint to advertise
    sleep(1);
 
    // Act
    odometry.pose.pose.position.x = 10;
    odometry.pose.pose.position.y = 5;
    odometry_endpoint.Publish(odometry);
    odometry_endpoint.Publish(odometry);
    odometry_endpoint.Publish(odometry);
    odometry_endpoint.Publish(odometry);
    usleep( 25000 );
    ros::spinOnce();

    count1 = _count;
    x1 = _odometry.pose.pose.position.x;
    y1 = _odometry.pose.pose.position.y;

    odometry.pose.pose.position.x = 15;
    odometry.pose.pose.position.y = 25;
    odometry_endpoint.Publish(odometry);
    odometry_endpoint.Publish(odometry);
    usleep( 25000 );
    ros::spinOnce();
 
    count2 = _count;
    x2 = _odometry.pose.pose.position.x;
    y2 = _odometry.pose.pose.position.y;

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
  TEST(OdometryPublisherEndpointTests, canStartAndStopOdometryReportingServiceWithEndpoint) {
    // Establish Context
    boost::shared_ptr<OdometryPublisherEndpoint> odometry_endpoint =
      boost::shared_ptr<OdometryPublisherEndpoint>(new OdometryPublisherEndpoint());

    EncoderCountEndpoint* encoderCountEndpoint = new EncoderCountEndpoint();
    OdometryReportingService odometryReportingService(odometry_endpoint, *encoderCountEndpoint);
 
    // Give ROS time to fully initialize and for the odometry_endpoint to advertise
    sleep(1);
 
    // Act
    odometryReportingService.beginReporting();
    sleep(4);
    odometryReportingService.stopReporting();
 
    // Assert
    // See assertion note above from 
    // OdometryPublisherEndpointTests.canPublishOdometryWithEndpoint
  }
#endif
}
