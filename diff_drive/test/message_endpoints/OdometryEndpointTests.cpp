// OdometryEndpointTests.cpp
 
#include <gtest/gtest.h>

#include "nav_msgs/Odometry.h"

//#include "EncoderCountsEndpoint.hpp"
//#include "OdometryReportingService.hpp"
#include "OdometryEndpoint.hpp"
 
using namespace diff_drive_message_endpoints;
//using namespace diff_drive_application_services;
 
namespace diff_drive_test_message_endpoints
{
// Define unit test to verify ability to publish laser scans 
// to ROS using the concrete message endpoint.
TEST(OdometryEndpointTests, canPublishOdometryWithEndpoint) 
{
  // Establish Context
  OdometryEndpoint odometry_endpoint;
  nav_msgs::Odometry odometry;

  // Give ROS time to fully initialize and for the odometry_endpoint to advertise
  sleep(1);

  // Act
  odometry.header.seq = 1;
  odometry.pose.pose.position.x = 0;
  odometry.pose.pose.position.y = 5;
  odometry_endpoint.Publish( odometry );

  odometry.header.seq = 2;
  odometry.pose.pose.position.x = 15;
  odometry.pose.pose.position.y = 25;
  odometry_endpoint.Publish( odometry );

  // Assert
  // Nothing to assert other than using terminal windows to 
  // watch publication activity. Alternatively, for better testing, 
  // you could create a subscriber and subscribe to the reports 
  // You could then track how many reports were received and 
  // assert checks, accordingly.
}
#if 0 
// Define unit test to verify ability to leverage the reporting 
// service using the concrete message endpoint. This is more of a 
// package integration test than a unit test, making sure that all 
// of the pieces are playing together nicely within the package.
TEST(OdometryEndpointTests, canStartAndStopOdometryReportingServiceWithEndpoint) {
  // Establish Context
  boost::shared_ptr<OdometryEndpoint> odometry_endpoint =
    boost::shared_ptr<OdometryEndpoint>(new OdometryEndpoint());

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
  // OdometryEndpointTests.canPublishOdometryWithEndpoint
}
#endif
}
