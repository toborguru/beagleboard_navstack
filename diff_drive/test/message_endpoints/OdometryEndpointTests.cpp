// OdometryEndpointTests.cpp
 
#include <gtest/gtest.h>
#include "nav_msgs/Odometry.h"
#include "OdometryEndpoint.hpp"
#include "EncoderCountEndpoint.hpp"
#include "OdometryReportingService.hpp"
 
using namespace diff_drive_application_services;
using namespace diff_drive_message_endpoints;
 
namespace diff_drive_test_message_endpoints
{
  // Define unit test to verify ability to publish laser scans 
  // to ROS using the concrete message endpoint.
  TEST(OdometryEndpointTests, canPublishOdometryWithEndpoint) {
    // Establish Context
    OdometryEndpoint odometryEndpoint;
    nav_msgs::Odometry odometry;
 
    // Give ROS time to fully initialize and for the odometryEndpoint to advertise
    sleep(1);
 
    // Act
    odometry.header.seq = 1;
    odometryEndpoint.publish(odometry);
 
    odometry.header.seq = 2;
    odometryEndpoint.publish(odometry);
 
    // Assert
    // Nothing to assert other than using terminal windows to 
    // watch publication activity. Alternatively, for better testing, 
    // you could create a subscriber and subscribe to the reports 
    // You could then track how many reports were received and 
    // assert checks, accordingly.
  }
 
  // Define unit test to verify ability to leverage the reporting 
  // service using the concrete message endpoint. This is more of a 
  // package integration test than a unit test, making sure that all 
  // of the pieces are playing together nicely within the package.
  TEST(OdometryEndpointTests, canStartAndStopOdometryReportingServiceWithEndpoint) {
    // Establish Context
    boost::shared_ptr<OdometryEndpoint> odometryEndpoint =
      boost::shared_ptr<OdometryEndpoint>(new OdometryEndpoint());

    EncoderCountEndpoint* encoderCountEndpoint = new EncoderCountEndpoint();
    OdometryReportingService odometryReportingService(odometryEndpoint, *encoderCountEndpoint);
 
    // Give ROS time to fully initialize and for the odometryEndpoint to advertise
    sleep(1);
 
    // Act
    odometryReportingService.beginReporting();
    sleep(4);
    odometryReportingService.stopReporting();
 
    // Assert
    // See assertion note above from 
    // OdometryEndpointTests.canPublishOdometryWithEndpoint
  }
}
