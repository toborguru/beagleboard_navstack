// EncoderCountEndpointTests.cpp
 
#include <gtest/gtest.h>
#include "EncoderCountEndpoint.hpp"
#include "OdometryEndpoint.hpp"
#include "OdometryReportingService.hpp"
 
using namespace diff_drive_application_services;
using namespace diff_drive_message_endpoints;
 
namespace diff_drive_test_message_endpoints
{
  // Define unit test to verify ability to publish laser scans 
  // to ROS using the concrete message endpoint.
  TEST(EncoderCountEndpointTests, canPublishEncoderCountWithEndpoint) {
    // Establish Context
    EncoderCountEndpoint encoderCountEndpoint;

    diff_drive_core::EncoderCount_t encoderCount;
 
    // Give ROS time to fully initialize and for the encoderCountEndpoint to advertise
    sleep(1);
 
    // Act
    encoderCount = encoderCountEndpoint.RequestEncoderCounts();
 
    encoderCount = encoderCountEndpoint.RequestEncoderCounts();
 
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
  TEST(EncoderCountEndpointTests, canStartAndStopEncoderCountReportingServiceWithEndpoint) {
    // Establish Context
    boost::shared_ptr<OdometryEndpoint> odometryEndpoint =
      boost::shared_ptr<OdometryEndpoint>(new OdometryEndpoint());

    EncoderCountEndpoint* encoderCountEndpoint = new EncoderCountEndpoint();

    OdometryReportingService OdometryReportingService(odometryEndpoint, *encoderCountEndpoint);
 
    // Give ROS time to fully initialize and for the encoderCountEndpoint to advertise
    sleep(1);
 
    // Act
    OdometryReportingService.beginReporting();
    sleep(4);
    OdometryReportingService.stopReporting();
 
    // Assert
    // See assertion note above from 
    // EncoderCountEndpointTests.canPublishEncoderCountWithEndpoint
  }
}
