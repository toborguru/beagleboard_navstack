// TickVelocityEndpointTests.cpp
 
#include <gtest/gtest.h>

#include "diff_drive/TickVelocity.h"

//#include "EncoderCountsEndpoint.hpp"
//#include "TickVelocityReportingService.hpp"
#include "TickVelocityEndpoint.hpp"
 
using namespace diff_drive_message_endpoints;
//using namespace diff_drive_application_services;
 
namespace diff_drive_test_message_endpoints
{
  // Define unit test to verify ability to publish laser scans 
  // to ROS using the concrete message endpoint.
  TEST(TickVelocityEndpointTests, canPublishTickVelocityWithEndpoint) {
    // Establish Context
    TickVelocityEndpoint tick_velocity_endpoint;
    diff_drive::TickVelocity tick_velocity;
 
    // Give ROS time to fully initialize and for the tick_velocity_endpoint to advertise
    sleep(1);
 
    // Act
    tick_velocity.linear_ticks_sec = 10;
    tick_velocity.angular_ticks_sec = 5;
    tick_velocity_endpoint.Publish(tick_velocity);
 
    tick_velocity.linear_ticks_sec = 15;
    tick_velocity.angular_ticks_sec = 25;
    tick_velocity_endpoint.Publish(tick_velocity);
 
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
  TEST(TickVelocityEndpointTests, canStartAndStopTickVelocityReportingServiceWithEndpoint) {
    // Establish Context
    boost::shared_ptr<TickVelocityEndpoint> tick_velocity_endpoint =
      boost::shared_ptr<TickVelocityEndpoint>(new TickVelocityEndpoint());

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
    // TickVelocityEndpointTests.canPublishTickVelocityWithEndpoint
  }
#endif
}
