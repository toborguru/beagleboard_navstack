// OdometryReportingServiceTests.cpp
 
#include <gtest/gtest.h>
#include "OdometryEndpointStub.hpp"
#include "EncoderCountEndpointStub.hpp"
#include "OdometryReportingService.hpp"
 
using namespace diff_drive_application_services;
using namespace diff_drive_test_message_endpoints_test_doubles;
 
namespace diff_drive_test_application_services
{
  // Define the unit test to verify ability to leverage the reporting service using the message endpoint stub
  TEST(OdometryReportingServiceTests, canStartAndStopOdometryReportingService) {
    // Establish Context
    boost::shared_ptr<OdometryEndpointStub> odometryEndpointStub =
      boost::shared_ptr<OdometryEndpointStub>(new OdometryEndpointStub());

    EncoderCountEndpointStub* encoderCountEndpointStub = new EncoderCountEndpointStub();
    OdometryReportingService odometryReportingService(odometryEndpointStub, *encoderCountEndpointStub);
 
    // Act
    odometryReportingService.beginReporting();
    sleep(5);
    odometryReportingService.stopReporting();
 
    // Assert
 
    // Since we just ran the reader for 2 seconds, we should expect a few readings.
    // Arguably, this test is a bit light but makes sure laser scans are being pushed 
    // to the message endpoint for publication.
    EXPECT_TRUE(odometryEndpointStub->countOfOdometrysPublished > 0 &&
      odometryEndpointStub->countOfOdometrysPublished <= 10);
  }
}
