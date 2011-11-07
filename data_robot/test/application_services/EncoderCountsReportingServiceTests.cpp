// EncoderCountsReportingServiceTests.cpp
 
#include <gtest/gtest.h>
#include "EncoderCountsEndpointStub.hpp"
#include "ExternalBusEndpointStub.hpp"
#include "EncoderCountsReportingService.hpp"
 
using namespace data_robot_application_services;
using namespace data_robot_test_message_endpoints_test_doubles;
 
namespace data_robot_test_application_services
{
// Define the unit test to verify ability to leverage the reporting service using the messege endpoint stub
TEST(EncoderCountsReportingServiceTests, canStartAndStopEncoderCountsReportingService) 
{
  // Establish Context
  boost::shared_ptr<EncoderCountsEndpointStub> encoder_counts_endpoint_stub =
      boost::shared_ptr<EncoderCountsEndpointStub>( new EncoderCountsEndpointStub() );

  boost::shared_ptr<ExternalBusEndpointStub> external_bus_endpoint_stub = 
      boost::shared_ptr<ExternalBusEndpointStub>( new ExternalBusEndpointStub() );

  EncoderCountsReportingService encoder_counts_reporting_service( encoder_counts_endpoint_stub, 
                                                                  external_bus_endpoint_stub );

  // Act
  encoder_counts_reporting_service.BeginReporting();
  sleep(2);
  encoder_counts_reporting_service.StopReporting();

  // Assert

  // Since we just ran the reader for 2 seconds, we should expect a few readings.
  // Arguably, this test is a bit light but makes sure laser scans are being pushed 
  // to the message endpoint for publication.
  EXPECT_EQ( 20, encoder_counts_endpoint_stub->_count_of_encoder_counts_published );
}
}
