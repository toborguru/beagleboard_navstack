// OdometryReportingServiceTests.cpp
 
#include <gtest/gtest.h>

#include "tf/transform_datatypes.h"

#include "BaseModel.hpp"
#include "OdometryEndpointStub.hpp"
#include "EncoderCountsEndpointStub.hpp"
#include "OdometryReportingService.hpp"
 
using namespace diff_drive_application_services;
using namespace diff_drive_test_message_endpoints_test_doubles;
 
namespace diff_drive_test_application_services
{
  // Define the unit test to verify ability to leverage the reporting service using the message endpoint stub
  TEST(OdometryReportingServiceTests, canCanStartAndStopOdometryReports) 
  {
    diff_drive::EncoderCounts encoder_counts;

    int received1;
    int received2;
    int received3;

    // Establish Context
    boost::shared_ptr<OdometryEndpointStub> odometry_endpoint_stub =
        boost::shared_ptr<OdometryEndpointStub>( new OdometryEndpointStub() );

    boost::shared_ptr<EncoderCountsEndpointStub> encoder_counts_endpoint_stub =
        boost::shared_ptr<EncoderCountsEndpointStub>( new EncoderCountsEndpointStub() );

    boost::shared_ptr<diff_drive_core::BaseModel> base_model = 
        boost::shared_ptr<diff_drive_core::BaseModel>( new diff_drive_core::BaseModel() );
    
    OdometryReportingService odometry_reporting_service(  odometry_endpoint_stub, 
                                                          encoder_counts_endpoint_stub,
                                                          base_model );

    // Act
    encoder_counts.left_count = 100;
    encoder_counts.right_count = 100;

    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
 
    received1 = odometry_endpoint_stub->_count_of_odometrys_published;
   
    odometry_reporting_service.BeginReporting();

    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
 
    received2 = odometry_endpoint_stub->_count_of_odometrys_published;
    
    odometry_reporting_service.StopReporting();

    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
 
    received3 = odometry_endpoint_stub->_count_of_odometrys_published;
    
    // Assert
 
    // Arguably, this test is a bit light but makes sure odometry msgs are being pushed 
    // to the message endpoint for publication.
    EXPECT_EQ( 0, received1 );
    EXPECT_EQ( 4, received2 );
    EXPECT_EQ( 4, received3 );
  }

  // Define the unit test to verify ability to leverage the reporting service using the message endpoint stub
  TEST(OdometryReportingServiceTests, canCanSendCountsToOdometryReportingService) 
  {
    diff_drive::EncoderCounts encoder_counts;

    // Establish Context
    boost::shared_ptr<OdometryEndpointStub> odometry_endpoint_stub =
        boost::shared_ptr<OdometryEndpointStub>( new OdometryEndpointStub() );

    boost::shared_ptr<EncoderCountsEndpointStub> encoder_counts_endpoint_stub =
        boost::shared_ptr<EncoderCountsEndpointStub>( new EncoderCountsEndpointStub() );

    boost::shared_ptr<diff_drive_core::BaseModel> base_model = 
        boost::shared_ptr<diff_drive_core::BaseModel>( new diff_drive_core::BaseModel() );
    
    OdometryReportingService odometry_reporting_service(  odometry_endpoint_stub, 
                                                          encoder_counts_endpoint_stub,
                                                          base_model );

    // Act
    encoder_counts.left_count = 100;
    encoder_counts.right_count = 100;

    odometry_reporting_service.BeginReporting();

    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
 
    // Assert
 
    // Arguably, this test is a bit light but makes sure odometry msgs are being pushed 
    // to the message endpoint for publication.
    EXPECT_EQ( 4, odometry_endpoint_stub->_count_of_odometrys_published );
    EXPECT_FLOAT_EQ( 4.0 * M_PI, odometry_endpoint_stub->_x );
  }
}
