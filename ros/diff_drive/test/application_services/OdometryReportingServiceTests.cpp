// OdometryReportingServiceTests.cpp
 
#include <gtest/gtest.h>

#include "BaseModel.hpp"
#include "OdometryEndpointStub.hpp"
#include "MovementStatusEndpointStub.hpp"
#include "EncoderCountsEndpointStub.hpp"
#include "OdometryReportingService.hpp"
 
using namespace diff_drive_application_services;
using namespace diff_drive_test_message_endpoints_test_doubles;
 
namespace diff_drive_test_application_services
{
  // Define the unit test to verify ability to leverage the reporting service using the message endpoint stub
  TEST(OdometryReportingServiceTests, canCanStartAndStopReports) 
  {
    diff_drive::EncoderCounts encoder_counts;

    int received_odometry_1;
    int received_odometry_2;
    int received_odometry_3;
    int received_odometry_4;
    int received_odometry_5;

    int received_movement_status_1;
    int received_movement_status_2;
    int received_movement_status_3;
    int received_movement_status_4;
    int received_movement_status_5;

    bool subscribed_1;
    bool subscribed_2;
    bool subscribed_3;

    // Establish Context
    boost::shared_ptr<OdometryEndpointStub> odometry_endpoint_stub =
        boost::shared_ptr<OdometryEndpointStub>( new OdometryEndpointStub() );

    boost::shared_ptr<MovementStatusEndpointStub> movement_status_endpoint_stub =
        boost::shared_ptr<MovementStatusEndpointStub>( new MovementStatusEndpointStub() );

    boost::shared_ptr<EncoderCountsEndpointStub> encoder_counts_endpoint_stub =
        boost::shared_ptr<EncoderCountsEndpointStub>( new EncoderCountsEndpointStub() );

    boost::shared_ptr<diff_drive_core::BaseModel> base_model = 
        boost::shared_ptr<diff_drive_core::BaseModel>( new diff_drive_core::BaseModel() );
    
    OdometryReportingService odometry_reporting_service(  odometry_endpoint_stub, 
                                                          movement_status_endpoint_stub, 
                                                          encoder_counts_endpoint_stub,
                                                          base_model );

    // Act
    encoder_counts.left_count = 100;
    encoder_counts.right_count = 100;

    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
 
    received_odometry_1 = odometry_endpoint_stub->_count_of_messages_published;
    received_movement_status_1 = movement_status_endpoint_stub->_count_of_messages_published;
    subscribed_1 = encoder_counts_endpoint_stub->IsSubscribed();    
 
    odometry_reporting_service.BeginReporting();

    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
 
    received_odometry_2 = odometry_endpoint_stub->_count_of_messages_published;
    received_movement_status_2 = movement_status_endpoint_stub->_count_of_messages_published;
    
    odometry_reporting_service.StopReportingOdometry();

    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
 
    received_odometry_3 = odometry_endpoint_stub->_count_of_messages_published;
    received_movement_status_3 = movement_status_endpoint_stub->_count_of_messages_published;
    subscribed_2 = encoder_counts_endpoint_stub->IsSubscribed();    
    
    odometry_reporting_service.BeginReportingOdometry();
    odometry_reporting_service.StopReportingMovementStatus();

    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
 
    received_odometry_4 = odometry_endpoint_stub->_count_of_messages_published;
    received_movement_status_4 = movement_status_endpoint_stub->_count_of_messages_published;
    
    odometry_reporting_service.StopReportingOdometry();

    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
 
    received_odometry_5 = odometry_endpoint_stub->_count_of_messages_published;
    received_movement_status_5 = movement_status_endpoint_stub->_count_of_messages_published;
    subscribed_3 = encoder_counts_endpoint_stub->IsSubscribed();    
    
    // Assert
 
    // Arguably, this test is a bit light but makes sure odometry msgs are being pushed 
    // to the message endpoint for publication.
    EXPECT_EQ( 0, received_odometry_1 );
    EXPECT_EQ( 4, received_odometry_2 );
    EXPECT_EQ( 4, received_odometry_3 );
    EXPECT_EQ( 8, received_odometry_4 );
    EXPECT_EQ( 8, received_odometry_5 );

    EXPECT_EQ( 0, received_movement_status_1 );
    EXPECT_EQ( 4, received_movement_status_2 );
    EXPECT_EQ( 8, received_movement_status_3 );
    EXPECT_EQ( 8, received_movement_status_4 );
    EXPECT_EQ( 8, received_movement_status_5 );

    EXPECT_EQ( false, subscribed_1 );
    EXPECT_EQ( true, subscribed_2 );
    EXPECT_EQ( false, subscribed_3 );

  }

  // Define the unit test to verify ability to leverage the reporting service using the message endpoint stub
  TEST(OdometryReportingServiceTests, canCanSendCountsToOdometryReportingService) 
  {
    diff_drive::EncoderCounts encoder_counts;

    // Establish Context
    boost::shared_ptr<OdometryEndpointStub> odometry_endpoint_stub =
        boost::shared_ptr<OdometryEndpointStub>( new OdometryEndpointStub() );

    boost::shared_ptr<MovementStatusEndpointStub> movement_status_endpoint_stub =
        boost::shared_ptr<MovementStatusEndpointStub>( new MovementStatusEndpointStub() );

    boost::shared_ptr<EncoderCountsEndpointStub> encoder_counts_endpoint_stub =
        boost::shared_ptr<EncoderCountsEndpointStub>( new EncoderCountsEndpointStub() );

    boost::shared_ptr<diff_drive_core::BaseModel> base_model = 
        boost::shared_ptr<diff_drive_core::BaseModel>( new diff_drive_core::BaseModel( 0.5, 100, 1.0 ) );
    
    OdometryReportingService odometry_reporting_service(  odometry_endpoint_stub, 
                                                          movement_status_endpoint_stub, 
                                                          encoder_counts_endpoint_stub,
                                                          base_model );

    // Act
    encoder_counts.left_count = 100;
    encoder_counts.right_count = 100;
    encoder_counts.dt_ms = 10;

    odometry_reporting_service.BeginReporting();

    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
    encoder_counts_endpoint_stub->AddTicks( encoder_counts );
 
    // Assert
 
    // Arguably, this test is a bit light but makes sure odometry msgs are being pushed 
    // to the message endpoint for publication.
    EXPECT_EQ( 4, odometry_endpoint_stub->_count_of_messages_published );
    EXPECT_FLOAT_EQ( 4.0 * M_PI, odometry_endpoint_stub->_x );
  }
}
