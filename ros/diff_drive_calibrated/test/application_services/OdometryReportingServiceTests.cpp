// OdometryReportingServiceTests.cpp
 
#include <gtest/gtest.h>

#include "BaseModel.hpp"
#include "OdometryPublisherEndpointStub.hpp"
#include "MovementStatusPublisherEndpointStub.hpp"
#include "EncoderCountsSubscriberEndpointStub.hpp"
#include "OdometryReportingService.hpp"
 
using namespace differential_drive_application_services;
using namespace differential_drive_test_message_endpoints_test_doubles;
 
namespace differential_drive_test_application_services
{
// Define the unit test to verify ability to leverage the reporting service using the message endpoint stub
TEST(OdometryReportingServiceTests, canCanStartAndstopReports) 
{
  differential_drive::EncoderCounts encoder_counts;

  int received_odometry_1;
  int received_odometry_2;
  int received_odometry_3;
  int received_odometry_4;
  int received_odometry_5;
  int received_odometry_6;
  int received_odometry_7;

  int received_movement_status_1;
  int received_movement_status_2;
  int received_movement_status_3;
  int received_movement_status_4;
  int received_movement_status_5;
  int received_movement_status_6;
  int received_movement_status_7;

  // Establish Context
  boost::shared_ptr<OdometryPublisherEndpointStub> odometry_endpoint_stub =
      boost::shared_ptr<OdometryPublisherEndpointStub>( new OdometryPublisherEndpointStub() );

  boost::shared_ptr<MovementStatusPublisherEndpointStub> movement_status_endpoint_stub =
      boost::shared_ptr<MovementStatusPublisherEndpointStub>( new MovementStatusPublisherEndpointStub() );

  boost::shared_ptr<EncoderCountsSubscriberEndpointStub> encoder_counts_endpoint_stub =
      boost::shared_ptr<EncoderCountsSubscriberEndpointStub>( new EncoderCountsSubscriberEndpointStub() );

  boost::shared_ptr<differential_drive_core::BaseModel> base_model = 
      boost::shared_ptr<differential_drive_core::BaseModel>( new differential_drive_core::BaseModel() );
  
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

  usleep(25000);

  received_odometry_1 = odometry_endpoint_stub->_count_of_messages_published;
  received_movement_status_1 = movement_status_endpoint_stub->_count_of_messages_published;

  // Verify we don't re-subscribe
  odometry_reporting_service.startReporting();
  odometry_reporting_service.startReporting();
  odometry_reporting_service.startReporting();
  odometry_reporting_service.startReporting();

  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );

  usleep(25000);

  received_odometry_2 = odometry_endpoint_stub->_count_of_messages_published;
  received_movement_status_2 = movement_status_endpoint_stub->_count_of_messages_published;
  
  odometry_reporting_service.stopReportingOdometry();

  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );

  usleep(25000);

  received_odometry_3 = odometry_endpoint_stub->_count_of_messages_published;
  received_movement_status_3 = movement_status_endpoint_stub->_count_of_messages_published;
  
  odometry_reporting_service.startReportingOdometry();
  odometry_reporting_service.stopReportingMovementStatus();

  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );

  usleep(25000);

  received_odometry_4 = odometry_endpoint_stub->_count_of_messages_published;
  received_movement_status_4 = movement_status_endpoint_stub->_count_of_messages_published;
  
  odometry_reporting_service.stopReportingOdometry();

  // These will be stored in the queue, and processed when processing starts again
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );

  usleep(25000);

  received_odometry_5 = odometry_endpoint_stub->_count_of_messages_published;
  received_movement_status_5 = movement_status_endpoint_stub->_count_of_messages_published;
  
  odometry_reporting_service.startReporting();

  // This will cause us to unsubscribe from the encoder endpoint 
  odometry_reporting_service.stopProcessingEncoderCounts();

  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );

  usleep(25000);

  received_odometry_6 = odometry_endpoint_stub->_count_of_messages_published;
  received_movement_status_6 = movement_status_endpoint_stub->_count_of_messages_published;
  
  odometry_reporting_service.startProcessingEncoderCounts();

  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );

  usleep(25000);

  received_odometry_7 = odometry_endpoint_stub->_count_of_messages_published;
  received_movement_status_7 = movement_status_endpoint_stub->_count_of_messages_published;
  
  odometry_reporting_service.startReporting();
  odometry_reporting_service.stopProcessingEncoderCounts();
  // Assert

  // Arguably, this test is a bit light but makes sure odometry msgs are being pushed 
  // to the message endpoint for publication.
  EXPECT_EQ( 0, received_odometry_1 );
  EXPECT_EQ( 4, received_odometry_2 );
  EXPECT_EQ( 4, received_odometry_3 );
  EXPECT_EQ( 8, received_odometry_4 );
  EXPECT_EQ( 8, received_odometry_5 );
  EXPECT_EQ( 12, received_odometry_6 );
  EXPECT_EQ( 16, received_odometry_7 );

  EXPECT_EQ( 0, received_movement_status_1 );
  EXPECT_EQ( 4, received_movement_status_2 );
  EXPECT_EQ( 8, received_movement_status_3 );
  EXPECT_EQ( 8, received_movement_status_4 );
  EXPECT_EQ( 8, received_movement_status_5 );
  EXPECT_EQ( 12, received_movement_status_6 );
  EXPECT_EQ( 16, received_movement_status_7 );
}

// Define the unit test to verify ability to leverage the reporting service using the message endpoint stub
TEST(OdometryReportingServiceTests, canCanSendCountsToOdometryReportingService) 
{
  differential_drive::EncoderCounts encoder_counts;

  // Establish Context
  boost::shared_ptr<OdometryPublisherEndpointStub> odometry_endpoint_stub =
      boost::shared_ptr<OdometryPublisherEndpointStub>( new OdometryPublisherEndpointStub() );

  boost::shared_ptr<MovementStatusPublisherEndpointStub> movement_status_endpoint_stub =
      boost::shared_ptr<MovementStatusPublisherEndpointStub>( new MovementStatusPublisherEndpointStub() );

  boost::shared_ptr<EncoderCountsSubscriberEndpointStub> encoder_counts_endpoint_stub =
      boost::shared_ptr<EncoderCountsSubscriberEndpointStub>( new EncoderCountsSubscriberEndpointStub() );

  boost::shared_ptr<differential_drive_core::BaseModel> base_model = 
      boost::shared_ptr<differential_drive_core::BaseModel>( new differential_drive_core::BaseModel( 0.5, 100, 1.0 ) );
  
  OdometryReportingService odometry_reporting_service(  odometry_endpoint_stub, 
                                                        movement_status_endpoint_stub, 
                                                        encoder_counts_endpoint_stub,
                                                        base_model );

  // Act
  encoder_counts.left_count = 100;
  encoder_counts.right_count = 100;
  encoder_counts.dt_ms = 10;

  odometry_reporting_service.startReporting();

  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );
  encoder_counts_endpoint_stub->AddTicks( encoder_counts );

  usleep(25000);

  // Assert

  // Arguably, this test is a bit light but makes sure odometry msgs are being pushed 
  // to the message endpoint for publication.
  EXPECT_EQ( 4, odometry_endpoint_stub->_count_of_messages_published );
  EXPECT_FLOAT_EQ( 4.0 * M_PI, odometry_endpoint_stub->_x );
}
}
