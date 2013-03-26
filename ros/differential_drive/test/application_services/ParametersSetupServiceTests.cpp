// OdometryReportingServiceTests.cpp
 
#include <gtest/gtest.h>

#include "BaseModel.hpp"
#include "ParametersSetupService.hpp"
#include "DifferentialParametersRepositoryStub.hpp"
#include "OdometryPublisherEndpointStub.hpp"
#include "MovementStatusPublisherEndpointStub.hpp"
#include "EncoderCountsSubscriberEndpointStub.hpp"
#include "OdometryReportingService.hpp"
 
using namespace differential_drive_application_services;
using namespace differential_drive_test_data_repositories_test_doubles;
using namespace differential_drive_test_message_endpoints_test_doubles;
 
namespace differential_drive_test_application_services
{
// Define the unit test to verify ability to update the data using the data repository stub
TEST(ParametersSetupServiceTests, canSetupAndUpdateBaseModel)
{
  int wheel_ticks1;
  int wheel_ticks2;
  int wheel_ticks3;

  // Establish Context
  // Setup the Odometry Service
  boost::shared_ptr<OdometryPublisherEndpointStub> odometry_endpoint_stub =
      boost::shared_ptr<OdometryPublisherEndpointStub>( new OdometryPublisherEndpointStub() );

  boost::shared_ptr<MovementStatusPublisherEndpointStub> movement_status_endpoint_stub =
      boost::shared_ptr<MovementStatusPublisherEndpointStub>( new MovementStatusPublisherEndpointStub() );

  boost::shared_ptr<EncoderCountsSubscriberEndpointStub> encoder_counts_endpoint_stub =
      boost::shared_ptr<EncoderCountsSubscriberEndpointStub>( new EncoderCountsSubscriberEndpointStub() );

  boost::shared_ptr<differential_drive_core::BaseModel> base_model =
      boost::shared_ptr<differential_drive_core::BaseModel>( new differential_drive_core::BaseModel() );

  boost::shared_ptr<OdometryReportingService> odometry_reporting_service =
      boost::shared_ptr<OdometryReportingService>( new OdometryReportingService(  odometry_endpoint_stub,
                                                                                  movement_status_endpoint_stub,
                                                                                  encoder_counts_endpoint_stub,
                                                                                  base_model ) );
  // Setup the data repo

  boost::shared_ptr<DifferentialParametersRepositoryStub> parameters_repository_stub =
          boost::shared_ptr<DifferentialParametersRepositoryStub>( new DifferentialParametersRepositoryStub() );

  // Setup the Parameter Service
  ParametersSetupService base_model_service( parameters_repository_stub, base_model, odometry_reporting_service );

  //base_model = base_model_service.GetBaseModel();

  // Act
  wheel_ticks1 = base_model->GetWheelTicks();

  parameters_repository_stub->base_geometry.wheel_ticks = 200;
  wheel_ticks2 = base_model->GetWheelTicks();
  
  //base_model_service.Update();
  wheel_ticks3 = base_model->GetWheelTicks();

  // Assert
  EXPECT_EQ( 0, wheel_ticks1);
  EXPECT_EQ( 0, wheel_ticks2);
  EXPECT_EQ( 200, wheel_ticks3);
}
}
