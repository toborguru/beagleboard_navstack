// OdometryReportingServiceTests.cpp
 
#include <gtest/gtest.h>

#include "BaseModel.hpp"
#include "BaseModelSetupService.hpp"
#include "BaseModelRepositoryStub.hpp"
 
using namespace differential_drive_application_services;
using namespace differential_drive_test_data_repositories_test_doubles;
 
namespace differential_drive_test_application_services
{
  // Define the unit test to verify ability to update the data using the data repository stub
  TEST(BaseModelSetupServiceTests, canSetupAndUpdateBaseModel)
  {
    int wheel_ticks1;
    int wheel_ticks2;
    int wheel_ticks3;

    // Establish Context
    boost::shared_ptr<BaseModelRepositoryStub> base_model_repository_stub =
            boost::shared_ptr<BaseModelRepositoryStub>( new BaseModelRepositoryStub() );
  
    BaseModelSetupService base_model_service( base_model_repository_stub );

    boost::shared_ptr<differential_drive_core::BaseModel> p_base_model;

    p_base_model = base_model_service.GetBaseModel();

    // Act
    wheel_ticks1 = p_base_model->GetWheelTicks();

    base_model_repository_stub->base_geometry.wheel_ticks = 200;
    wheel_ticks2 = p_base_model->GetWheelTicks();
    
    base_model_service.Update();
    wheel_ticks3 = p_base_model->GetWheelTicks();

    // Assert
    EXPECT_EQ( 0, wheel_ticks1);
    EXPECT_EQ( 0, wheel_ticks2);
    EXPECT_EQ( 200, wheel_ticks3);
  }
}
