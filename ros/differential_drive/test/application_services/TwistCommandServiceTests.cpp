// TwistCommandServiceTests.cpp
 
#include <gtest/gtest.h>

#include "BaseModel.hpp"
#include "TickVelocityPublisherEndpointStub.hpp"
#include "TwistSubscriberEndpointStub.hpp"
#include "TwistCommandService.hpp"
 
using namespace differential_drive_application_services;
using namespace differential_drive_test_message_endpoints_test_doubles;
 
namespace differential_drive_test_application_services
{
  // Define the unit test to verify ability to leverage the reporting service using the message endpoint stub
  TEST(TwistCommandServiceTests, canCanStartAndstopAcceptingCommands)
  {
    geometry_msgs::Twist twist;

    int received1;
    int received2;
    int received3;

    // Establish Context
    boost::shared_ptr<TickVelocityPublisherEndpointStub> tick_velocity_endpoint_stub =
        boost::shared_ptr<TickVelocityPublisherEndpointStub>( new TickVelocityPublisherEndpointStub() );

    boost::shared_ptr<TwistSubscriberEndpointStub> twist_endpoint_stub =
        boost::shared_ptr<TwistSubscriberEndpointStub>( new TwistSubscriberEndpointStub() );

    boost::shared_ptr<differential_drive_core::BaseModel> base_model = 
        boost::shared_ptr<differential_drive_core::BaseModel>( new differential_drive_core::BaseModel() );
    
    TwistCommandService twist_command_service(  tick_velocity_endpoint_stub, 
                                                          twist_endpoint_stub,
                                                          base_model );

    // Act
    twist.linear.x  = 1.0;
    twist.angular.z = 0.0;

    twist_endpoint_stub->AddTicks( twist );
    twist_endpoint_stub->AddTicks( twist );
    twist_endpoint_stub->AddTicks( twist );
    twist_endpoint_stub->AddTicks( twist );
 
    received1 = tick_velocity_endpoint_stub->_count_of_tick_velocities_published;
   
    twist_command_service.startAcceptingCommands();

    twist_endpoint_stub->AddTicks( twist );
    twist_endpoint_stub->AddTicks( twist );
    twist_endpoint_stub->AddTicks( twist );
    twist_endpoint_stub->AddTicks( twist );
 
    received2 = tick_velocity_endpoint_stub->_count_of_tick_velocities_published;
    
    twist_command_service.stopAcceptingCommands();

    twist_endpoint_stub->AddTicks( twist );
    twist_endpoint_stub->AddTicks( twist );
    twist_endpoint_stub->AddTicks( twist );
    twist_endpoint_stub->AddTicks( twist );
 
    received3 = tick_velocity_endpoint_stub->_count_of_tick_velocities_published;
    
    // Assert
 
    // Arguably, this test is a bit light but makes sure tick_velocity msgs are being pushed 
    // to the message endpoint for publication.
    EXPECT_EQ( 0, received1 );
    EXPECT_EQ( 4, received2 );
    EXPECT_EQ( 4, received3 );
  }

  // Define the unit test to verify ability to leverage the reporting service using the message endpoint stub
  TEST(TwistCommandServiceTests, canCanSendCommandsToTwistCommandService) 
  {
    geometry_msgs::Twist twist;
    int linear1;
    int linear2;
    int angular1;
    int angular2;

    // Establish Context
    boost::shared_ptr<TickVelocityPublisherEndpointStub> tick_velocity_endpoint_stub =
        boost::shared_ptr<TickVelocityPublisherEndpointStub>( new TickVelocityPublisherEndpointStub() );

    boost::shared_ptr<TwistSubscriberEndpointStub> twist_endpoint_stub =
        boost::shared_ptr<TwistSubscriberEndpointStub>( new TwistSubscriberEndpointStub() );

    boost::shared_ptr<differential_drive_core::BaseModel> base_model = 
        boost::shared_ptr<differential_drive_core::BaseModel>( new differential_drive_core::BaseModel( 0.5, 100, 1.0 ) );
    
    TwistCommandService twist_command_service(  tick_velocity_endpoint_stub, 
                                                          twist_endpoint_stub,
                                                          base_model );

    // Act
    twist.linear.x = 1.0;
    twist.angular.z = 0.0;

    twist_command_service.startAcceptingCommands();

    twist_endpoint_stub->AddTicks( twist );

    linear1 = tick_velocity_endpoint_stub->_linear;
    angular1 = tick_velocity_endpoint_stub->_angular;

    twist.linear.x = 0.0;
    twist.angular.z = 1.0;
    twist_endpoint_stub->AddTicks( twist );

    linear2 = tick_velocity_endpoint_stub->_linear;
    angular2 = tick_velocity_endpoint_stub->_angular;
 
    // Assert
 
    // Arguably, this test is a bit light but makes sure tick_velocity msgs are being pushed 
    // to the message endpoint for publication.
    EXPECT_EQ( 32, linear1 );
    EXPECT_EQ( 0, angular1 );

    EXPECT_EQ( 0, linear2 );
    EXPECT_EQ( 32, angular2 );
  }
}
