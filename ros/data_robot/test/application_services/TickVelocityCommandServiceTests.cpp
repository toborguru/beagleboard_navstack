// TickVelocityCommandServiceTests.cpp
 
#include <gtest/gtest.h>

#include "TickVelocitySubscriberEndpointStub.hpp"
#include "BusRequestProcessorEndpointStub.hpp"
#include "TickVelocityCommandService.hpp"
 
using namespace data_robot_application_services;
using namespace data_robot_test_message_endpoints_test_doubles;
 
namespace data_robot_test_application_services
{
  // Define the unit test to verify ability to leverage the reporting service using the message endpoint stub
  TEST(TickVelocityCommandServiceTests, canCanStartAndStopAcceptingCommands)
  {
    differential_drive::TickVelocity tick_velocity;

    int received1;
    int received2;
    int received3;

    // Establish Context
    boost::shared_ptr<TickVelocitySubscriberEndpointStub> tick_velocity_endpoint_stub =
        boost::shared_ptr<TickVelocitySubscriberEndpointStub>( new TickVelocitySubscriberEndpointStub() );

    boost::shared_ptr<BusRequestProcessorEndpointStub> external_bus_endpoint_stub =
        boost::shared_ptr<BusRequestProcessorEndpointStub>( new BusRequestProcessorEndpointStub() );

    TickVelocityCommandService tick_velocity_command_service( tick_velocity_endpoint_stub, 
                                                              external_bus_endpoint_stub );
                                                      
    // Act
    tick_velocity.linear_ticks_sec  = 10;
    tick_velocity.angular_ticks_sec = 0;

    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
 
    received1 = external_bus_endpoint_stub->_count_of_bus_requests_processed;
   
    tick_velocity_command_service.BeginAcceptingCommands();

    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
 
    received2 = external_bus_endpoint_stub->_count_of_bus_requests_processed;
    
    tick_velocity_command_service.StopAcceptingCommands();

    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
    tick_velocity_endpoint_stub->AddTicks( tick_velocity );
 
    received3 = external_bus_endpoint_stub->_count_of_bus_requests_processed;
    
    // Assert
 
    // Arguably, this test is a bit light but makes sure tick_velocity msgs are being pushed 
    // to the message endpoint for publication.
    EXPECT_EQ( 0, received1 );
    EXPECT_EQ( 4, received2 );
    EXPECT_EQ( 4, received3 );
  }
#if 0
  // Define the unit test to verify ability to leverage the reporting service using the message endpoint stub
  TEST(TickVelocityCommandServiceTests, canCanSendCommandsToTickVelocityCommandService) 
  {
    // Establish Context
    boost::shared_ptr<TickVelocitySubscriberEndpointStub> tick_velocity_endpoint_stub =
        boost::shared_ptr<TickVelocitySubscriberEndpointStub>( new TickVelocitySubscriberEndpointStub() );

    boost::shared_ptr<BusRequestProcessorEndpointStub> external_bus_endpoint_stub =
        boost::shared_ptr<BusRequestProcessorEndpointStub>( new BusRequestProcessorEndpointStub() );

    TickVelocityCommandService tick_velocity_command_service( tick_velocity_endpoint_stub, 
                                                              external_bus_endpoint_stub );
                                                      
    // Act
    differential_drive::TickVelocity tick_velocity;

    int linear1;
    int linear2;
    int angular1;
    int angular2;

    // Establish Context
    // Act
    tick_velocity.linear_ticks_sec = 10;
    tick_velocity.angular_ticks_sec = 0;

    tick_velocity_command_service.BeginAcceptingCommands();

    tick_velocity_endpoint_stub->AddTicks( tick_velocity );

    linear1 = tick_velocity_endpoint_stub->_linear;
    angular1 = tick_velocity_endpoint_stub->_angular;

    tick_velocity.linear_ticks_sec  = 10;
    tick_velocity.angular_ticks_sec = 0;

    tick_velocity.linear.x = 0.0;
    tick_velocity.angular.z = 1.0;
    tick_velocity_endpoint_stub->AddTicks( tick_velocity );

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
#endif 
}
