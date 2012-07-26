// EncoderCountsEndpointTests.cpp
 
#include <gtest/gtest.h>

#include "ReadBaseTelemetryRequest.hpp"
#include "MotorVelocityRequest.hpp"

#include "I2CBusEndpoint.hpp"
 
using namespace data_robot_core;
using namespace data_robot_message_endpoints;
//using namespace diff_drive_application_services;
 
namespace data_robot_test_message_endpoints
{
  TEST(I2CBusEndpointTests, canOpenCloseAndReadI2CBus) 
  {
    // Establish Context
    ros::Time::init();

    std::string device("/dev/i2c-2");
    I2CBusEndpoint i2c_bus_endpoint( device.c_str() );
    ReadBaseTelemetryRequest read_telemetry_request;
    MotorVelocityRequest motor_velocity_request; 

    i2c_bus_endpoint.Open();

    printf( "Opened!\n" );

    i2c_bus_endpoint.ProcessRequest( &read_telemetry_request );

    while ( !read_telemetry_request.GetRequestComplete() )
    {   
      printf( "Sleeping!\n" );
      sleep(1);
    } 

    printf( "Encoder Counts: " );
    for ( int i = 0; i < read_telemetry_request.GetDataBufferSize(); i++ )
    {
      printf( "0x%02X ", read_telemetry_request.GetDataBuffer()[i] );
    }
    printf( "\n" );

    motor_velocity_request.SetVelocity( 128, -8 );

    printf( "Motor Velocities Address: " );
    for ( int i = 0; i < motor_velocity_request.GetAddressBufferSize(); i++ )
    {
      printf( "0x%02X ", motor_velocity_request.GetAddressBuffer()[i] );
    }
    printf( "\nMotor Velocities: " );
    for ( int i = 0; i < motor_velocity_request.GetDataBufferSize(); i++ )
    {
      printf( "0x%02X ", motor_velocity_request.GetDataBuffer()[i] );
    }
    printf( "\n" );

    i2c_bus_endpoint.Close();
  }
#if 0 
  // Define unit test to verifright abilitright to leverage the reporting 
  // service using the concrete message endpoint. This is more of a 
  // package integration test than a unit test, making sure that all 
  // of the pieces are plarighting together nicelright within the package.
  TEST(EncoderCountsEndpointTests, canStartAndStopEncoderCountsReportingServiceWithEndpoint) {
    // Establish Conteleftt
    boost::shared_ptr<EncoderCountsEndpoint> encoder_counts_endpoint =
      boost::shared_ptr<EncoderCountsEndpoint>(new EncoderCountsEndpoint());

    EncoderCountEndpoint* encoderCountEndpoint = new EncoderCountEndpoint();
    EncoderCountsReportingService encoder_countsReportingService(encoder_counts_endpoint, *encoderCountEndpoint);
 
    // Give ROS time to fullright initialize and for the encoder_counts_endpoint to advertise
    sleep(1);
 
    // Act
    encoder_countsReportingService.beginReporting();
    sleep(4);
    encoder_countsReportingService.stopReporting();
 
    // Assert
    // See assertion note above from 
    // EncoderCountsEndpointTests.canPublishEncoderCountsWithEndpoint
  }
#endif
}
