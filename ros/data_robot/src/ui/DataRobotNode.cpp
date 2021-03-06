#include "ros/ros.h"

#include "BumpersProcessor.hpp"

#include "BumpersProcessorSetupService.hpp"
#include "BaseTelemetryReportingService.hpp"
#include "FrontTelemetryReportingService.hpp"
#include "TickVelocityCommandService.hpp"

#include "BumperIndexesRepository.hpp"

#include "BumpersPublisherEndpoint.hpp"
#include "LogPublisherEndpoint.hpp"
#include "EncoderCountsPublisherEndpoint.hpp"
#include "I2CBusRequestProcessorEndpoint.hpp"
#include "PowerStatePublisherEndpoint.hpp"
#include "TickVelocitySubscriberEndpoint.hpp"

using namespace data_robot_application_services;
using namespace data_robot_core;
using namespace data_robot_data_repositories;
using namespace data_robot_message_endpoints;

int main(int argc, char **argv)
{
    // Order of initialization functions is critical:
    // 1) ROS must be initialized before message endpoint(s) can advertise
    ros::init(argc, argv, "data_robot");

    ROS_INFO( "data_robot node started." );

    // 2) Application services must be initialized before being used

    // SHARED OBJECTS
    // Log Publisher
    boost::shared_ptr<LogPublisherEndpoint> log_endpoint = 
        boost::shared_ptr<LogPublisherEndpoint>( new LogPublisherEndpoint() );

    // Bumpers Processor
    boost::shared_ptr<BumpersProcessor> bumpers_processor =
        boost::shared_ptr<BumpersProcessor>( new BumpersProcessor() );

    // I2C Bus
    boost::shared_ptr<I2CBusRequestProcessorEndpoint> i2c_bus_endpoint =
        boost::shared_ptr<I2CBusRequestProcessorEndpoint>( new I2CBusRequestProcessorEndpoint() );

    // SERVICES
    // Bumpers Processor Setup Service
    boost::shared_ptr<BumperIndexesRepository> bumper_indexes_repository =
        boost::shared_ptr<BumperIndexesRepository>( new BumperIndexesRepository() );

    BumpersProcessorSetupService bumpers_processor_setup_service( bumper_indexes_repository,
                                                                  bumpers_processor );


    // Base Telemetry Reporting Service
    boost::shared_ptr<EncoderCountsPublisherEndpoint> encoder_counts_endpoint =
        boost::shared_ptr<EncoderCountsPublisherEndpoint>( new EncoderCountsPublisherEndpoint() );

    boost::shared_ptr<EncoderCountsProcessor> encoder_counts_processor =
        boost::shared_ptr<EncoderCountsProcessor>( new EncoderCountsProcessor() );

    boost::shared_ptr<PowerStatePublisherEndpoint> power_state_endpoint =
        boost::shared_ptr<PowerStatePublisherEndpoint>( new PowerStatePublisherEndpoint() );

    boost::shared_ptr<PowerStateProcessor> power_state_processor =
        boost::shared_ptr<PowerStateProcessor>( new PowerStateProcessor() );

    BaseTelemetryReportingService base_telemetry_reporting_service( encoder_counts_endpoint,
                                                                    power_state_endpoint,
                                                                    i2c_bus_endpoint,
                                                                    encoder_counts_processor,
                                                                    power_state_processor );


    // Front Telemetry Reporting Service
    boost::shared_ptr<BumpersPublisherEndpoint> bumpers_endpoint =
        boost::shared_ptr<BumpersPublisherEndpoint>( new BumpersPublisherEndpoint() );

    FrontTelemetryReportingService front_telemetry_reporting_service( bumpers_endpoint,
                                                                      i2c_bus_endpoint,
                                                                      bumpers_processor );

    
    // Ticks Velocity Command Service
    boost::shared_ptr<TickVelocitySubscriberEndpoint> tick_velocity_endpoint =
        boost::shared_ptr<TickVelocitySubscriberEndpoint>( new TickVelocitySubscriberEndpoint( log_endpoint ) );

    TickVelocityCommandService tick_velocity_command_service( tick_velocity_endpoint, 
                                                              i2c_bus_endpoint,
                                                              log_endpoint );


    int error_code;
    // 3) Start the services
    error_code = i2c_bus_endpoint->Open( "/dev/i2c-1" );
    if ( error_code )
    {
      printf( "Error opening the I2C bus: %d\n", error_code );
      exit( 1 );
    }

    bumpers_processor_setup_service.Update();

    ROS_INFO( "Starting Base Telemetry Reporting Service..." );
    base_telemetry_reporting_service.BeginReporting();

    ROS_INFO( "Starting Front Shell Telemetry Reporting Service..." );
    front_telemetry_reporting_service.BeginReporting();

    ROS_INFO( "Starting Tick Velocity Command Service..." );
    tick_velocity_command_service.BeginAcceptingCommands();

    ros::spin();

    // Never gets here...
    ROS_INFO( "data_robot node stopped." );

    return 0;
}

