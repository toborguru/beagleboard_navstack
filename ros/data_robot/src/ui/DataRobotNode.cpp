#include "ros/ros.h"

#include "BumpersProcessor.hpp"

#include "BumpersProcessorSetupService.hpp"
#include "BaseTelemetryReportingService.hpp"
#include "FrontTelemetryReportingService.hpp"
#include "TickVelocityCommandService.hpp"

#include "BumperIndexesRepository.hpp"

#include "BumpersEndpoint.hpp"
#include "EncoderCountsEndpoint.hpp"
#include "I2CBusEndpoint.hpp"
#include "TickVelocityEndpoint.hpp"

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
    // Bumpers Processor
    boost::shared_ptr<BumpersProcessor> bumpers_processor =
        boost::shared_ptr<BumpersProcessor>( new BumpersProcessor() );


    // I2C Bus
    boost::shared_ptr<I2CBusEndpoint> i2c_bus_endpoint =
        boost::shared_ptr<I2CBusEndpoint>( new I2CBusEndpoint() );


    // SERVICES
    // Bumpers Processor Setup Service
    boost::shared_ptr<BumperIndexesRepository> bumper_indexes_repository =
        boost::shared_ptr<BumperIndexesRepository>( new BumperIndexesRepository() );

    BumpersProcessorSetupService bumpers_processor_setup_service( bumper_indexes_repository,
                                                                  bumpers_processor );


    // Encoder Counts Reporting Service
    boost::shared_ptr<EncoderCountsEndpoint> encoder_counts_endpoint =
        boost::shared_ptr<EncoderCountsEndpoint>( new EncoderCountsEndpoint() );

    BaseTelemetryReportingService base_telemetry_reporting_service( encoder_counts_endpoint,
                                                                    i2c_bus_endpoint );


    // Front Telemetry Reporting Service
    boost::shared_ptr<BumpersEndpoint> bumpers_endpoint =
        boost::shared_ptr<BumpersEndpoint>( new BumpersEndpoint() );

    FrontTelemetryReportingService front_telemetry_reporting_service( bumpers_endpoint,
                                                                      i2c_bus_endpoint,
                                                                      bumpers_processor );

    
    // Twist Command Service
    boost::shared_ptr<TickVelocityEndpoint> tick_velocity_endpoint =
        boost::shared_ptr<TickVelocityEndpoint>( new TickVelocityEndpoint() );

    TickVelocityCommandService tick_velocity_command_service( tick_velocity_endpoint, 
                                                              i2c_bus_endpoint );


    // 3) Start the services
    i2c_bus_endpoint->Open( "/dev/i2c-2" );

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

