#include "ros/ros.h"

#include "OdometryReportingService.hpp"
#include "TwistCommandService.hpp"
#include "ParametersSetupService.hpp"

#include "OdometryPublisherEndpoint.hpp"
#include "MovementStatusPublisherEndpoint.hpp"
#include "TickVelocityPublisherEndpoint.hpp"
#include "TwistSubscriberEndpoint.hpp"
#include "EncoderCountsSubscriberEndpoint.hpp"

#include "DifferentialParametersRepository.hpp"

using namespace differential_drive_application_services;
using namespace differential_drive_core;
using namespace differential_drive_message_endpoints;
using namespace differential_drive_data_repositories;

int main(int argc, char **argv)
{
    // Order of initialization functions is critical:
    // 1) ROS must be initialized before message endpoint(s) can advertise
    ros::init(argc, argv, "differential_drive");

    ROS_INFO( "differential_drive node started." );

    // 2) Application services must be initialized before being used
    // Base Model 
    boost::shared_ptr<BaseModel> base_model = 
        boost::shared_ptr<BaseModel>( new BaseModel() );

    // Odometry Reporting Service
    boost::shared_ptr<OdometryPublisherEndpoint> odometry_endpoint =
        boost::shared_ptr<OdometryPublisherEndpoint>( new OdometryPublisherEndpoint() );

    boost::shared_ptr<MovementStatusPublisherEndpoint> movement_status_endpoint =
        boost::shared_ptr<MovementStatusPublisherEndpoint>( new MovementStatusPublisherEndpoint() );

    boost::shared_ptr<EncoderCountsSubscriberEndpoint> encoder_counts_endpoint =
        boost::shared_ptr<EncoderCountsSubscriberEndpoint>( new EncoderCountsSubscriberEndpoint() );

    OdometryReportingService odometry_reporting_service(  odometry_endpoint, 
                                                          movement_status_endpoint, 
                                                          encoder_counts_endpoint,
                                                          base_model );

    
    // Twist Command Service
    boost::shared_ptr<TickVelocityPublisherEndpoint> tick_velocity_endpoint =
        boost::shared_ptr<TickVelocityPublisherEndpoint>( new TickVelocityPublisherEndpoint() );

    boost::shared_ptr<TwistSubscriberEndpoint> twist_endpoint =
        boost::shared_ptr<TwistSubscriberEndpoint>( new TwistSubscriberEndpoint() );

    TwistCommandService twist_command_service(  tick_velocity_endpoint, 
                                                twist_endpoint,
                                                base_model );


    // Base Model Setup Service
    boost::shared_ptr<DifferentialParametersRepository> base_model_repository =
        boost::shared_ptr<DifferentialParametersRepository>( new DifferentialParametersRepository() );

    ParametersSetupService base_model_setup_service(  base_model_repository,
                                                                  base_model );
                                                                  
  

    // 3) Start the services
    ROS_INFO( "Starting the Base Model Setup Service..." );
    base_model_setup_service.Update();
    base_model_setup_service.StartUpdating();

    ROS_INFO( "Starting Odometry Reporting Service..." );
    odometry_reporting_service.StartReporting();

    ROS_INFO( "Starting Twist Command Service..." );
    twist_command_service.StartAcceptingCommands();

    ros::spin();

    // Never gets here...
    ROS_INFO( "differential_drive_node stopped." );

    return 0;
}

