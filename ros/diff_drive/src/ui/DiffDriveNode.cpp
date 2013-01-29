#include "ros/ros.h"

#include "OdometryReportingService.hpp"
#include "TwistCommandService.hpp"

#include "OdometryPublisherEndpoint.hpp"
#include "MovementStatusPublisherEndpoint.hpp"
#include "TickVelocityPublisherEndpoint.hpp"
#include "TwistSubscriberEndpoint.hpp"
#include "EncoderCountsSubscriberEndpoint.hpp"

#include "BaseModelRepository.hpp"

using namespace diff_drive_application_services;
using namespace diff_drive_core;
using namespace diff_drive_message_endpoints;
using namespace diff_drive_data_repositories;

int main(int argc, char **argv)
{
    // Order of initialization functions is critical:
    // 1) ROS must be initialized before message endpoint(s) can advertise
    ros::init(argc, argv, "diff_drive");

    ROS_INFO( "diff_drive node started." );

    // 2) Application services must be initialized before being used
    // Base Model Setup
    BaseModelRepository base_model_repository;

    boost::shared_ptr<BaseModel> base_model = 
        boost::shared_ptr<BaseModel>( new BaseModel(base_model_repository.QueryBaseGeometry( )) );

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


    // 3) Start the services
    ROS_INFO( "Starting Odometry Reporting Service..." );
    odometry_reporting_service.BeginReporting();

    ROS_INFO( "Starting Twist Command Service..." );
    twist_command_service.BeginAcceptingCommands();

    ros::spin();

    // Never gets here...
    ROS_INFO( "diff_drive_node stopped." );

    return 0;
}

