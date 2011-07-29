#include "ros/ros.h"

#include "OdometryReportingService.hpp"
#include "OdometryEndpoint.hpp"
#include "EncoderCountI2CEndpoint.hpp"

using namespace diff_drive_application_services;
using namespace diff_drive_core;
using namespace diff_drive_message_endpoints;

int main(int argc, char **argv)
{
    ros::NodeHandlePtr _nodeHandlePtr;
    IEncoderCountEndpoint* _encoderCountEndpoint;
    boost::shared_ptr<diff_drive_core::IOdometryEndpoint> _odometryEndpoint;
    boost::shared_ptr<diff_drive_application_services::OdometryReportingService> _odometryReportingService;

    // Order of initialization functions is critical:
    // 1) ROS must be initialized before message endpoint(s) can advertise
    ros::init(argc, argv, "diff_drive");
    _nodeHandlePtr.reset(new ros::NodeHandle);

    // 2) Application services must be initialized before being used
    _odometryEndpoint = boost::shared_ptr<IOdometryEndpoint>(
      new OdometryEndpoint());

    _encoderCountEndpoint = new EncoderCountI2CEndpoint();
 
    _odometryReportingService = boost::shared_ptr<OdometryReportingService>(
      new OdometryReportingService(_odometryEndpoint, *_encoderCountEndpoint));

    // 3) Start reporting... SJL this may need a start/stop service
    _odometryReportingService->beginReporting();

    ros::spin();

    return 0;
}
