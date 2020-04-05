/** @file
 *  ROS node which publishes @c MovementStatus messages.
 */
 
#include <ros/ros.h>
#include "diff_drive_calibrated/MovementStatus.h"
#include "MovementStatusPublisherEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
MovementStatusPublisherEndpoint::MovementStatusPublisherEndpoint() 
  // Setup topic for publishing laser scans to
  : _status_publisher(
    _status_node.advertise<diff_drive_calibrated::MovementStatus>( "movement_status", 5 )) 
{ 
}

MovementStatusPublisherEndpoint::~MovementStatusPublisherEndpoint() 
{
}

/** This class is an movement status listener, and publishes any new messages available.
 */
void MovementStatusPublisherEndpoint::onMovementStatusAvailableEvent(const diff_drive_calibrated::MovementStatus& movement_status)
{
  publish( movement_status );
}

void MovementStatusPublisherEndpoint::publish( const diff_drive_calibrated::MovementStatus& status )
{
  _status_publisher.publish(status);

  ROS_DEBUG(  "Published status with state: %d linear: %f average: %f stasis: %f average: %f stasis enabled: %d ", 
              status.motors_state,
              status.linear_velocity,
              status.linear_velocity_average,
              status.stasis_velocity,
              status.stasis_velocity_average,
              status.stasis_wheel_enabled  );
}
}
