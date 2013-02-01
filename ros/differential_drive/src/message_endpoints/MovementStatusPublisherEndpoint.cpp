/** @file
 *  ROS node which publishes @c MovementStatus messages.
 */
 
#include <ros/ros.h>
#include "differential_drive/MovementStatus.h"
#include "MovementStatusPublisherEndpoint.hpp"
 
namespace differential_drive_message_endpoints
{
MovementStatusPublisherEndpoint::MovementStatusPublisherEndpoint() 
  // Setup topic for publishing laser scans to
  : _status_publisher(
    _status_node.advertise<differential_drive::MovementStatus>( "movement_status", 5 )) 
{ 
}

MovementStatusPublisherEndpoint::~MovementStatusPublisherEndpoint() 
{
}

void MovementStatusPublisherEndpoint::Publish( const differential_drive::MovementStatus& status )
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
