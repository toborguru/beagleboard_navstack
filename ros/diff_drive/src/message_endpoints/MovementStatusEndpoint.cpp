/** @file
 *  ROS node which publishes @c MovementStatus messages.
 */
 
#include <ros/ros.h>
#include "diff_drive/MovementStatus.h"
#include "MovementStatusEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
MovementStatusEndpoint::MovementStatusEndpoint() 
  // Setup topic for publishing laser scans to
  : _status_publisher(
    _status_node.advertise<diff_drive::MovementStatus>( "movement_status", 5 )) 
{ 
}

MovementStatusEndpoint::~MovementStatusEndpoint() 
{
}

void MovementStatusEndpoint::Publish( const diff_drive::MovementStatus& status )
{
  _status_publisher.publish(status);

  ROS_DEBUG(  "Published status with state: %d linear: %f average: %f stasis: %f average: %f stasis enabled: %d ", 
              status.motors_state,
              status.linear_velocity,
              status.linear_velocity_average,
              status.stasis_velocity,
              status.stasis_velocity_average,
              status.stasis_wheel_enabled  );

  ros::spinOnce();
}
}
