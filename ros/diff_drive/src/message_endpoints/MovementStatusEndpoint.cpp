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
    _status_node.advertise<diff_drive::MovementStatus>( "status", 5 )) 
{ 
}

MovementStatusEndpoint::~MovementStatusEndpoint() 
{
}

void MovementStatusEndpoint::Publish( const diff_drive::MovementStatus& status ) const 
{
  _status_publisher.publish(status);

  ROS_DEBUG(  "Published status with state: %d linear: %f stasis: %f", 
              status.motors_state,
              status.linear_velocity,
              status.stasis_wheel_velocity );

  ros::spinOnce();
}
}
