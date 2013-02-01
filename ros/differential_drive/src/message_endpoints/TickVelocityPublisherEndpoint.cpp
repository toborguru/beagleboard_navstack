/** @file
 *  ROS node which publishes @c TickVelocity messages.
 */
 
#include <ros/ros.h>
#include "differential_drive/TickVelocity.h"
#include "TickVelocityPublisherEndpoint.hpp"
 
namespace differential_drive_message_endpoints
{
TickVelocityPublisherEndpoint::TickVelocityPublisherEndpoint() 
  // Setup topic for publishing laser scans to
  : _tick_velocity_publisher(
    _tick_velocity_node.advertise<differential_drive::TickVelocity>( "tick_velocity", 5 )) 
{ 
}

TickVelocityPublisherEndpoint::~TickVelocityPublisherEndpoint() 
{
}

void TickVelocityPublisherEndpoint::Publish( const differential_drive::TickVelocity& tick_velocity ) const 
{
  _tick_velocity_publisher.publish(tick_velocity);

  ROS_DEBUG(  "Published tick_velocity with linear, angular of: %d, %d", 
              tick_velocity.linear_ticks_sec,
              tick_velocity.angular_ticks_sec );
}
}
