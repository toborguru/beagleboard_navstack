/** @file
 *  ROS node which publishes @c TickVelocity messages.
 */
 
#include <ros/ros.h>
#include "diff_drive_calibrated/TickVelocity.h"
#include "TickVelocityPublisherEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
TickVelocityPublisherEndpoint::TickVelocityPublisherEndpoint() 
  // Setup topic for publishing laser scans to
  : _tick_velocity_publisher(
    _tick_velocity_node.advertise<diff_drive_calibrated::TickVelocity>( "tick_velocity", 5 )) 
{ 
}

TickVelocityPublisherEndpoint::~TickVelocityPublisherEndpoint() 
{
}

/** This class is a tick velocity listener, act on new tick velocity available.
 */
void TickVelocityPublisherEndpoint::onTickVelocityAvailableEvent(const diff_drive_calibrated::TickVelocity& tick_velocity)
{
  publish( tick_velocity );
}

void TickVelocityPublisherEndpoint::publish( const diff_drive_calibrated::TickVelocity& tick_velocity ) const 
{
  _tick_velocity_publisher.publish(tick_velocity);

  ROS_DEBUG(  "Published tick_velocity with linear, angular of: %d, %d", 
              tick_velocity.linear_ticks_sec,
              tick_velocity.angular_ticks_sec );
}
}
