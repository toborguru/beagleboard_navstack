// TickVelocityEndpoint.cpp
 
#include <ros/ros.h>
#include "diff_drive/TickVelocity.h"
#include "TickVelocityEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
TickVelocityEndpoint::TickVelocityEndpoint() 
  // Setup topic for publishing laser scans to
  : _tick_velocity_publisher(
    _tick_velocity_node.advertise<diff_drive::TickVelocity>( "tick_velocity", 5 )) 
{ 
}

TickVelocityEndpoint::~TickVelocityEndpoint() 
{
}

void TickVelocityEndpoint::Publish( const diff_drive::TickVelocity& tick_velocity ) const 
{
  _tick_velocity_publisher.publish(tick_velocity);

  ros::spinOnce();
  ROS_INFO( "Published to tick_velocity topic with linear, angular of: %d, %d", 
            tick_velocity.linear_ticks_sec,
            tick_velocity.angular_ticks_sec );
}
}
