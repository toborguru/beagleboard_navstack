// TickVelocityPublisherEndpoint.hpp
 
#ifndef GUARD_TickVelocityPublisherEndpoint
#define GUARD_TickVelocityPublisherEndpoint
 
#include <ros/ros.h>

#include "differential_drive/TickVelocity.h"

#include "ITickVelocityPublisherEndpoint.hpp"
 
namespace differential_drive_message_endpoints
{
class TickVelocityPublisherEndpoint : public differential_drive_core::ITickVelocityPublisherEndpoint
{ 
public:
  TickVelocityPublisherEndpoint();

  ~TickVelocityPublisherEndpoint();

  void OnTickVelocityAvailableEvent(const differential_drive::TickVelocity& tick_velocity);  

  void Publish( const differential_drive::TickVelocity& tick_velocity ) const;

private:
  // Create handle to node
  ros::NodeHandle _tick_velocity_node;

  ros::Publisher _tick_velocity_publisher;
};
}
 
#endif /* GUARD_TickVelocityPublisherEndpoint */
