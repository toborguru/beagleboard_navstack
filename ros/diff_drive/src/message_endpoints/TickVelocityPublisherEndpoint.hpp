// TickVelocityPublisherEndpoint.hpp
 
#ifndef GUARD_TickVelocityPublisherEndpoint
#define GUARD_TickVelocityPublisherEndpoint
 
#include <ros/ros.h>

#include "diff_drive/TickVelocity.h"

#include "ITickVelocityPublisherEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
  class TickVelocityPublisherEndpoint : public diff_drive_core::ITickVelocityPublisherEndpoint
  { 
    public:
      TickVelocityPublisherEndpoint();

      virtual ~TickVelocityPublisherEndpoint();
 
      virtual void Publish( const diff_drive::TickVelocity& tick_velocity ) const;
 
    private:
      // Create handle to node
      ros::NodeHandle _tick_velocity_node;
 
      ros::Publisher _tick_velocity_publisher;
  };
}
 
#endif /* GUARD_TickVelocityPublisherEndpoint */
