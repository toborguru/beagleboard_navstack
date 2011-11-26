// TickVelocityEndpoint.hpp
 
#ifndef GUARD_TickVelocityEndpoint
#define GUARD_TickVelocityEndpoint
 
#include <ros/ros.h>

#include "diff_drive/TickVelocity.h"

#include "ITickVelocityEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
  class TickVelocityEndpoint : public diff_drive_core::ITickVelocityEndpoint
  { 
    public:
      TickVelocityEndpoint();

      virtual ~TickVelocityEndpoint();
 
      virtual void Publish( const diff_drive::TickVelocity& tick_velocity ) const;
 
    private:
      // Create handle to node
      ros::NodeHandle _tick_velocity_node;
 
      ros::Publisher _tick_velocity_publisher;
  };
}
 
#endif /* GUARD_TickVelocityEndpoint */
