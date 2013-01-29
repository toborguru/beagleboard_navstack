// PowerStatePublisherEndpoint.hpp
 
#ifndef GUARD_PowerStatePublisherEndpoint
#define GUARD_PowerStatePublisherEndpoint
 
#include <ros/ros.h>

#include "data_robot/PowerState.h"

#include "IPowerStatePublisherEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
  class PowerStatePublisherEndpoint : public data_robot_core::IPowerStatePublisherEndpoint
  { 
    public:
      PowerStatePublisherEndpoint();

      virtual ~PowerStatePublisherEndpoint() {};
 
      virtual void Publish( const data_robot::PowerState& power_state );
 
    private:
      // Create handle to node
      ros::NodeHandle _power_state_node;
 
      ros::Publisher _power_state_publisher;
  };
}
 
#endif /* GUARD_PowerStatePublisherEndpoint */
