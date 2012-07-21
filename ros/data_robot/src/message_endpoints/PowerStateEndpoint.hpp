// PowerStateEndpoint.hpp
 
#ifndef GUARD_PowerStateEndpoint
#define GUARD_PowerStateEndpoint
 
#include <ros/ros.h>

#include "data_robot/PowerState.h"

#include "IPowerStateEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
  class PowerStateEndpoint : public data_robot_core::IPowerStateEndpoint
  { 
    public:
      PowerStateEndpoint();

      virtual ~PowerStateEndpoint() {};
 
      virtual void Publish( const data_robot::PowerState& power_state );
 
    private:
      // Create handle to node
      ros::NodeHandle _power_state_node;
 
      ros::Publisher _power_state_publisher;
  };
}
 
#endif /* GUARD_PowerStateEndpoint */
