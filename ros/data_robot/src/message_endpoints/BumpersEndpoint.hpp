// BumpersEndpoint.hpp
 
#ifndef GUARD_BumpersEndpoint
#define GUARD_BumpersEndpoint
 
#include <ros/ros.h>

#include "data_robot/Bumpers.h"

#include "IBumpersEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
  class BumpersEndpoint : public data_robot_core::IBumpersEndpoint
  { 
    public:
      BumpersEndpoint();

      virtual ~BumpersEndpoint() {};
 
      virtual void Publish( const data_robot::Bumpers& bumpers );
 
    private:
      // Create handle to node
      ros::NodeHandle _bumpers_node;
 
      ros::Publisher _bumpers_publisher;
  };
}
 
#endif /* GUARD_BumpersEndpoint */
