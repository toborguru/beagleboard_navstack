// BumpersPublisherEndpoint.hpp
 
#ifndef GUARD_BumpersPublisherEndpoint
#define GUARD_BumpersPublisherEndpoint
 
#include <ros/ros.h>

#include "data_robot/Bumpers.h"

#include "IBumpersPublisherEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
  class BumpersPublisherEndpoint : public data_robot_core::IBumpersPublisherEndpoint
  { 
    public:
      BumpersPublisherEndpoint();

      virtual ~BumpersPublisherEndpoint() {};
 
      virtual void Publish( const data_robot::Bumpers& bumpers );
 
    private:
      // Create handle to node
      ros::NodeHandle _bumpers_node;
 
      ros::Publisher _bumpers_publisher;
  };
}
 
#endif /* GUARD_BumpersPublisherEndpoint */
