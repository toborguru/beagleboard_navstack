// OdometryEndpoint.hpp
 
#ifndef GUARD_OdometryEndpoint
#define GUARD_OdometryEndpoint
 
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "IOdometryEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
  class OdometryEndpoint : public diff_drive_core::IOdometryEndpoint
  { 
    public:
      virtual ~OdometryEndpoint() {};
 
      virtual void publish(const nav_msgs::Odometry& odometryScan) const;
 
    private:
      // Create handle to node
      ros::NodeHandle _odometryNode;
 
      ros::Publisher _odometryPublisher;
  };
}
 
#endif /* GUARD_OdometryEndpoint */
