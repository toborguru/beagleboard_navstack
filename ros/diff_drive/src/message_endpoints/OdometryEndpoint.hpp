// OdometryEndpoint.hpp
 
#ifndef GUARD_OdometryEndpoint
#define GUARD_OdometryEndpoint
 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "nav_msgs/Odometry.h"

#include "IOdometryEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
  class OdometryEndpoint : public diff_drive_core::IOdometryEndpoint
  { 
    public:
      OdometryEndpoint();

      virtual ~OdometryEndpoint() {};
 
      virtual void Publish(const nav_msgs::Odometry& odometry_scan);
 
    private:
      // Create handle to node
      ros::NodeHandle _odometry_node;
 
      ros::Publisher _odometry_publisher;

      tf::TransformBroadcaster _transform_broadcaster;
  };
}
 
#endif /* GUARD_OdometryEndpoint */
