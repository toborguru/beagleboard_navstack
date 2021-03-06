// OdometryPublisherEndpoint.hpp
 
#ifndef GUARD_OdometryPublisherEndpoint
#define GUARD_OdometryPublisherEndpoint
 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "nav_msgs/Odometry.h"

#include "IOdometryPublisherEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
class OdometryPublisherEndpoint : public diff_drive_core::IOdometryPublisherEndpoint
{ 
public:
  OdometryPublisherEndpoint();

  ~OdometryPublisherEndpoint() {};

  void publish(const nav_msgs::Odometry& odometry_scan);

  void onOdometryAvailableEvent(const nav_msgs::Odometry& odometry);

private:
  // Create handle to node
  ros::NodeHandle _odometry_node;

  ros::Publisher _odometry_publisher;

  tf::TransformBroadcaster _transform_broadcaster;
};
}
 
#endif /* GUARD_OdometryPublisherEndpoint */
