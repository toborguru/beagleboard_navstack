// OdometryEndpoint.cpp
 
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "OdometryEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
  OdometryEndpoint::OdometryEndpoint() 
    // Setup topic for publishing laser scans to
    : _odometryPublisher(
      _odometryNode.advertise<nav_msgs::Odometry>("odometry", 5)) 
  { 
  }
 
  void OdometryEndpoint::publish(const nav_msgs::Odometry& odometry) const 
  {
    _odometryPublisher.publish(odometry);
    ros::spinOnce();
    ROS_INFO("Published odometry to odometry topic with x, y of: %f, %f", odometry.pose.pose.position.x,
                                                                          odometry.pose.pose.position.y );
  };
}
