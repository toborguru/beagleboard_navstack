// OdometryEndpoint.cpp
 
#include <ros/ros.h>

#include "nav_msgs/Odometry.h"

#include "OdometryEndpoint.hpp"
 
namespace diff_drive_message_endpoints
{
OdometryEndpoint::OdometryEndpoint() 
  // Setup topic for publishing laser scans to
  : _odometry_publisher(
    _odometry_node.advertise<nav_msgs::Odometry>("odometry", 5)) 
{ 
}

void OdometryEndpoint::Publish( const nav_msgs::Odometry& odometry )
{
  nav_msgs::Odometry stamped_odometry;
  geometry_msgs::TransformStamped odometry_transform;

  stamped_odometry = odometry;

  // first, we'll populate the header for the odometry msg
  stamped_odometry.header.frame_id = "odom";
  stamped_odometry.child_frame_id = "base_link";
 
  // and publish the odometry msg
  _odometry_publisher.publish(stamped_odometry);

  // then we'll populate the transform
  odometry_transform.header.stamp = stamped_odometry.header.stamp;
  odometry_transform.header.frame_id = stamped_odometry.header.frame_id;
  odometry_transform.child_frame_id = stamped_odometry.child_frame_id;

  odometry_transform.transform.translation.x = stamped_odometry.pose.pose.position.x;
  odometry_transform.transform.translation.y = stamped_odometry.pose.pose.position.y;
  odometry_transform.transform.translation.z = stamped_odometry.pose.pose.position.z;
  odometry_transform.transform.rotation = stamped_odometry.pose.pose.orientation;

  // send the transform
  _transform_broadcaster.sendTransform(odometry_transform);

  ros::spinOnce();
  ROS_INFO("Published odometry to odometry topic with x, y of: %f, %f", odometry.pose.pose.position.x,
                                                                        odometry.pose.pose.position.y );
}
}
