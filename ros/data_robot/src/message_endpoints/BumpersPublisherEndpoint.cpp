/** @file
 *  ROS node which publishes @c Bumpers messages and broadcasts a coordinate frame 
 *  transform which share a @c frame_id of @e "odom" and a @c child_frame_id of @e 
 *  "base_link".
 */
 
#include <ros/ros.h>

#include "BumpersPublisherEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
/** Default constructor, advertises on the ROS @e "bumpers" topic.
 */
BumpersPublisherEndpoint::BumpersPublisherEndpoint() 
  // Setup topic for publishing 
  : _bumpers_publisher(
    _bumpers_node.advertise<data_robot::Bumpers>("bumpers", 10)) 
{ 
}

/** Publishes the @p bumpers msg and broadcasts the coordinate frame 
 *  transform.
 */
void BumpersPublisherEndpoint::Publish( const data_robot::Bumpers& bumpers )
{
  // and publish the bumpers msg
  _bumpers_publisher.publish(bumpers);

  ros::spinOnce();

  //ROS_DEBUG(  "Published bumpers with L: %d R: %d", 
  //            bumpers.left_count, bumpers.right_count );
}
}
