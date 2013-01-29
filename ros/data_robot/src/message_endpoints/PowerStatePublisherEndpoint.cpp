/** @file
 *  ROS node which publishes @c PowerState messages and broadcasts a coordinate frame 
 *  transform which share a @c frame_id of @e "odom" and a @c child_frame_id of @e 
 *  "base_link".
 */
 
#include <ros/ros.h>

#include "data_robot/PowerState.h"

#include "PowerStatePublisherEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
/** Default constructor, advertises on the ROS @e "power_state" topic.
 */
PowerStatePublisherEndpoint::PowerStatePublisherEndpoint() 
  // Setup topic for publishing 
  : _power_state_publisher(
    _power_state_node.advertise<data_robot::PowerState>("power_state", 10)) 
{ 
}

/** Publishes the @p power_state msg and broadcasts the coordinate frame 
 *  transform.
 */
void PowerStatePublisherEndpoint::Publish( const data_robot::PowerState& power_state )
{
  // and publish the power_state msg
  _power_state_publisher.publish(power_state);

  ros::spinOnce();

  //ROS_DEBUG(  "Published power_state with L: %d R: %d", 
  //            power_state.left_count, power_state.right_count );
}
}
