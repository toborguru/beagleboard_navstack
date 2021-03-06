/** @file
 *  ROS node which publishes @c EncoderCounts messages and broadcasts a coordinate frame 
 *  transform which share a @c frame_id of @e "odom" and a @c child_frame_id of @e 
 *  "base_link".
 */
 
#include <ros/ros.h>

#include "diff_drive_calibrated/EncoderCounts.h"

#include "EncoderCountsPublisherEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
/** Default constructor, advertises on the ROS @e "encoder_counts" topic.
 */
EncoderCountsPublisherEndpoint::EncoderCountsPublisherEndpoint() 
  // Setup topic for publishing laser scans to
  : _encoder_counts_publisher(
    _encoder_counts_node.advertise<diff_drive_calibrated::EncoderCounts>("encoder_counts", 10)) 
{ 
}

/** Publishes the @p encoder_counts msg and broadcasts the coordinate frame 
 *  transform.
 */
void EncoderCountsPublisherEndpoint::Publish( const diff_drive_calibrated::EncoderCounts& encoder_counts )
{
  // and publish the encoder_counts msg
  _encoder_counts_publisher.publish(encoder_counts);

  ros::spinOnce();

  ROS_DEBUG(  "Published encoder_counts with L: %d R: %d", 
              encoder_counts.left_count, encoder_counts.right_count );
}
}
