// EncoderCountsEndpoint.hpp
 
#ifndef GUARD_EncoderCountsEndpoint
#define GUARD_EncoderCountsEndpoint
 
#include <ros/ros.h>

#include "diff_drive/EncoderCounts.h"

#include "IEncoderCountsEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
  class EncoderCountsEndpoint : public data_robot_core::IEncoderCountsEndpoint
  { 
    public:
      EncoderCountsEndpoint();

      virtual ~EncoderCountsEndpoint() {};
 
      virtual void Publish(const diff_drive::EncoderCounts& encoder_counts_scan);
 
    private:
      // Create handle to node
      ros::NodeHandle _encoder_counts_node;
 
      ros::Publisher _encoder_counts_publisher;
  };
}
 
#endif /* GUARD_EncoderCountsEndpoint */
