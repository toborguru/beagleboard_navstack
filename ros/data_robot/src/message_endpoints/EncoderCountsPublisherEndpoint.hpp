// EncoderCountsPublisherEndpoint.hpp
 
#ifndef GUARD_EncoderCountsPublisherEndpoint
#define GUARD_EncoderCountsPublisherEndpoint
 
#include <ros/ros.h>

#include "differential_drive/EncoderCounts.h"

#include "IEncoderCountsPublisherEndpoint.hpp"
 
namespace data_robot_message_endpoints
{
  class EncoderCountsPublisherEndpoint : public data_robot_core::IEncoderCountsPublisherEndpoint
  { 
    public:
      EncoderCountsPublisherEndpoint();

      virtual ~EncoderCountsPublisherEndpoint() {};
 
      virtual void Publish( const differential_drive::EncoderCounts& encoder_counts );
 
    private:
      // Create handle to node
      ros::NodeHandle _encoder_counts_node;
 
      ros::Publisher _encoder_counts_publisher;
  };
}
 
#endif /* GUARD_EncoderCountsPublisherEndpoint */