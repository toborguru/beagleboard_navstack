// IEncoderCountsPublisherEndpoint.hpp
 
#ifndef GUARD_IEncoderCountsPublisherEndpoint
#define GUARD_IEncoderCountsPublisherEndpoint
 
#include "diff_drive/EncoderCounts.h"
 
namespace data_robot_core
{
class IEncoderCountsPublisherEndpoint
{
  public:
    // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
    virtual ~IEncoderCountsPublisherEndpoint() {}
    virtual void Publish(const diff_drive::EncoderCounts& encoder_counts) = 0;
};
}
 
#endif /* GUARD_IEncoderCountsPublisherEndpoint */
