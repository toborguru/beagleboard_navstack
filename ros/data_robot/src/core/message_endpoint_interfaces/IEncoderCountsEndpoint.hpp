// IEncoderCountsEndpoint.hpp
 
#ifndef GUARD_IEncoderCountsEndpoint
#define GUARD_IEncoderCountsEndpoint
 
#include "diff_drive/EncoderCounts.h"
 
namespace data_robot_core
{
  class IEncoderCountsEndpoint
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IEncoderCountsEndpoint() {}
      virtual void Publish(const diff_drive::EncoderCounts& encoder_counts) = 0;
  };
}
 
#endif /* GUARD_IEncoderCountsEndpoint */
