// IEncoderCountsListener.hpp
 
#ifndef GUARD_IEncoderCountsListener
#define GUARD_IEncoderCountsListener

#include "diff_drive/EncoderCounts.h"
 
namespace data_robot_core
{
  class IEncoderCountsListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IEncoderCountsListener() {}
      virtual void OnEncoderCountsAvailableEvent( const diff_drive::EncoderCounts& telemetry ) = 0;
  };
}
 
#endif /* GUARD_IEncoderCountsListener */
