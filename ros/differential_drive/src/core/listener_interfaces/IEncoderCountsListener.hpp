// IEncoderCountsListener.hpp
 
#ifndef GUARD_IEncoderCountsListener
#define GUARD_IEncoderCountsListener

#include "differential_drive/EncoderCounts.h"
 
namespace differential_drive_core
{
  class IEncoderCountsListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IEncoderCountsListener() {}
      virtual void OnEncoderCountsAvailableEvent( const differential_drive::EncoderCounts& encoder_counts ) = 0;
  };
}
 
#endif /* GUARD_IEncoderCountsListener */
