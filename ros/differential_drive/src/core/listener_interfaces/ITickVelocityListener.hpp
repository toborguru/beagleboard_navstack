// ITickVelocityListener.hpp
 
#ifndef GUARD_ITickVelocityListener
#define GUARD_ITickVelocityListener

#include "differential_drive/TickVelocity.h"
 
namespace differential_drive_core
{
  class ITickVelocityListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~ITickVelocityListener() {}
      virtual void OnTickVelocityAvailableEvent( const differential_drive::TickVelocity& tick_velocity ) = 0;
  };
}
 
#endif /* GUARD_ITickVelocityListener */
