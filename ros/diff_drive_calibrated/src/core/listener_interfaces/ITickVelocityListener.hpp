// ITickVelocityListener.hpp
 
#ifndef GUARD_ITickVelocityListener
#define GUARD_ITickVelocityListener

#include "diff_drive_calibrated/TickVelocity.h"
 
namespace diff_drive_core
{
  class ITickVelocityListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~ITickVelocityListener() {}
      virtual void onTickVelocityAvailableEvent( const diff_drive_calibrated::TickVelocity& tick_velocity ) = 0;
  };
}
 
#endif /* GUARD_ITickVelocityListener */
