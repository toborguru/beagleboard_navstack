// ITickVelocityEndpoint.hpp
 
#ifndef GUARD_ITickVelocityEndpoint
#define GUARD_ITickVelocityEndpoint
 
#include "diff_drive/TickVelocity.h"
 
namespace diff_drive_core
{
  class ITickVelocityEndpoint
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~ITickVelocityEndpoint() {}
      virtual void Publish(const diff_drive::TickVelocity& tick_velocity) const = 0;
  };
}
 
#endif /* GUARD_ITickVelocityEndpoint */
