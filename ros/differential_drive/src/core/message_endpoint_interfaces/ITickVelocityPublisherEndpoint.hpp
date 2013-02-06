// ITickVelocityPublisherEndpoint.hpp
 
#ifndef GUARD_ITickVelocityPublisherEndpoint
#define GUARD_ITickVelocityPublisherEndpoint
 
#include "differential_drive/TickVelocity.h"

#include "ITickVelocityListener.hpp"

namespace differential_drive_core
{
class ITickVelocityPublisherEndpoint : public ITickVelocityListener
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~ITickVelocityPublisherEndpoint() {}
  virtual void Publish(const differential_drive::TickVelocity& tick_velocity) const = 0;
};
}
 
#endif /* GUARD_ITickVelocityPublisherEndpoint */
