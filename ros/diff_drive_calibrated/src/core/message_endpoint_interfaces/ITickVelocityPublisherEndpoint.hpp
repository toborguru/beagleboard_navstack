// ITickVelocityPublisherEndpoint.hpp
 
#ifndef GUARD_ITickVelocityPublisherEndpoint
#define GUARD_ITickVelocityPublisherEndpoint
 
#include "diff_drive_calibrated/TickVelocity.h"

#include "ITickVelocityListener.hpp"

namespace diff_drive_core
{
class ITickVelocityPublisherEndpoint : public ITickVelocityListener
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~ITickVelocityPublisherEndpoint() {}
  virtual void publish(const diff_drive_calibrated::TickVelocity& tick_velocity) const = 0;
};
}
 
#endif /* GUARD_ITickVelocityPublisherEndpoint */
