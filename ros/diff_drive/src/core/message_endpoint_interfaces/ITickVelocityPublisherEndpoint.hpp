// ITickVelocityPublisherEndpoint.hpp
 
#ifndef GUARD_ITickVelocityPublisherEndpoint
#define GUARD_ITickVelocityPublisherEndpoint
 
#include "diff_drive/TickVelocity.h"
 
namespace diff_drive_core
{
class ITickVelocityPublisherEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~ITickVelocityPublisherEndpoint() {}
  virtual void Publish(const diff_drive::TickVelocity& tick_velocity) const = 0;
};
}
 
#endif /* GUARD_ITickVelocityPublisherEndpoint */
