// IMovementStatusPublisherEndpoint.hpp
 
#ifndef GUARD_IMovementStatusPublisherEndpoint
#define GUARD_IMovementStatusPublisherEndpoint
 
#include "differential_drive/MovementStatus.h"

#include "IMovementStatusListener.hpp"

namespace differential_drive_core
{
class IMovementStatusPublisherEndpoint : public IMovementStatusListener
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IMovementStatusPublisherEndpoint() {}
  virtual void Publish(const differential_drive::MovementStatus& status) = 0;
};
}
 
#endif /* GUARD_IMovementStatusPublisherEndpoint */
