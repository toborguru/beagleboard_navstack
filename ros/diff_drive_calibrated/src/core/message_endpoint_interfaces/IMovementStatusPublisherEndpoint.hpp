// IMovementStatusPublisherEndpoint.hpp
 
#ifndef GUARD_IMovementStatusPublisherEndpoint
#define GUARD_IMovementStatusPublisherEndpoint
 
#include "diff_drive_calibrated/MovementStatus.h"

#include "IMovementStatusListener.hpp"

namespace diff_drive_core
{
class IMovementStatusPublisherEndpoint : public IMovementStatusListener
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IMovementStatusPublisherEndpoint() {}
  virtual void publish(const diff_drive_calibrated::MovementStatus& status) = 0;
};
}
 
#endif /* GUARD_IMovementStatusPublisherEndpoint */
