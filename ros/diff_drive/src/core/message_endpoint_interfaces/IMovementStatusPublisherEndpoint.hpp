// IMovementStatusPublisherEndpoint.hpp
 
#ifndef GUARD_IMovementStatusPublisherEndpoint
#define GUARD_IMovementStatusPublisherEndpoint
 
#include "diff_drive/MovementStatus.h"
 
namespace diff_drive_core
{
class IMovementStatusPublisherEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IMovementStatusPublisherEndpoint() {}
  virtual void Publish(const diff_drive::MovementStatus& status) = 0;
};
}
 
#endif /* GUARD_IMovementStatusPublisherEndpoint */
