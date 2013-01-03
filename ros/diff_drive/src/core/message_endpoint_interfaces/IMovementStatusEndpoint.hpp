// IMovementStatusEndpoint.hpp
 
#ifndef GUARD_IMovementStatusEndpoint
#define GUARD_IMovementStatusEndpoint
 
#include "diff_drive/MovementStatus.h"
 
namespace diff_drive_core
{
  class IMovementStatusEndpoint
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IMovementStatusEndpoint() {}
      virtual void Publish(const diff_drive::MovementStatus& status) = 0;
  };
}
 
#endif /* GUARD_IMovementStatusEndpoint */
