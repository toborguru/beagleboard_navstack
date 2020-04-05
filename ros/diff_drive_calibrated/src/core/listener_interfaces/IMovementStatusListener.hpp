// IMovementStatusListener.hpp
 
#ifndef GUARD_IMovementStatusListener
#define GUARD_IMovementStatusListener

#include "diff_drive_calibrated/MovementStatus.h"
 
namespace diff_drive_core
{
  class IMovementStatusListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IMovementStatusListener() {}
      virtual void onMovementStatusAvailableEvent( const diff_drive_calibrated::MovementStatus& status ) = 0;
  };
}
 
#endif /* GUARD_IMovementStatusListener */
