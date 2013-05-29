// IMovementStatusListener.hpp
 
#ifndef GUARD_IMovementStatusListener
#define GUARD_IMovementStatusListener

#include "differential_drive/MovementStatus.h"
 
namespace differential_drive_core
{
  class IMovementStatusListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IMovementStatusListener() {}
      virtual void onMovementStatusAvailableEvent( const differential_drive::MovementStatus& status ) = 0;
  };
}
 
#endif /* GUARD_IMovementStatusListener */
