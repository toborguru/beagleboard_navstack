// IPowerStateListener.hpp
 
#ifndef GUARD_IPowerStateListener
#define GUARD_IPowerStateListener

#include "data_robot/PowerState.h"
 
namespace data_robot_core
{
  class IPowerStateListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IPowerStateListener() {}
      virtual void OnPowerStateAvailableEvent( const data_robot::PowerState& power_state ) = 0;
  };
}
 
#endif /* GUARD_IPowerStateListener */
