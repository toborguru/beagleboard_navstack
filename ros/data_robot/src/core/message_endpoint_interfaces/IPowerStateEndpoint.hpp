// IPowerStateEndpoint.hpp
 
#ifndef GUARD_IPowerStateEndpoint
#define GUARD_IPowerStateEndpoint
 
#include "data_robot/PowerState.h"
 
namespace data_robot_core
{
  class IPowerStateEndpoint
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IPowerStateEndpoint() {}
      virtual void Publish( const data_robot::PowerState& power_state ) = 0;
  };
}
 
#endif /* GUARD_IPowerStateEndpoint */
