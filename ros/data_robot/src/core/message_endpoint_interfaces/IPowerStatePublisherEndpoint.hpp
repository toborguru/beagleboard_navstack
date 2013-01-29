// IPowerStatePublisherEndpoint.hpp
 
#ifndef GUARD_IPowerStatePublisherEndpoint
#define GUARD_IPowerStatePublisherEndpoint
 
#include "data_robot/PowerState.h"
 
namespace data_robot_core
{
  class IPowerStatePublisherEndpoint
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IPowerStatePublisherEndpoint() {}
      virtual void Publish( const data_robot::PowerState& power_state ) = 0;
  };
}
 
#endif /* GUARD_IPowerStatePublisherEndpoint */
