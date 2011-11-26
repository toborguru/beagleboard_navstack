// ITickVelocityEndpoint.hpp

#ifndef GUARD_ITickVelocityEndpoint
#define GUARD_ITickVelocityEndpoint

#include "ITickVelocityListener.hpp"

namespace data_robot_core
{
class ITickVelocityEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~ITickVelocityEndpoint() {}
  virtual void Subscribe() = 0;
  virtual void Unsubscribe() = 0;
  virtual void Attach( ITickVelocityListener& tick_velocity_listener ) = 0; 
};
}

#endif /* GUARD_ITickVelocityEndpoint */
