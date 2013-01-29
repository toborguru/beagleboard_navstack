// ITickVelocitySubscriberEndpoint.hpp

#ifndef GUARD_ITickVelocitySubscriberEndpoint
#define GUARD_ITickVelocitySubscriberEndpoint

#include "ITickVelocityListener.hpp"

namespace data_robot_core
{
class ITickVelocitySubscriberEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~ITickVelocitySubscriberEndpoint() {}
  virtual void Subscribe() = 0;
  virtual void Unsubscribe() = 0;
  virtual void Attach( ITickVelocityListener& tick_velocity_listener ) = 0; 
};
}

#endif /* GUARD_ITickVelocitySubscriberEndpoint */
