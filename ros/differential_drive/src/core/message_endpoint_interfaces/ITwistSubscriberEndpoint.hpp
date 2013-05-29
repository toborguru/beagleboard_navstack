// ITwistSubscriberEndpoint.hpp

#ifndef GUARD_ITwistSubscriberEndpoint
#define GUARD_ITwistSubscriberEndpoint

#include "ITwistListener.hpp"

namespace differential_drive_core
{
class ITwistSubscriberEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~ITwistSubscriberEndpoint() {}
  virtual void Subscribe() = 0;
  virtual void Unsubscribe() = 0;
  virtual bool IsSubscribed() = 0;
  virtual void attach( ITwistListener& twist_listener ) = 0; 
  virtual void detach( ITwistListener& twist_listener ) = 0; 
};
}

#endif /* GUARD_ITwistSubscriberEndpoint */
