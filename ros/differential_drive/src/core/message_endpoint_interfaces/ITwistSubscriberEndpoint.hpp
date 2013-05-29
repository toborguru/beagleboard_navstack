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
  virtual void subscribe() = 0;
  virtual void unsubscribe() = 0;
  virtual bool isSubscribed() = 0;
  virtual void attach( ITwistListener& twist_listener ) = 0; 
  virtual void detach( ITwistListener& twist_listener ) = 0; 
};
}

#endif /* GUARD_ITwistSubscriberEndpoint */
