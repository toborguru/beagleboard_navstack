// IEncoderCountsSubscriberEndpoint.hpp

#ifndef GUARD_IEncoderCountsSubscriberEndpoint
#define GUARD_IEncoderCountsSubscriberEndpoint

#include "IEncoderCountsListener.hpp"

namespace differential_drive_core
{
class IEncoderCountsSubscriberEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IEncoderCountsSubscriberEndpoint() {}
  virtual void Subscribe() = 0;
  virtual void Unsubscribe() = 0;
  virtual bool IsSubscribed() = 0;
  virtual void attach( IEncoderCountsListener& encoder_counts_listener ) = 0; 
  virtual void detach( IEncoderCountsListener& encoder_counts_listener ) = 0; 
};
}

#endif /* GUARD_IEncoderCountsSubscriberEndpoint */
