// IEncoderCountsSubscriberEndpoint.hpp

#ifndef GUARD_IEncoderCountsSubscriberEndpoint
#define GUARD_IEncoderCountsSubscriberEndpoint

#include "IEncoderCountsListener.hpp"

namespace diff_drive_core
{
class IEncoderCountsSubscriberEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IEncoderCountsSubscriberEndpoint() {}
  virtual void subscribe() = 0;
  virtual void unsubscribe() = 0;
  virtual bool isSubscribed() = 0;
  virtual void attach( IEncoderCountsListener& encoder_counts_listener ) = 0; 
  virtual void detach( IEncoderCountsListener& encoder_counts_listener ) = 0; 
};
}

#endif /* GUARD_IEncoderCountsSubscriberEndpoint */
