// EncoderCountsSubscriberEndpointStub.hpp
 
#ifndef GUARD_EncoderCountsSubscriberEndpointStub
#define GUARD_EncoderCountsSubscriberEndpointStub
 
#include "IEncoderCountsSubscriberEndpoint.hpp"
 
namespace differential_drive_test_message_endpoints_test_doubles
{
class EncoderCountsSubscriberEndpointStub : public differential_drive_core::IEncoderCountsSubscriberEndpoint
{ 
public:
  EncoderCountsSubscriberEndpointStub() 
    : _subscribed(false)
  {}

  void AddTicks( const differential_drive::EncoderCounts encoder_counts )
  { 
    for (unsigned int i= 0; i < _encoder_counts_listeners.size(); i++)
    { 
      _encoder_counts_listeners[i]->OnEncoderCountsAvailableEvent(encoder_counts);
    }
  }

  void Subscribe()
  { 
    _subscribed = true;
  }

  void Unsubscribe()
  { 
    _subscribed = false;
  }

  bool IsSubscribed()
  { 
  return _subscribed;
  }

  void Attach( differential_drive_core::IEncoderCountsListener& encoder_counts_listener )
  { 
    _encoder_counts_listeners.push_back(&encoder_counts_listener);
  }

private:
  bool _subscribed;
  std::vector<differential_drive_core::IEncoderCountsListener*> _encoder_counts_listeners;
};
}
 
#endif /* GUARD_EncoderCountsSubscriberEndpointStub */