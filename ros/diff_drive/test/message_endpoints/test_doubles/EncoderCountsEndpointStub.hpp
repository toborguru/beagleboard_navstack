// EncoderCountsEndpointStub.hpp
 
#ifndef GUARD_EncoderCountsEndpointStub
#define GUARD_EncoderCountsEndpointStub
 
#include "IEncoderCountsEndpoint.hpp"
 
namespace diff_drive_test_message_endpoints_test_doubles
{
class EncoderCountsEndpointStub : public diff_drive_core::IEncoderCountsEndpoint
{ 
public:
  EncoderCountsEndpointStub() 
    : _subscribed(false)
  {}

  void AddTicks( const diff_drive::EncoderCounts encoder_counts )
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

  void Attach( diff_drive_core::IEncoderCountsListener& encoder_counts_listener )
  { 
    _encoder_counts_listeners.push_back(&encoder_counts_listener);
  }

private:
  bool _subscribed;
  std::vector<diff_drive_core::IEncoderCountsListener*> _encoder_counts_listeners;
};
}
 
#endif /* GUARD_EncoderCountsEndpointStub */
