// TwistSubscriberEndpointStub.hpp
 
#ifndef GUARD_TwistSubscriberEndpointStub
#define GUARD_TwistSubscriberEndpointStub
 
#include "ITwistSubscriberEndpoint.hpp"
 
namespace differential_drive_test_message_endpoints_test_doubles
{
class TwistSubscriberEndpointStub : public differential_drive_core::ITwistSubscriberEndpoint
{ 
public:
  TwistSubscriberEndpointStub() 
    : _subscribed(false)
  {}

  void AddTicks( const geometry_msgs::Twist twist )
  { 
    for (unsigned int i= 0; i < _twist_listeners.size(); i++)
    { 
      _twist_listeners[i]->OnTwistAvailableEvent(twist);
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

  void Attach( differential_drive_core::ITwistListener& twist_listener )
  { 
    _twist_listeners.push_back(&twist_listener);
  }

private:
  bool _subscribed;
  std::vector<differential_drive_core::ITwistListener*> _twist_listeners;
};
}
 
#endif /* GUARD_TwistSubscriberEndpointStub */
