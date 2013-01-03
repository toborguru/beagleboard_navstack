// TwistEndpointStub.hpp
 
#ifndef GUARD_TwistEndpointStub
#define GUARD_TwistEndpointStub
 
#include "ITwistEndpoint.hpp"
 
namespace diff_drive_test_message_endpoints_test_doubles
{
class TwistEndpointStub : public diff_drive_core::ITwistEndpoint
{ 
public:
  TwistEndpointStub() 
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

  void Attach( diff_drive_core::ITwistListener& twist_listener )
  { 
    _twist_listeners.push_back(&twist_listener);
  }

private:
  bool _subscribed;
  std::vector<diff_drive_core::ITwistListener*> _twist_listeners;
};
}
 
#endif /* GUARD_TwistEndpointStub */
