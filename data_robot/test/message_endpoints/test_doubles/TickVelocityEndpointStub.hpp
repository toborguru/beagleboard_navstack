// TickVelocityEndpointStub.hpp
 
#ifndef GUARD_TickVelocityEndpointStub
#define GUARD_TickVelocityEndpointStub
 
#include "ITickVelocityEndpoint.hpp"
 
namespace data_robot_test_message_endpoints_test_doubles
{
class TickVelocityEndpointStub : public data_robot_core::ITickVelocityEndpoint
{ 
public:
  TickVelocityEndpointStub() 
    : _subscribed(false)
  {}

  void AddTicks( const diff_drive::TickVelocity tick_velocity )
  { 
    if ( _subscribed )
    {
      for (unsigned int i= 0; i < _tick_velocity_listeners.size(); i++)
      { 
        _tick_velocity_listeners[i]->OnTickVelocityAvailableEvent(tick_velocity);
      }
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

  void Attach( data_robot_core::ITickVelocityListener& tick_velocity_listener )
  { 
    _tick_velocity_listeners.push_back(&tick_velocity_listener);
  }

private:
  bool _subscribed;
  std::vector<data_robot_core::ITickVelocityListener*> _tick_velocity_listeners;
};
}
 
#endif /* GUARD_TickVelocityEndpointStub */
