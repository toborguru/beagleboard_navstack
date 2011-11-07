// TickVelocityForwarder.hpp
 
#ifndef GUARD_TickVelocityForwarder
#define GUARD_TickVelocityForwarder
 
#include "diff_drive/TickVelocity.h"

#include "ITickVelocityListener.hpp"
#include "IExternalBusEndpoint.hpp"
#include "MotorVelocityRequest.hpp"

namespace data_robot_core
{
class TickVelocityForwarder : public ITickVelocityListener
{
public:
  TickVelocityForwarder();

  void SetExternalBus( IExternalBusEndpoint *p_external_bus );

  void OnTickVelocityAvailableEvent( const diff_drive::TickVelocity& tick_velocity );

private:
  MotorVelocityRequest  _velocity_request;
  IExternalBusEndpoint *_p_external_bus;
};
}

#endif /* GUARD_TickVelocityForwarder */
