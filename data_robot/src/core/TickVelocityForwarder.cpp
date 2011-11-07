
#include "TickVelocityForwarder.hpp"

namespace data_robot_core
{
/** Default constructor.
 */
TickVelocityForwarder::TickVelocityForwarder()
{
}

/** Sets the bus to send the request to.
 *  
 *  Setting this value to NULL will disable the forwarder.
 */
void TickVelocityForwarder::SetExternalBus( IExternalBusEndpoint *p_external_bus )
{
  _p_external_bus = p_external_bus;
}

/** Callback for ITickVelocityListener
 */
void TickVelocityForwarder::OnTickVelocityAvailableEvent( const diff_drive::TickVelocity& tick_velocity )
{
  _velocity_request.Lock();
  _velocity_request.SetVelocity( tick_velocity.linear_ticks_sec, tick_velocity.angular_ticks_sec );
  _velocity_request.Unlock();

  if ( _p_external_bus != NULL )
  {
    _p_external_bus->ProcessRequest( &_velocity_request );
  }
}
}
