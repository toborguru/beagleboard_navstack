// TwistCommandService.cpp
 
#include "TwistCommandService.hpp"
 
using namespace diff_drive_core;
 
namespace diff_drive_application_services
{
/** Default constructor.
*
*  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
*  of objects. The object pointed to will be destroyed when all pointers to the object have been
*  destroyed.
*/
TwistCommandService::TwistCommandService( boost::shared_ptr<ITickVelocityEndpoint> tick_velocity_endpoint,
                                          boost::shared_ptr<ITwistEndpoint> twist_endpoint, 
                                          boost::shared_ptr<const diff_drive_core::BaseModel> base_model )
: _p_tick_velocity_endpoint( tick_velocity_endpoint ),
  _p_twist_endpoint( twist_endpoint ),
  _p_base_model( base_model ),
  _is_accepting_commands( false )
{
  _p_twist_endpoint->Attach( _twist_converter );

  _twist_converter.Attach( *this );
  _twist_converter.SetBaseModel(*_p_base_model );
}

/** Do everything required to start listening for twist commands and passing them on as tick_velocity.
*/
void TwistCommandService::BeginAcceptingCommands() 
{
  _is_accepting_commands = true;

  _p_twist_endpoint->Subscribe();
}

/** Do everything required to stop listening for twist commands and passing them on as tick_velocity.
*/
void TwistCommandService::StopAcceptingCommands()
{
  _p_twist_endpoint->Unsubscribe();

  _is_accepting_commands = false;
}

/** This class is a tick velocity listener, act on new tick velocity available.
*/
void TwistCommandService::OnTickVelocityAvailableEvent(const diff_drive::TickVelocity& tick_velocity)
{
  if ( _is_accepting_commands )
  {
    // Send tick_velocity to the message end point
    _p_tick_velocity_endpoint->Publish( tick_velocity );
  }
}
}
