// TwistCommandService.cpp
 
#include "TwistCommandService.hpp"
 
using namespace differential_drive_core;
 
namespace differential_drive_application_services
{
/** Default constructor.
 *
 *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
 *  of objects. The object pointed to will be destroyed when all pointers to the object have been
 *  destroyed.
 */
TwistCommandService::TwistCommandService( boost::shared_ptr<ITickVelocityPublisherEndpoint> tick_velocity_endpoint,
                                          boost::shared_ptr<ITwistSubscriberEndpoint> twist_endpoint, 
                                          boost::shared_ptr<const differential_drive_core::BaseModel> base_model )
: _p_tick_velocity_endpoint( tick_velocity_endpoint ),
  _p_twist_endpoint( twist_endpoint ),
  _p_base_model( base_model )
{
  _twist_converter.SetBaseModel(*_p_base_model );
}

/** Do everything required to start listening for twist commands and passing them on as tick_velocity.
 */
void TwistCommandService::BeginAcceptingCommands() 
{
  _twist_converter.Attach( *_p_tick_velocity_endpoint );
  _p_twist_endpoint->Attach( _twist_converter );
}

/** Do everything required to stop listening for twist commands and passing them on as tick_velocity.
 */
void TwistCommandService::StopAcceptingCommands()
{
  _twist_converter.Detach( *_p_tick_velocity_endpoint );
  _p_twist_endpoint->Detach( _twist_converter );
}
}
