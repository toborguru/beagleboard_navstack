// TickVelocityCommandService.cpp
 
#include "TickVelocityCommandService.hpp"
 
using namespace data_robot_core;
 
namespace data_robot_application_services
{
/** Default  constructor.
 *
 *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
 *  of objects. The object pointed to will be destroyed when all pointers to the object have been
 *  destroyed.
 */
TickVelocityCommandService::TickVelocityCommandService( boost::shared_ptr<ITickVelocitySubscriberEndpoint> tick_velocity_endpoint,
                                          boost::shared_ptr<IBusRequestProcessorEndpoint> external_bus_endpoint,
                                          boost::shared_ptr<ILogPublisherEndpoint> log_endpoint )
  : _p_tick_velocity_endpoint( tick_velocity_endpoint ),
    _p_log_endpoint( log_endpoint ),
    _p_external_bus_endpoint( external_bus_endpoint ),
    _is_accepting_commands( false )
{ 
  _p_tick_velocity_endpoint->Attach( _tick_velocity_forwarder ); 
  _tick_velocity_forwarder.SetExternalBus( _p_external_bus_endpoint.get() );
}

/** Do everything required to start listening for tick_velocity commands and passing them on to the robot.
 */
void TickVelocityCommandService::BeginAcceptingCommands() 
{
  _is_accepting_commands = true;

  _p_tick_velocity_endpoint->Subscribe();

  Log( INFO, NONE, "Tick Velocity Command Service: Started\n" );
}

/** Do everything required to stop listening for tick_velocity commands and passing them on to the robot.
 */
void TickVelocityCommandService::StopAcceptingCommands()
{
  _p_tick_velocity_endpoint->Unsubscribe();

  _is_accepting_commands = false;

  Log( INFO, NONE, "Tick Velocity Command Service: Stopped\n" );
}

void TickVelocityCommandService::Log( data_robot_core::LogLevel_T level, data_robot_core::LogFlags_T flags, const std::string& log_message )
{
  if ( _p_log_endpoint != NULL )
  {
    _p_log_endpoint->Publish( level, flags, log_message );
  }
}
}
