// TickVelocityCommandService.hpp
 
#ifndef GUARD_TickVelocityCommandService
#define GUARD_TickVelocityCommandService
 
#include <boost/shared_ptr.hpp>

#include "ITickVelocitySubscriberEndpoint.hpp"
#include "IBusRequestProcessorEndpoint.hpp"
#include "ILogPublisherEndpoint.hpp"

#include "TickVelocityForwarder.hpp"
 
namespace data_robot_application_services
{
class TickVelocityCommandService
{
public:
  explicit TickVelocityCommandService(  boost::shared_ptr<data_robot_core::ITickVelocitySubscriberEndpoint> tick_velocity_endpoint,
                                        boost::shared_ptr<data_robot_core::IBusRequestProcessorEndpoint>  external_bus_endpoint,  
                                        boost::shared_ptr<data_robot_core::ILogPublisherEndpoint> log_endpoint );

  void BeginAcceptingCommands();
  void StopAcceptingCommands();

  void OnTickVelocityAvailableEvent(  const diff_drive_calibrated::TickVelocity& tick_velocity );

private:
  void Log( data_robot_core::LogLevel_T level, data_robot_core::LogFlags_T flags, const std::string& log_message );

  data_robot_core::TickVelocityForwarder  _tick_velocity_forwarder;

  boost::shared_ptr<data_robot_core::ITickVelocitySubscriberEndpoint> _p_tick_velocity_endpoint;
  boost::shared_ptr<data_robot_core::ILogPublisherEndpoint>           _p_log_endpoint;
  boost::shared_ptr<data_robot_core::IBusRequestProcessorEndpoint>    _p_external_bus_endpoint; 

  bool  _is_accepting_commands;
};
}
 
#endif /* GUARD_TickVelocityCommandService */
