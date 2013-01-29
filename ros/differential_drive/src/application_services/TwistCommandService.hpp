// TwistCommandService.hpp
 
#ifndef GUARD_TwistCommandService
#define GUARD_TwistCommandService
 
#include <boost/shared_ptr.hpp>

#include "differential_drive/TickVelocity.h"

#include "BaseModel.hpp"
#include "TwistConverter.hpp"
#include "TwistConverter.hpp"
#include "ITickVelocityListener.hpp"
#include "ITickVelocityPublisherEndpoint.hpp"
#include "ITwistSubscriberEndpoint.hpp"
 
namespace differential_drive_application_services
{
class TwistCommandService : public differential_drive_core::ITickVelocityListener
{
public:
  explicit TwistCommandService( boost::shared_ptr<differential_drive_core::ITickVelocityPublisherEndpoint> tick_velocity_endpoint,
                                boost::shared_ptr<differential_drive_core::ITwistSubscriberEndpoint>        twist_endpoint,
                                boost::shared_ptr<const differential_drive_core::BaseModel>       base_model );

  void OnTickVelocityAvailableEvent(const differential_drive::TickVelocity& tick_velocity);

  void BeginAcceptingCommands();
  void StopAcceptingCommands();

private:
  differential_drive_core::TwistConverter _twist_converter;

  boost::shared_ptr<differential_drive_core::ITickVelocityPublisherEndpoint> _p_tick_velocity_endpoint;
  boost::shared_ptr<differential_drive_core::ITwistSubscriberEndpoint>        _p_twist_endpoint;
  boost::shared_ptr<const differential_drive_core::BaseModel>       _p_base_model;

  bool  _is_accepting_commands;
};
}
 
#endif /* GUARD_TwistCommandService */