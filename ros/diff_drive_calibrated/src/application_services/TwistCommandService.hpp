// TwistCommandService.hpp
 
#ifndef GUARD_TwistCommandService
#define GUARD_TwistCommandService
 
#include <boost/shared_ptr.hpp>

#include "diff_drive_calibrated/TickVelocity.h"

#include "BaseModel.hpp"
#include "TwistConverter.hpp"
#include "TwistConverter.hpp"
#include "ITickVelocityPublisherEndpoint.hpp"
#include "ITwistSubscriberEndpoint.hpp"
 
namespace diff_drive_application_services
{
class TwistCommandService
{
public:
  explicit TwistCommandService( boost::shared_ptr<diff_drive_core::ITickVelocityPublisherEndpoint> tick_velocity_endpoint,
                                boost::shared_ptr<diff_drive_core::ITwistSubscriberEndpoint>        twist_endpoint,
                                boost::shared_ptr<const diff_drive_core::BaseModel>       base_model );

  void startAcceptingCommands();
  void stopAcceptingCommands();

private:
  diff_drive_core::TwistConverter _twist_converter;

  boost::shared_ptr<diff_drive_core::ITickVelocityPublisherEndpoint> _p_tick_velocity_endpoint;
  boost::shared_ptr<diff_drive_core::ITwistSubscriberEndpoint>        _p_twist_endpoint;
  boost::shared_ptr<const diff_drive_core::BaseModel>       _p_base_model;
};
}
 
#endif /* GUARD_TwistCommandService */
