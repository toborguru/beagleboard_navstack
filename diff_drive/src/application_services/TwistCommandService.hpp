// TwistCommandService.hpp
 
#ifndef GUARD_TwistCommandService
#define GUARD_TwistCommandService
 
#include <boost/shared_ptr.hpp>
#include "ITickVelocityEndpoint.hpp"
#include "ITwistEndpoint.hpp"
#include "BaseModel.hpp"
 
namespace diff_drive_application_services
{
  class TwistCommandService
  {
    public:
      explicit TwistCommandService( boost::shared_ptr<diff_drive_core::ITickVelocityEndpoint> tick_velocity_endpoint,
                                    boost::shared_ptr<diff_drive_core::ITwistEndpoint>        twist_endpoint,
                                    boost::shared_ptr<const diff_drive_core::BaseModel>       base_model );

      void BeginAcceptingCommands();
      void StopAcceptingCommands();
 
    private:
      // Forward declare the implementation class
      class TwistCommandServiceImpl;
      boost::shared_ptr<TwistCommandServiceImpl> _p_impl;
  };
}
 
#endif /* GUARD_TwistCommandService */
