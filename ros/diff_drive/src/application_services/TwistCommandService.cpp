// TwistCommandService.cpp
 
#include "diff_drive/TickVelocity.h"

#include "ITickVelocityListener.hpp"
#include "TwistConverter.hpp"
#include "TwistCommandService.hpp"
 
using namespace diff_drive_core;
 
namespace diff_drive_application_services
{
  /** Private implementation of TwistCommandService
   */
  class TwistCommandService::TwistCommandServiceImpl : public diff_drive_core::ITickVelocityListener
  {
    public:
      explicit TwistCommandServiceImpl(  boost::shared_ptr<ITickVelocityEndpoint> tick_velocity_endpoint,
                                              boost::shared_ptr<ITwistEndpoint> twist_endpoint,
                                              boost::shared_ptr<const diff_drive_core::BaseModel> base_model );

      void OnTickVelocityAvailableEvent(  const diff_drive::TickVelocity& tick_velocity );

      TwistConverter twist_converter;
 
      boost::shared_ptr<ITickVelocityEndpoint>            p_tick_velocity_endpoint;
      boost::shared_ptr<ITwistEndpoint>                   p_twist_endpoint; 
      boost::shared_ptr<const diff_drive_core::BaseModel> p_base_model;

      bool  is_accepting_commands;
    private:
  };

  /** Default constructor.
   *
   *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
   *  of objects. The object pointed to will be destroyed when all pointers to the object have been
   *  destroyed.
   */
  TwistCommandService::TwistCommandService( boost::shared_ptr<ITickVelocityEndpoint> tick_velocity_endpoint,
                                                      boost::shared_ptr<ITwistEndpoint> twist_endpoint, 
                                                      boost::shared_ptr<const diff_drive_core::BaseModel> base_model )
    : _p_impl(new TwistCommandServiceImpl( tick_velocity_endpoint, twist_endpoint, base_model )) 
  {
  }
 
  /** Default Impl constructor.
   *
   *  All the objects a growing class needs! boost::shared_ptr allows for easy dynamic allocation
   *  of objects. The object pointed to will be destroyed when all pointers to the object have been
   *  destroyed.
   */
  TwistCommandService::TwistCommandServiceImpl::TwistCommandServiceImpl(
                                                                boost::shared_ptr<ITickVelocityEndpoint> tick_velocity_endpoint,
                                                                boost::shared_ptr<ITwistEndpoint> twist_endpoint,
                                                                boost::shared_ptr<const diff_drive_core::BaseModel> base_model )
    : p_tick_velocity_endpoint( tick_velocity_endpoint ),
      p_twist_endpoint( twist_endpoint ),
      p_base_model( base_model ),
      is_accepting_commands( false )
  { 
    p_twist_endpoint->Attach( twist_converter );

    twist_converter.Attach( *this);
    twist_converter.SetBaseModel(*p_base_model );
  }

  /** Do everything required to start listening for twist commands and passing them on as tick_velocity.
   */
  void TwistCommandService::BeginAcceptingCommands() 
  {
    _p_impl->is_accepting_commands = true;

    _p_impl->p_twist_endpoint->Subscribe();
  }
 
  /** Do everything required to stop listening for twist commands and passing them on as tick_velocity.
   */
  void TwistCommandService::StopAcceptingCommands()
  {
    _p_impl->p_twist_endpoint->Unsubscribe();
  
    _p_impl->is_accepting_commands = false;
  }

  /** This class is a tick velocity listener, act on new tick velocity available.
   */
  void TwistCommandService::TwistCommandServiceImpl::OnTickVelocityAvailableEvent(const diff_drive::TickVelocity& tick_velocity)
  {
    if ( is_accepting_commands )
    {
      // Send tick_velocity to the message end point
      p_tick_velocity_endpoint->Publish( tick_velocity );
    }
  };
}
