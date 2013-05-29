// TwistConverter.hpp
 
#ifndef GUARD_TwistConverter
#define GUARD_TwistConverter
 
#include <vector>

#include "geometry_msgs/Twist.h"
#include "differential_drive/TickVelocity.h"

#include "BaseModel.hpp"
#include "ITickVelocityListener.hpp"
#include "ITwistListener.hpp"

namespace differential_drive_core
{
  class TwistConverter : public ITwistListener
  {
    public:
      TwistConverter();

      void attach( ITickVelocityListener& tick_velocity_listener );
      void detach( ITickVelocityListener& tick_velocity_listener );

      void setBaseModel( const BaseModel& base_model );

      void onTwistAvailableEvent( const geometry_msgs::Twist& twist );
 
    private:
      differential_drive::TickVelocity convertTwist( const geometry_msgs::Twist twist_cmd );

      void notifyTickVelocityListeners(const differential_drive::TickVelocity& tick_velocity);
 
      std::vector<ITickVelocityListener*> _tick_velocity_listeners;

      const BaseModel* _p_base_model;
  };
}

#endif /* GUARD_TwistConverter */
