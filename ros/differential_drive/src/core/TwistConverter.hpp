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

      void Attach( ITickVelocityListener& tick_velocity_listener );

      void SetBaseModel( const BaseModel& base_model );

      void OnTwistAvailableEvent( const geometry_msgs::Twist& twist );
 
    private:
      differential_drive::TickVelocity ConvertTwist( const geometry_msgs::Twist twist_cmd );

      void NotifyTickVelocityListeners(const differential_drive::TickVelocity& tick_velocity);
 
      std::vector<ITickVelocityListener*> _tick_velocity_listeners;

      const BaseModel* _p_base_model;
  };
}

#endif /* GUARD_TwistConverter */
