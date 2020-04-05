// TwistConverter.hpp
 
#ifndef GUARD_TwistConverter
#define GUARD_TwistConverter
 
#include <vector>

#include "geometry_msgs/Twist.h"
#include "diff_drive_calibrated/TickVelocity.h"

#include "BaseModel.hpp"
#include "ITickVelocityListener.hpp"
#include "ITwistListener.hpp"

namespace diff_drive_core
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
      diff_drive_calibrated::TickVelocity convertTwist( const geometry_msgs::Twist twist_cmd );

      void notifyTickVelocityListeners(const diff_drive_calibrated::TickVelocity& tick_velocity);
 
      std::vector<ITickVelocityListener*> _tick_velocity_listeners;

      const BaseModel* _p_base_model;
  };
}

#endif /* GUARD_TwistConverter */
