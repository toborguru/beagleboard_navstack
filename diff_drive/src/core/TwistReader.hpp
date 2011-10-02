// TwistReader.hpp
 
#ifndef GUARD_TwistReader
#define GUARD_TwistReader
 
#include <vector>

#include "geometry_msgs/Twist.h"
#include "diff_drive/TickVelocity.h"

#include "BaseModel.hpp"
#include "ITickVelocityListener.hpp"
#include "ITwistListener.hpp"

namespace diff_drive_core
{
  class TwistReader : public ITwistListener
  {
    public:
      TwistReader();

      void Attach( ITickVelocityListener& tick_velocity_listener );

      void SetBaseModel( BaseModel& base_model );

      void OnTwistAvailableEvent( const geometry_msgs::Twist& twist );
 
    private:
      diff_drive::TickVelocity TwistReceived( const geometry_msgs::Twist twist_cmd );

      void NotifyTickVelocityListeners(const diff_drive::TickVelocity& tick_velocity);
 
      std::vector<ITickVelocityListener*> _tick_velocity_listeners;

      BaseModel* _p_base_model;
  };
}

#endif /* GUARD_TwistReader */
