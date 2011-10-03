// OdometryIntegrater.hpp
 
#ifndef GUARD_OdometryIntegrater
#define GUARD_OdometryIntegrater
 
#include <vector>

#include "nav_msgs/Odometry.h"
#include "diff_drive/EncoderCounts.h"

#include "BaseModel.hpp"
#include "IOdometryListener.hpp"
#include "IEncoderCountsListener.hpp"

namespace diff_drive_core
{
  class OdometryIntegrater : public IEncoderCountsListener
  {
    public:
      OdometryIntegrater();

      void Attach( IOdometryListener& odometry_listener );

      void SetBaseModel( BaseModel& base_model );

      void OnEncoderCountsAvailableEvent( const diff_drive::EncoderCounts& encoder_counts );
 
    private:
      nav_msgs::Odometry AddNewCounts( const diff_drive::EncoderCounts counts,
                                         const nav_msgs::Odometry last_position );

      void NotifyOdometryListeners(const nav_msgs::Odometry& odometry);
 
      std::vector<IOdometryListener*> _odometry_listeners;
  
      nav_msgs::Odometry _current_position;

      BaseModel* _p_base_model;
  };
}

#endif /* GUARD_OdometryIntegrater */
