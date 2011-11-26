// OdometryIntegrator.hpp
 
#ifndef GUARD_OdometryIntegrator
#define GUARD_OdometryIntegrator
 
#include <vector>

#include "nav_msgs/Odometry.h"
#include "diff_drive/EncoderCounts.h"

#include "BaseModel.hpp"
#include "IOdometryListener.hpp"
#include "IEncoderCountsListener.hpp"

namespace diff_drive_core
{
  class OdometryIntegrator : public IEncoderCountsListener
  {
    public:
      OdometryIntegrator();

      void Attach( IOdometryListener& odometry_listener );

      void SetBaseModel( const BaseModel& base_model );

      void OnEncoderCountsAvailableEvent( const diff_drive::EncoderCounts& encoder_counts );
 
    private:
      nav_msgs::Odometry AddNewCounts( const diff_drive::EncoderCounts counts,
                                         const nav_msgs::Odometry last_position );

      void NotifyOdometryListeners(const nav_msgs::Odometry& odometry);
 
      std::vector<IOdometryListener*> _odometry_listeners;
  
      nav_msgs::Odometry _current_position;

      const BaseModel* _p_base_model;
  };
}

#endif /* GUARD_OdometryIntegrator */
