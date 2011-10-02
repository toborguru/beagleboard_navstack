// EncoderCountsReader.hpp
 
#ifndef GUARD_EncoderCountsReader
#define GUARD_EncoderCountsReader
 
#include <vector>

#include "nav_msgs/Odometry.h"
#include "diff_drive/EncoderCounts.h"

#include "BaseModel.hpp"
#include "IOdometryListener.hpp"
#include "IEncoderCountsListener.hpp"

namespace diff_drive_core
{
  class EncoderCountsReader : public IEncoderCountsListener
  {
    public:
      EncoderCountsReader();

      void Attach( IOdometryListener& odometry_listener );

      void SetBaseModel( BaseModel& base_model );

      void OnEncoderCountsAvailableEvent( const diff_drive::EncoderCounts& encoder_counts );
 
    private:
      nav_msgs::Odometry CountsReceived( const diff_drive::EncoderCounts counts,
                                         const nav_msgs::Odometry last_position );

      void NotifyOdometryListeners(const nav_msgs::Odometry& odometry);
 
      std::vector<IOdometryListener*> _odometry_listeners;
  
      nav_msgs::Odometry _current_position;

      BaseModel* _p_base_model;
  };
}

#endif /* GUARD_EncoderCountsReader */
