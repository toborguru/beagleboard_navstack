// EncoderCountsReader.hpp
 
#ifndef GUARD_EncoderCountsReader
#define GUARD_EncoderCountsReader
 
#include <pthread.h>
#include <vector>
#include "nav_msgs/Odometry.h"
#include "diff_drive/EncoderCounts.h"
#include "IOdometryListener.hpp"
#include "IEncoderCountsEndpoint.hpp"

namespace diff_drive_core
{
  class EncoderCountsReader : public IEncoderCountsListener
  {
    public:
      EncoderCountsReader();

      ~EncoderCountsReader();
 
      void Attach( IOdometryListener& odometryListener );

      void Connect( IEncoderCountsEndpoint& encoderEndpoint );
      void Disconnect();

      void OnEncoderCountsAvailableEvent( const diff_drive::EncoderCounts& encoderCounts );
 
    private:
      nav_msgs::Odometry CountsReceived( const diff_drive::EncoderCounts counts,
                                         const nav_msgs::Odometry lastPosition );

      void NotifyOdometryListeners(const nav_msgs::Odometry& odometry);
 
      std::vector<IOdometryListener*> _odometryListeners;
  
      IEncoderCountsEndpoint*  _p_encoderCountsEndpoint;      
      nav_msgs::Odometry      _currentPosition;
  };
}

#endif /* GUARD_EncoderCountsReader */
