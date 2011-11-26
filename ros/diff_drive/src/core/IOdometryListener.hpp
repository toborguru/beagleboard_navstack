// IOdometryListener.hpp
 
#ifndef GUARD_IOdometryListener
#define GUARD_IOdometryListener
 
//#include "sensor_msgs/Odometry.h"
#include <nav_msgs/Odometry.h>
 
namespace diff_drive_core
{
  class IOdometryListener
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IOdometryListener() {}
      virtual void OnOdometryAvailableEvent( const nav_msgs::Odometry& odometry ) = 0;
  };
}
 
#endif /* GUARD_IOdometryListener */
