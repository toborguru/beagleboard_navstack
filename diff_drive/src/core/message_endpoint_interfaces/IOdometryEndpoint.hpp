// IOdometryEndpoint.hpp
 
#ifndef GUARD_IOdometryEndpoint
#define GUARD_IOdometryEndpoint
 
#include "nav_msgs/Odometry.h"
 
namespace diff_drive_core
{
  class IOdometryEndpoint
  {
    public:
      // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
      virtual ~IOdometryEndpoint() {}
      virtual void publish(const nav_msgs::Odometry& odometry) const = 0;
  };
}
 
#endif /* GUARD_IOdometryEndpoint */
