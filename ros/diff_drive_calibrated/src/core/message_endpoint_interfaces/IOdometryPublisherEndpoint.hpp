// IOdometryPublisherEndpoint.hpp
 
#ifndef GUARD_IOdometryPublisherEndpoint
#define GUARD_IOdometryPublisherEndpoint
 
#include "nav_msgs/Odometry.h"

#include "IOdometryListener.hpp"
 
namespace diff_drive_core
{
class IOdometryPublisherEndpoint : public IOdometryListener
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IOdometryPublisherEndpoint() {}
  virtual void publish(const nav_msgs::Odometry& odometry) = 0;
};
}
 
#endif /* GUARD_IOdometryPublisherEndpoint */
