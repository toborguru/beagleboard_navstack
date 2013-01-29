// IOdometryPublisherEndpoint.hpp
 
#ifndef GUARD_IOdometryPublisherEndpoint
#define GUARD_IOdometryPublisherEndpoint
 
#include "nav_msgs/Odometry.h"
 
namespace differential_drive_core
{
class IOdometryPublisherEndpoint
{
public:
  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~IOdometryPublisherEndpoint() {}
  virtual void Publish(const nav_msgs::Odometry& odometry) = 0;
};
}
 
#endif /* GUARD_IOdometryPublisherEndpoint */