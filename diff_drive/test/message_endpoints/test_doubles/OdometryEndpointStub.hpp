// OdometryEndpointStub.hpp
 
#ifndef GUARD_OdometryEndpointStub
#define GUARD_OdometryEndpointStub
 
#include "nav_msgs/Odometry.h"
#include "IOdometryEndpoint.hpp"
 
namespace diff_drive_test_message_endpoints_test_doubles
{
  class OdometryEndpointStub : public diff_drive_core::IOdometryEndpoint
  { 
    public:
      OdometryEndpointStub()
        : countOfOdometrysPublished(0) { }
 
      void publish(const nav_msgs::Odometry& odometry) const {
        countOfOdometrysPublished++;
 
        // Output the laser scan seq number to the terminal; this isn't the
        // unit test, but merely a helpful means to show what's going on.
        std::cout << "Odometry endpoint stub sent to OdometryReceiver with a seq of: " << odometry.header.seq << std::endl;
      };
 
      // Extend IOdometryEndpoint for unit testing needs.
      // May be modified by const functions but maintains logical constness [Meyers, 2005, Item 3].
      mutable int countOfOdometrysPublished;
  };
}
 
#endif /* GUARD_OdometryEndpointStub */
