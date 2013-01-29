// OdometryPublisherEndpointStub.hpp
 
#ifndef GUARD_OdometryPublisherEndpointStub
#define GUARD_OdometryPublisherEndpointStub
 
#include "tf/transform_datatypes.h"

#include "nav_msgs/Odometry.h"
#include "IOdometryPublisherEndpoint.hpp"
 
namespace differential_drive_test_message_endpoints_test_doubles
{
  class OdometryPublisherEndpointStub : public differential_drive_core::IOdometryPublisherEndpoint
  { 
    public:
      OdometryPublisherEndpointStub()
        : _count_of_messages_published(0),
          _x(0.0),
          _y(0.0),
          _theta(0.0),
          _linear(0.0),
          _angular(0.0),
          _covariance(0.0)
      { }

      mutable int _count_of_messages_published;
      mutable double _x;
      mutable double _y;
      mutable double _theta;
      mutable double _linear;
      mutable double _angular;
      mutable double _covariance;

      void Publish(const nav_msgs::Odometry& odometry)
      {
        _count_of_messages_published++;

        _x = odometry.pose.pose.position.x;
        _y = odometry.pose.pose.position.y;
        _theta = tf::getYaw(odometry.pose.pose.orientation);
        _covariance = odometry.pose.covariance[0];
        _linear = odometry.twist.twist.linear.x;
        _angular = odometry.twist.twist.angular.z;

#if 0
        // Output the read values to the terminal; this isn't the
        // unit test, but merely a helpful means to show what's going on.
        std::cout << "Odometry published on OdometryPublisherEndpoint with x: " 
                  << _x
                  << ", y: "
                  << _y
                  << ", theta: "
                  << _theta
                  << std::endl;
#endif
      }
  };
}
 
#endif /* GUARD_OdometryPublisherEndpointStub */
