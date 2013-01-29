// TickVelocityPublisherEndpointStub.hpp
 
#ifndef GUARD_TickVelocityPublisherEndpointStub
#define GUARD_TickVelocityPublisherEndpointStub
 
#include "ITickVelocityPublisherEndpoint.hpp"
 
namespace diff_drive_test_message_endpoints_test_doubles
{
  class TickVelocityPublisherEndpointStub : public diff_drive_core::ITickVelocityPublisherEndpoint
  { 
    public:
      TickVelocityPublisherEndpointStub()
        : _count_of_tick_velocities_published(0),
          _linear( 0 ),
          _angular( 0 )
      { }

      mutable int _count_of_tick_velocities_published;
      mutable int _linear;
      mutable int _angular;

      void Publish(const diff_drive::TickVelocity& tick_velocity) const
      {
        _count_of_tick_velocities_published++;

        _linear = tick_velocity.linear_ticks_sec;
        _angular = tick_velocity.angular_ticks_sec;

#if 0
        // Output the read values to the terminal; this isn't the
        // unit test, but merely a helpful means to show what's going on.
        std::cout << "TickVelocity published on TickVelocityPublisherEndpoint with x: " 
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
 
#endif /* GUARD_TickVelocityPublisherEndpointStub */
