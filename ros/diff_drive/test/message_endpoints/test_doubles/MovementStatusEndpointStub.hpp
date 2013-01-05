// MovementStatusEndpointStub.hpp
 
#ifndef GUARD_MovementStatusEndpointStub
#define GUARD_MovementStatusEndpointStub
 
#include "tf/transform_datatypes.h"

#include "diff_drive/MovementStatus.h"
#include "IMovementStatusEndpoint.hpp"
 
namespace diff_drive_test_message_endpoints_test_doubles
{
  class MovementStatusEndpointStub : public diff_drive_core::IMovementStatusEndpoint
  { 
    public:
      MovementStatusEndpointStub()
        : _count_of_messages_published(0),
          _linear(0.0),
          _linear_average(0.0),
          _stasis(0.0),
          _stasis_average(0.0),
          _state(0),
          _stasis_enabled(false)
      { }

      mutable int _count_of_messages_published;
      mutable double _linear;
      mutable double _linear_average;
      mutable double _stasis;
      mutable double _stasis_average;
      mutable int _state;
      mutable bool _stasis_enabled;

      void Publish(const diff_drive::MovementStatus& movement_status)
      {
        _count_of_messages_published++;

        _state = movement_status.motors_state;
        _linear = movement_status.linear_velocity;
        _linear_average = movement_status.linear_velocity_average;
        _stasis = movement_status.stasis_velocity;
        _stasis_average = movement_status.stasis_velocity_average;
        _stasis_enabled = movement_status.stasis_wheel_enabled;

#if 0
        // Output the read values to the terminal; this isn't the
        // unit test, but merely a helpful means to show what's going on.
        std::cout << "MovementStatus published on MovementStatusEndpoint with state: " 
                  << _state
                  << ", linear: "
                  << _linear
                  << ", average: "
                  << _linear_average
                  << ", stasis: "
                  << _stasis
                  << ", average: "
                  << _stasis_average
                  << ", stasis enabled: "
                  << _stasis_enabled
                  << std::endl;
#endif
      }
  };
}
 
#endif /* GUARD_MovementStatusEndpointStub */
