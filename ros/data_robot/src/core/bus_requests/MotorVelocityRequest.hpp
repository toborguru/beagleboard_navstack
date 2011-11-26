// MotorVelocityRequest.hpp

#include "diff_drive/TickVelocity.h"

#include "BusRequest.hpp"

#ifndef GUARD_MotorVelocityRequest
#define GUARD_MotorVelocityRequest

namespace data_robot_core
{
/** This class is a BusRequest that is used to set the desired speed for the robot.
 *
 */
class MotorVelocityRequest : public BusRequest
{
public:
  MotorVelocityRequest( bool is_blockable = true, bool is_lockable = true );

  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~MotorVelocityRequest() {};

  void  SetVelocity( int16_t linear, int16_t angular );

private:
};
}
 
#endif /* GUARD_MotorVelocityRequest */
