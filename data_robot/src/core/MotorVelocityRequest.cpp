// MotorVelocityRequest.hpp

#include "BusAddresses.h"
#include "MotorVelocityRequest.hpp"

extern "C"
{
  #include "RollOverHelpers.h"
}

#define NUM_DATA_BYTES  4

namespace data_robot_core
{
MotorVelocityRequest::MotorVelocityRequest( bool is_blockable, bool is_lockable )
                    : BusRequest( is_blockable, is_lockable )
{
  uint16_t address = TICK_VELOCITY_ADDRESS;

  SetAddress( (uint8_t*)&address, ADDRESS_SIZE );
  SetRequestType( REQUEST_WRITE );
  SetDataBufferSize( NUM_DATA_BYTES );
}

void MotorVelocityRequest::SetVelocity( int16_t linear, int16_t angular )
{
  uint16_t velocity[2];

  velocity[0] = linear;
  velocity[1] = angular;

  SetData( (uint8_t*)&velocity, NUM_DATA_BYTES );
}

}
