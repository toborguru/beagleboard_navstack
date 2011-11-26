// MotorVelocityRequest.cpp

#include "BusAddresses.hpp"
#include "MotorVelocityRequest.hpp"

extern "C"
{
  #include "RollOverHelpers.h"
}

namespace data_robot_core
{
MotorVelocityRequest::MotorVelocityRequest( bool is_blockable, bool is_lockable )
                    : BusRequest( is_blockable, is_lockable )
{
  uint16_t address;

  address = BB_TRAINER + ( TICK_VELOCITY_OFFSET << 8 );

  SetAddress( (uint8_t*)&address, ADDRESS_SIZE );
  SetRequestType( REQUEST_WRITE );
}

void MotorVelocityRequest::SetVelocity( int16_t linear, int16_t angular )
{
  uint16_t velocity[2];

  velocity[0] = linear;
  velocity[1] = angular;

  SetDataBufferSize( sizeof(velocity) );
  SetData( (uint8_t*)&velocity, sizeof(velocity) );
}
}
