// ReadFrontTelemetryRequest.hpp

#include <ros/ros.h>

#include "BusAddresses.hpp"
#include "ReadFrontTelemetryRequest.hpp"

namespace data_robot_core
{
ReadFrontTelemetryRequest::ReadFrontTelemetryRequest( bool is_blockable, bool is_lockable )
                    : BusRequest( is_blockable, is_lockable )
{
  uint16_t address;

  address = FRONT_SHELL + ( FRONT_TELEMETRY_OFFSET << 8 );

  SetAddress( (uint8_t*)&address, ADDRESS_SIZE );
  SetRequestType( REQUEST_READ );
  SetDataBufferSize( sizeof(FrontShellTelemetry_T) );
}

FrontShellTelemetry_T ReadFrontTelemetryRequest::GetTelemetry()
{ 
  FrontShellTelemetry_T telemetry; 

  bool set_lock = false;

  uint8_t *p_data_buffer = BusRequest::GetDataBuffer();

  if ( IsLockable() && ! IsLocked() )
  {
    set_lock = true;
    Lock();
  }

  if ( sizeof(telemetry) <= GetDataBufferSize() )
  {
    memcpy( &telemetry, p_data_buffer, sizeof(telemetry) );
  }
  else
  {
    ROS_ERROR( "Data Buffer Size of: %u, is not large enough to hold: %lu bytes!\n", GetDataBufferSize(), sizeof(telemetry) );
  }

  if ( set_lock )
  {
    Unlock();
  }

  return telemetry;
}
}
