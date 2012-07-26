// ReadBaseTelemetryRequest.cpp

#include <ros/ros.h>

#include "BusAddresses.hpp"
#include "ReadBaseTelemetryRequest.hpp"

extern "C"
{
  #include "RollOverHelpers.h"
}

#define NUM_DATA_BYTES  8

namespace data_robot_core
{
ReadBaseTelemetryRequest::ReadBaseTelemetryRequest( bool is_blockable, bool is_lockable )
                    : BusRequest( is_blockable, is_lockable )
{
  uint16_t address;

  address = BB_TRAINER + ( ENCODER_COUNTS_OFFSET << 8 );

  SetAddress( (uint8_t*)&address, ADDRESS_SIZE );
  SetRequestType( REQUEST_READ );
  SetDataBufferSize( NUM_DATA_BYTES );
}

BaseTelemetry_T ReadBaseTelemetryRequest::GetTelemetry()
{
  BaseTelemetry_T telemetry;

  bool set_lock = false;

  uint8_t *p_data_buffer = BusRequest::GetDataBuffer();

  if ( IsLockable() && ! IsLocked() )
  {
    set_lock = true;
    Lock();
  }

  if ( NUM_DATA_BYTES >= GetDataBufferSize() )
  {
    memcpy( &telemetry.left_encoder, p_data_buffer, 8 );

    telemetry.seconds = GetTimeStamp().tv_sec;
    telemetry.nano_seconds  = GetTimeStamp().tv_nsec;
  }
  else
  {
    ROS_ERROR( "Data Buffer Size of: %d, is not large enough to hold: %d bytes!\n", GetDataBufferSize(), NUM_DATA_BYTES );
  }

  if ( set_lock )
  {
    Unlock();
  }

  return telemetry;
}
}
