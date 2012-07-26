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
                    : BusRequest( is_blockable, is_lockable ),
                      _last_left_count(0),
                      _last_right_count(0),
                      _last_stasis_count(0),
                      _last_millis(0)
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

  int16_t new_left_count;
  int16_t new_right_count;
  int16_t new_stasis_count;
  uint16_t new_millis;

  uint16_t diff_millis;

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

    new_left_count = telemetry.left_encoder;
    new_right_count = telemetry.right_encoder;
    new_stasis_count = telemetry.stasis_encoder;
    new_millis = telemetry.encoder_time;

    diff_millis = DifferentiateUint16RollOver( _last_millis, new_millis );
  
    telemetry.left_encoder = DifferentiateInt16RollOver( _last_left_count, new_left_count );
    telemetry.right_encoder = DifferentiateInt16RollOver( _last_right_count, new_right_count );
    telemetry.stasis_encoder = DifferentiateInt16RollOver( _last_stasis_count, new_stasis_count );

    telemetry.encoder_time = diff_millis;

    telemetry.seconds = GetTimeStamp().tv_sec;
    telemetry.nano_seconds  = GetTimeStamp().tv_nsec;

    _last_millis = new_millis;
    _last_left_count = new_left_count;
    _last_right_count = new_right_count;
    _last_stasis_count = new_stasis_count;

    if ( diff_millis == 0 )
    {
      ROS_WARN(  "No change in encoder measurement time." );
    }
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
