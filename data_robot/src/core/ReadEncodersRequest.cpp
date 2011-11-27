// ReadEncodersRequest.hpp

#include "BusAddresses.h"
#include "ReadEncodersRequest.hpp"

extern "C"
{
  #include "RollOverHelpers.h"
}

#define NUM_DATA_BYTES  12 

namespace data_robot_core
{
ReadEncodersRequest::ReadEncodersRequest( bool is_blockable, bool is_lockable )
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

diff_drive::EncoderCounts ReadEncodersRequest::GetEncoderCounts()
{
  diff_drive::EncoderCounts encoder_counts;

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
    new_left_count = *(int16_t*)&p_data_buffer[0];
    new_right_count = *(int16_t*)&p_data_buffer[2];
    new_stasis_count = *(int16_t*)&p_data_buffer[4];
    new_millis = *(int16_t*)&p_data_buffer[6];

    diff_millis = DifferentiateUint16RollOver( _last_millis, new_millis );
  
      encoder_counts.left_count = DifferentiateInt16RollOver( _last_left_count, new_left_count );
      encoder_counts.right_count = DifferentiateInt16RollOver( _last_right_count, new_right_count );
      encoder_counts.stasis_count = DifferentiateInt16RollOver( _last_stasis_count, new_stasis_count );

      encoder_counts.dt_ms = diff_millis;

      encoder_counts.reading_time.sec   = GetTimeStamp().tv_sec;
      encoder_counts.reading_time.nsec  = GetTimeStamp().tv_nsec;

      _last_millis = new_millis;
      _last_left_count = new_left_count;
      _last_right_count = new_right_count;
      _last_stasis_count = new_stasis_count;
    
    if ( diff_millis > 0 )
    {
    }
    else
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

  return encoder_counts;
}
}
