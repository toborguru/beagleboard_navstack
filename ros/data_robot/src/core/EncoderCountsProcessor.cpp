#include "EncoderCountsProcessor.hpp"

extern "C"
{
  #include "RollOverHelpers.h"
}

namespace data_robot_core
{
/** Default constructor.
 */
EncoderCountsProcessor::EncoderCountsProcessor() 
                : _last_left_count( 0 ),
                  _last_right_count( 0 ),
                  _last_stasis_count( 0 ),
                  _last_millis( 0 )
{
}

/** Function to report new encoder data to this processor.
 *
 */
void EncoderCountsProcessor::AddNewData( int16_t left_count, int16_t right_count, int16_t stasis_count, uint16_t millis )
{
  _encoder_counts.left_count = DifferentiateCounts( _last_left_count, left_count );
  _encoder_counts.right_count = DifferentiateCounts( _last_right_count, right_count );
  _encoder_counts.stasis_count = DifferentiateCounts( _last_stasis_count, stasis_count );
  _encoder_counts.dt_ms = DifferentiateMillis( _last_millis, millis );

  _last_left_count = left_count;
  _last_right_count = right_count;
  _last_stasis_count = stasis_count;
  _last_millis = millis;
}

/** Access Function. 
 *  @returns New counts since previous AddNewData call.
 */
differential_drive::EncoderCounts EncoderCountsProcessor::GetEncoderCounts() const
{
  return _encoder_counts;
}

int16_t EncoderCountsProcessor::DifferentiateCounts( int16_t old_counts, int16_t new_counts ) const
{
  return DifferentiateInt16RollOver( old_counts, new_counts );
}

uint16_t EncoderCountsProcessor::DifferentiateMillis( uint16_t old_millis, uint16_t new_millis ) const
{
  return DifferentiateUint16RollOver( old_millis, new_millis );
}
}
