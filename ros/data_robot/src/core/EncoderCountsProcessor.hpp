// EncoderCountsProcessor.hpp
 
#ifndef GUARD_EncoderCountsProcessor
#define GUARD_EncoderCountsProcessor

#include "diff_drive/EncoderCounts.h"

namespace data_robot_core
{ 
class EncoderCountsProcessor
{
public:

  EncoderCountsProcessor();

  void      AddNewData( int16_t left_count, int16_t right_count, int16_t stasis_count, uint16_t millis );

  diff_drive::EncoderCounts GetEncoderCounts() const;

private:
  int16_t   DifferentiateCounts( int16_t old_counts, int16_t new_counts ) const;
  uint16_t  DifferentiateMillis( uint16_t old_millis, uint16_t new_millis ) const;

  diff_drive::EncoderCounts _encoder_counts;

  int16_t _last_left_count;
  int16_t _last_right_count;
  int16_t _last_stasis_count;
  uint16_t _last_millis;
};
}
 
#endif /* GUARD_EncoderCountsProcessor */
