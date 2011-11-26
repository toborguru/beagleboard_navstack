// ReadEncodersRequest.hpp

#include "diff_drive/EncoderCounts.h"

#include "BusRequest.hpp"

#ifndef GUARD_ReadEncodersRequest
#define GUARD_ReadEncodersRequest

namespace data_robot_core
{
/** This class is a BusRequest that is used to read the encoder counts from 
 *  the robot.
 *
 */
class ReadEncodersRequest : public BusRequest
{
public:
  ReadEncodersRequest( bool is_blockable = true, bool is_lockable = true );

  // Virtual destructor to pass pointer ownership without exposing base class [Meyers, 2005, Item 7]
  virtual ~ReadEncodersRequest() {};

  diff_drive::EncoderCounts GetEncoderCounts();

private:
  int16_t _last_left_count;
  int16_t _last_right_count;
  int16_t _last_stasis_count;
  uint16_t _last_millis;
};
}
 
#endif /* GUARD_ReadEncodersRequest */
