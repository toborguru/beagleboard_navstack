// Log.hpp

#ifndef GUARD_Log
#define GUARD_Log

namespace data_robot_core
{
enum LogLevel_T
{ 
  DEBUG = 0,
  INFO,
  WARNING,
  ERROR,
  FATAL
};

enum LogFlags_T
{ 
  NONE          = 0,
  ONCE          = 0x1,
  THROTTLE_1S   = 0x2,
  THROTTLE_15S  = 0x4,
  THROTTLE_60S  = 0x8
};
}
 
#endif /* GUARD_Log */
