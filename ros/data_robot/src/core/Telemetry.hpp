/** This file contains the structure definitions for the telemetry groups.
 */

#include <inttypes.h>

#ifndef GUARD_Telemetry
#define GUARD_Telemetry

namespace data_robot_core
{
typedef struct __attribute__((__packed__))
{
  uint16_t    command;            // W
  int16_t     linear_velocity;    // R/W
  int16_t     angular_velocity;   // R/W
  int16_t     left_encoder;       // R
  int16_t     right_encoder;      // R
  int16_t     stasis_encoder;     // R
  uint16_t    encoder_time;       // R
  uint16_t    system_time;        // R
  uint16_t    status_flag;        // R
  uint16_t    voltage;            // R
  uint16_t    current;            // R
  uint16_t    linear_accel;       // R/W
  uint16_t    angular_accel;      // R/W
  uint32_t    seconds;
  uint32_t    nano_seconds;
  // PID values
} BaseTelemetry_T;

typedef struct
{
  uint8_t   bumpers;
  uint32_t    seconds;
  uint32_t    nano_seconds;
} FrontShellTelemetry_T;
}

#endif
