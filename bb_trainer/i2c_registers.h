#ifndef __I2C_REGISTERS_H_
#define __I2C_REGISTERS_H_

#include <inttypes.h>

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
  // PID values
} I2C_REGISTERS_t;

extern volatile I2C_REGISTERS_t* gp_commands_read;
extern volatile I2C_REGISTERS_t* gp_commands_write;

extern volatile I2C_REGISTERS_t* gp_telemetry_read;
extern volatile I2C_REGISTERS_t* gp_telemetry_write;

#endif
