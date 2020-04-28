#ifndef __I2C_REGISTERS_H_
#define __I2C_REGISTERS_H_

#include <inttypes.h>

typedef struct __attribute__((__packed__))
{
  uint8_t     command_group;      // W    byte
  uint8_t     command;            // W    byte
  int16_t     linear_velocity;    // R/W  s7.8  ticks/sec
  int16_t     angular_velocity;   // R/W  s7.8  ticks/sec
  int16_t     left_encoder;       // R    s15.0 ticks
  int16_t     right_encoder;      // R    s15.0 ticks
  int16_t     stasis_encoder;     // R    s15.0 ticks
  uint16_t    encoder_time;       // R    u16.0 timer @ sys clk rate
  uint16_t    system_time;        // R    u16.0 timer @ sys clk rate
  uint16_t    status_flag;        // R    flags
  uint16_t    voltage;            // R    u16.0 ADC value
  uint16_t    current;            // R    u16.0 ADC value
  uint16_t    linear_accel;       // R/W  u16.0 ticks/sec^2
  uint16_t    angular_accel;      // R/W  u16.0 ticks/sec^2
  uint16_t    pid_kp;             // R/W  u8.8
  uint16_t    pid_ki;             // R/W  u8.8
  uint16_t    pid_kd;             // R/W  u8.8
  int16_t     left_setpoint;      // R    s7.8  ticks/update
  int16_t     left_error;         // R    s7.8  ticks/update
  int16_t     left_d_error;       // R    s7.8  ticks/update
  int32_t     left_i_term;        // R    s23.8 (ticks * ki) /update
  int16_t     right_setpoint;     // R    s7.8  ticks/update
  int16_t     right_error;        // R    s7.8  ticks/update
  int16_t     right_d_error;      // R    s7.8  ticks/update
  int32_t     right_i_term;       // R    s23.8 (ticks * ki) /update
} I2C_REGISTERS_t;

extern volatile I2C_REGISTERS_t* gp_commands;
extern volatile I2C_REGISTERS_t* gp_telemetry;

#endif
