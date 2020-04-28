
#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include "../util/pid.h"

#define MOTION_CONTROL_UPDATE_RATE_HZ   20


// PID Parameters per second
#define MOTION_CONTROL_DEFAULT_KP   80.0
#define MOTION_CONTROL_DEFAULT_KD   0.0
#define MOTION_CONTROL_DEFAULT_KI   60.0 
#define MOTION_CONTROL_MAX_CORRECTION   0xFF

#define MOTION_CONTROL_DEFAULT_MAX_VELOCITY     128 // ticks/sec
#define MOTION_CONTROL_DEFAULT_LINEAR_ACCEL     48  // u16.0 ticks/sec^2
#define MOTION_CONTROL_DEFAULT_ANGULAR_ACCEL    48  // u16.0 ticks/sec^3

#define STARTING_POWER 16

typedef struct
{
    volatile int16_t  angular_velocity_setpoint;  // s7.8 ticks/update
    volatile int16_t  angular_velocity;           // s7.8 ticks/update
    volatile int16_t  linear_velocity_setpoint;   // s7.8 ticks/update
    volatile int16_t  linear_velocity;            // s7.8 ticks/update
    volatile int32_t  encoder_setpoint;           // s23.8 ticks
    volatile int32_t  encoder;                    // s31.0 ticks
    volatile int16_t  power_out;                  // -pid.max_correction <= power_out <= pid.max_correction
    Pid_State_t pid;
} Motion_State_t;

// EXPORTED GLOBALS
extern volatile Motion_State_t g_left_wheel_motion;
extern volatile Motion_State_t g_right_wheel_motion;

extern volatile int32_t g_encoder_total;
extern volatile int32_t g_stasis_total;

extern volatile uint8_t g_estop;

void    Motion_Control_Init(    void );

void    Motion_Control_Run_Step(    void );

void    Motion_Control_Set_Velocity(    int16_t linear_velocity, int16_t angular_velocity ); // ticks/sec

void    Motion_Control_EStop(   void );    // Apply Brakes

void    Motion_Control_Stop(    void );     // Decelerate

void    Motion_Control_Set_Acceleration(    uint16_t linear_acceleration,       // u8.8 ticks/sec 
                                            uint16_t angular_acceleration );    // u8.8 ticks/sec

void    Motion_Control_Set_PID( Motion_State_t *p_state,
                                uint16_t k_p,   // u8.8
                                uint16_t k_i,   // u8.8
                                uint16_t k_d ); // u8.8

#endif
