/************************************************
 Sawyer Larkin
 D.A.T.A.

 Pid.h
 Pid Control
************************************************/

#include <inttypes.h>

#ifndef __PID_H
#define __PID_H

typedef struct
{   
    // PID State
    int16_t setpoint;
    int16_t error;
    int16_t prev_error;
    int16_t d_error;
    int16_t i_error;

    // Tuning Parameters
    int8_t  kp;
    int8_t  kd;
    int8_t  ki;
    int8_t  ko; // Output Scale: Used for fractional gains => Kp=kp/ko, Kd=kd/ko, Ki=ki/ko 
    int16_t max_i_error;
    int16_t max_correction;
} Pid_State_t;

int16_t Pid_Compute_Correction(int16_t current_value, Pid_State_t *_p_state);

#endif
