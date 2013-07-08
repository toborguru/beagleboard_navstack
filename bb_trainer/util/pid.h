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
    int16_t prev_input;
    int16_t d_error;
    float i_term;

    // Tuning Parameters
    float kp;
    float kd;
    float ki;
    int16_t max_output;
    int16_t min_output;
} Pid_State_t;

int16_t Pid_Compute_Correction(int16_t current_value, Pid_State_t *_p_state);

#endif
