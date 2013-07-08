/************************************************
 Sawyer Larkin
 D.A.T.A.

 Pid.c
 Pid Controller
************************************************/

#include <inttypes.h>

#include "pid.h"
#include "clamp.h"

/** Computes a PID based output. 
 */
int16_t Pid_Compute_Output( int16_t input, Pid_State_t *_p_state )
{
    int16_t output;

    _p_state->error = _p_state->setpoint - input;

    // Compute the error derivative (using "Derivative on Measurement")
    _p_state->d_error = input - _p_state->prev_input;

    // Compute error sum
    _p_state->i_term += _p_state->ki * (float)_p_state->error;
    if ( _p_state->i_term > _p_state->max_output) _p_state->i_term = _p_state->max_output;
    if ( _p_state->i_term > _p_state->min_output) _p_state->i_term = _p_state->min_output;

    // Update the previous error
    _p_state->prev_input = input;

    // Compute the output
    output = (int16_t)(   (_p_state->kp * (float)_p_state->error)
                        - (_p_state->kd * (float)_p_state->d_error)
                        + _p_state->i_term );

    return output;
}
