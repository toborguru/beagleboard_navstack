/************************************************
 Sawyer Larkin
 D.A.T.A.

 Pid.c
 Pid Controller
************************************************/

#include <inttypes.h>

#include "pid.h"
#include "clamp.h"

/** Computes a PID based correction. 
 */
int16_t Pid_Compute_Correction( int16_t current_value, Pid_State_t *_p_state )
{
    int16_t correction;

    _p_state->error = _p_state->setpoint - current_value;

    // Compute the derivative error
    _p_state->d_error = _p_state->prev_error - _p_state->error;

    // Update the previous error
    _p_state->prev_error = _p_state->error;

    correction = ( (_p_state->kp * _p_state->error) 
                 - (_p_state->kd * _p_state->d_error) 
                 + (_p_state->ki * _p_state->i_error) )
                / _p_state->ko;

    Clamp_Abs_Value( &correction, _p_state->max_correction );
    //if ( CLAMP_VALUE_NOT_CLAMPED == Clamp_Abs_Value( &correction, _p_state->max_correction ) )
    {
        // Compute the integral error
        _p_state->i_error += _p_state->error;

        // Constrain the integral error
        Clamp_Abs_Value( &(_p_state->i_error), _p_state->max_i_error );
    }

    return correction;
}
