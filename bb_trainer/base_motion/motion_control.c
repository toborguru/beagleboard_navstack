#include <stdlib.h>


#include "../util/pid.h"
#include "../util/clamp.h"
#include "../Avr.h"
#include "../i2c_registers.h"
#include "../system_clock.h"

#include "motors.h"
#include "motion_control.h"
#include "shaft_encoders.h"

#define MOTION_CONTROL_KP   100
#define MOTION_CONTROL_KD   0
#define MOTION_CONTROL_KI   25
#define MOTION_CONTROL_KO   5 // Output Scale: Used for fractional gains => Kp=kp/ko, Kd=kd/ko, Ki=ki/ko
#define MOTION_CONTROL_MAX_CORRECTION   0xFF
#define MOTION_CONTROL_MAX_I_ERROR      0x30

#define MOTION_CONTROL_DEFAULT_MAX_VELOCITY     128 // ticks/sec
#define MOTION_CONTROL_DEFAULT_LINEAR_ACCEL     10  // u16.0 ticks/sec (should be but isn't)
#define MOTION_CONTROL_DEFAULT_ANGULAR_ACCEL    10  // u16.0 ticks/sec (should be but isn't)

/* GLOBAL VARIABLES */
volatile Motion_State_t g_left_wheel_motion;
volatile Motion_State_t g_right_wheel_motion;

volatile int32_t g_encoder_total = 0;
volatile int32_t g_stasis_total = 0;

volatile uint8_t g_estop = 0;

/* MODULE VARIABLES */
volatile uint16_t   m_linear_acceleration;      // u8.8 ticks/update^2
volatile uint16_t   m_angular_acceleration;     // u8.8 ticks/update^2
volatile uint16_t   m_max_velocity;             // u16.0 ticks/sec

/* MODULE FUNCTION PROTOTYPES */

void    Motion_Control_Init_State(  volatile Motion_State_t *p_state );

void    Motion_Control_Init_Acceleration(   void );

// Returns power_out- runs Add To Encoder, Compute Position then Run PID
int16_t Motion_Control_Compute_Power(   int8_t new_encoder_ticks,   // s7.0 ticks
                                        volatile Motion_State_t *p_state );      

// Adds new encoder counts
void Motion_Control_Add_To_Encoder( int8_t new_encoder_ticks,   // s7.0 ticks 
                                    volatile Motion_State_t *p_state );     

// Computes the current target position
void    Motion_Control_Compute_Target_Position( volatile Motion_State_t *p_motion );

// Returns the correction from the PID controller
int16_t Motion_Control_Run_PID( int8_t new_encoder_ticks,           // s7.0 ticks
                                volatile Motion_State_t *p_state );

// Adds new fixed point counts to the target position
void    Motion_Control_Add_To_Position(     int16_t num_ticks,          // s7.8 ticks
                                            volatile Motion_State_t *p_motion);

void    Motion_Control_Set_Update_Velocity( int16_t linear_velocity,    // s7.8 ticks/update
                                            int16_t angular_velocity,   // s7.8 ticks/update
                                            volatile Motion_State_t *p_state );

/* FUNCTIONS */

void Motion_Control_Init(   void )
{
    Motion_Control_Init_Acceleration();
    Motion_Control_Init_State( &g_left_wheel_motion );
    Motion_Control_Init_State( &g_right_wheel_motion );
    Motion_Control_Set_Velocity( 0, 0 );

    m_max_velocity = MOTION_CONTROL_DEFAULT_MAX_VELOCITY;
    
    Motors_Set_Direction(MOTORS_L_INDEX, MOTORS_FORWARD);
    Motors_Set_Direction(MOTORS_R_INDEX, MOTORS_FORWARD);
}

/** Sets the default accelerations per #defines.
 */
void    Motion_Control_Init_Acceleration( void )
{
    Motion_Control_Set_Acceleration(    MOTION_CONTROL_DEFAULT_LINEAR_ACCEL, 
                                        MOTION_CONTROL_DEFAULT_ANGULAR_ACCEL );
}

/** Sets the linear and angular accelerations for the module, automatically adjusting
 *  for the step period.
 *  @param  linear_acceleration     In ticks/sec^2
 *  @param  angular_acceleration    In ticks/sec^2
 *
 */
void    Motion_Control_Set_Acceleration(    uint16_t linear_acceleration, 
                                            uint16_t angular_acceleration )
{
    // s15.0 -> s7.8 conversion also scaling for update rate
    m_linear_acceleration = ( linear_acceleration * 256 ) / MOTION_CONTROL_UPDATE_RATE_HZ;
    
    // s15.0 -> s7.8 conversion also scaling for update rate 
    m_angular_acceleration = ( angular_acceleration * 256 ) / MOTION_CONTROL_UPDATE_RATE_HZ;
}

/** This function initializes a Motion_State_t structure, using #defined defaults
 *  for the PID tuning parameters.
 */
void Motion_Control_Init_State( volatile Motion_State_t *p_state )
{
    // Reset motion state
    p_state->angular_velocity = 0;
    p_state->angular_velocity_setpoint = 0;

    p_state->linear_velocity = 0;
    p_state->linear_velocity_setpoint = 0;

    p_state->encoder_setpoint = 0;
    p_state->encoder = 0;

    // PID defaults
    p_state->pid.kp = MOTION_CONTROL_KP;
    p_state->pid.kd = MOTION_CONTROL_KD;
    p_state->pid.ki = MOTION_CONTROL_KI;
    p_state->pid.ko = MOTION_CONTROL_KO;
    p_state->pid.max_correction = MOTION_CONTROL_MAX_CORRECTION;
    p_state->pid.max_i_error = MOTION_CONTROL_MAX_I_ERROR;

    // Reset PID state
    p_state->pid.setpoint = 0;
    p_state->pid.error = 0;
    p_state->pid.d_error = 0;
    p_state->pid.i_error = 0;
    p_state->pid.prev_error = 0;
}

/** This function runs a complete motion step. This includes reading the encoder 
 *  values and commanding the motors.
 */
void Motion_Control_Run_Step(   void )
{
    /*  TODO Verify: I set pointers to the motor control structures because the 
     *  GNU compiler actually produces better code through pointers than direct 
     *  references to structure members (e.g. 'p->Encoder =' produces better 
     *  code than 'Left.Encoder =').
     */
    volatile Motion_State_t* p_l_wheel = &g_left_wheel_motion;
    volatile Motion_State_t* p_r_wheel = &g_right_wheel_motion;

    static int16_t stasis_total = 0;

    int16_t delta_left;
    int16_t delta_right;
    
    int16_t left_power;
    int16_t right_power;

    uint16_t measurement_time;

    // Get new encoder counts
    DISABLE_INTERRUPTS();
    delta_left = g_shaft_encoders_left_count;
    delta_right = g_shaft_encoders_right_count;
    stasis_total += g_shaft_encoders_stasis_count;

    g_shaft_encoders_left_count = 0;
    g_shaft_encoders_right_count = 0;
    g_shaft_encoders_stasis_count = 0;

    measurement_time = (uint16_t)g_system_clock;
    ENABLE_INTERRUPTS();

    // Set motor power
    left_power =    Motion_Control_Compute_Power(   delta_left,   p_l_wheel );
    right_power =   Motion_Control_Compute_Power(   delta_right,  p_r_wheel );

    if (!g_estop)
    {
        if (left_power < 0)
        {
            left_power *= -1;
            Motors_Set_Direction(   MOTORS_L_INDEX, MOTORS_REVERSE );
        }
        else
        {
            Motors_Set_Direction(   MOTORS_L_INDEX, MOTORS_FORWARD);
        }
     
        if (right_power < 0)
        {
            right_power *= -1;
            Motors_Set_Direction(   MOTORS_R_INDEX, MOTORS_REVERSE);
        }
        else
        {
            Motors_Set_Direction(   MOTORS_R_INDEX, MOTORS_FORWARD);
        }

        Motors_Set_Power(   MOTORS_L_INDEX, left_power & 0xFF);
        Motors_Set_Power(   MOTORS_R_INDEX, right_power & 0xFF);
    }

    // Update telemetry registers
    DISABLE_INTERRUPTS();

    gp_telemetry_write->left_encoder = (int16_t)(p_l_wheel->encoder);
    gp_telemetry_write->right_encoder = (int16_t)(p_r_wheel->encoder);
    gp_telemetry_write->stasis_encoder = stasis_total;
    gp_telemetry_write->encoder_time = measurement_time;

    ENABLE_INTERRUPTS();
} 

/** Set left and right motion structures up for the new velocities.
 *  @param  linear_velocity     New base linear velocity in ticks/sec
 *  @param  angular_velocity    New base angular velocity in ticks/sec
 */
void Motion_Control_Set_Velocity(   int16_t linear_velocity, int16_t angular_velocity )
{
    int32_t new_linear_velocity, new_angular_velocity, total_velocity;

    // s15.0 -> s7.8 conversion also scaling for update rate
    new_linear_velocity =   ( (int32_t)linear_velocity * 256 ) /    MOTION_CONTROL_UPDATE_RATE_HZ;

    // Each wheel gets 1/2 the angular rate
    new_angular_velocity =  ( (int32_t)angular_velocity * 256 ) / ( MOTION_CONTROL_UPDATE_RATE_HZ * 2 );

    total_velocity = abs( linear_velocity ) + abs( angular_velocity / 2 );

    if ( total_velocity > m_max_velocity )  // Exceeding wheel speed limits
    {
        // Reduce both velocities by the "overage ratio"
        new_linear_velocity = ( ( new_linear_velocity * m_max_velocity ) / total_velocity );
        new_angular_velocity = ( ( new_angular_velocity * m_max_velocity ) / total_velocity );
    }

    // Positive angular velocity causes the right wheel to spin faster.
    Motion_Control_Set_Update_Velocity( (int16_t)new_linear_velocity, -1 * new_angular_velocity, &g_left_wheel_motion );
    Motion_Control_Set_Update_Velocity( (int16_t)new_linear_velocity, new_angular_velocity, &g_right_wheel_motion );

    // Copy data to outgoing telemetry
    gp_telemetry_write->linear_velocity = linear_velocity;
    gp_telemetry_write->angular_velocity = angular_velocity;
}

void  Motion_Control_EStop( void )
{
    g_estop = 1;

    Motors_Set_Direction( MOTORS_L_INDEX, MOTORS_BRAKE );
    Motors_Set_Direction( MOTORS_R_INDEX, MOTORS_BRAKE );

    Motion_Control_Set_Velocity( 0, 0 );
}

void  Motion_Control_Stop( void )
{
    Motion_Control_Set_Velocity( 0, 0 );
}

void    Motion_Control_Set_Update_Velocity( int16_t linear_velocity,    // s7.8 ticks/update
                                            int16_t angular_velocity,   // s7.8 ticks/update
                                            volatile Motion_State_t *p_state )
{
    p_state->linear_velocity_setpoint = linear_velocity;
    p_state->angular_velocity_setpoint = angular_velocity;
}

/** This function computes an entire update cycle.
 */
int16_t Motion_Control_Compute_Power(   int8_t new_encoder_ticks,     // s15.0 ticks
                                        volatile Motion_State_t *p_state )
{
    Motion_Control_Add_To_Encoder( new_encoder_ticks, p_state );

    Motion_Control_Compute_Target_Position( p_state );

    return Motion_Control_Run_PID( new_encoder_ticks, p_state );
}

void Motion_Control_Add_To_Encoder( int8_t new_encoder_ticks,  // s7.0 ticks 
                                    volatile Motion_State_t *p_state )
{
    p_state->encoder += new_encoder_ticks;
}

/** This function runs a PID correction.
 */
int16_t Motion_Control_Run_PID( int8_t new_encoder_ticks,  // s7.0 ticks 
                                volatile Motion_State_t *p_state )
{
  int16_t power;
  int16_t error_squared;

  power = Pid_Compute_Correction( new_encoder_ticks, &( p_state->pid ) );

  if  ( ( p_state->linear_velocity == 0 ) &&
        ( p_state->angular_velocity == 0 ) &&
        (( error_squared = p_state->pid.error * p_state->pid.error ) < 5) ) // 10 should mean within 3 encoder ticks
  {
    power = (power * error_squared) / 8;
  }

  p_state->power_out = power;

  return power;
}

/** Add an s7.8 value to the current setpoint.
 *
 *  Do this with interrupts off since the motor control
 *  task might be reading the value. -where? SJL
 *
 */
void Motion_Control_Add_To_Position( int16_t num_ticks, volatile Motion_State_t *p_motion )
{
    int16_t new_pid_setpoint;

    new_pid_setpoint = (int16_t)( ( p_motion->encoder_setpoint / 256) - p_motion->encoder );

    // Prevent run-away setpoints! If there is already more positive error, than the velocity
    // you are trying to acheive, don't add more.
    if ( (num_ticks > 0) && (( num_ticks / 256 ) > new_pid_setpoint) )
    { 
        p_motion->encoder_setpoint += num_ticks; // Setpoint and num_ticks are sX.8 numbers
        new_pid_setpoint = (int16_t)( ( p_motion->encoder_setpoint / 256) - p_motion->encoder );
    }    
    // Similarly if there is already more error (going backward), than the velocity
    // you are trying to acheive, don't add more.
    else if ( (num_ticks < 0) && (( num_ticks / 256 ) < new_pid_setpoint) )
    { 
        p_motion->encoder_setpoint += num_ticks; // Setpoint and num_ticks are sX.8 numbers
        new_pid_setpoint = (int16_t)( ( p_motion->encoder_setpoint / 256) - p_motion->encoder );
    }    
    
    // HACK!!! new_pid_setpoint should be around 5, so 100 should be Way safe.
    // The pid setpoint was routinely getting set to 170-220...
    // With this change I have seen no anomalous behavior.
    if ( CLAMP_VALUE_NOT_CLAMPED == Clamp_Abs_Value(&new_pid_setpoint, 100 ) )
    {
        p_motion->pid.setpoint = new_pid_setpoint;
    }
}

void Motion_Control_Compute_Target_Position( volatile Motion_State_t *p_motion )
{
    int16_t new_velocity;

    // Linear Velocity
    if ( p_motion->linear_velocity < p_motion->linear_velocity_setpoint )
    {
        p_motion->linear_velocity += m_linear_acceleration;

        if ( p_motion->linear_velocity > p_motion->linear_velocity_setpoint )
        {
            p_motion->linear_velocity = p_motion->linear_velocity_setpoint;
        }
    }
    else
    {
        p_motion->linear_velocity -= m_linear_acceleration;

        if ( p_motion->linear_velocity < p_motion->linear_velocity_setpoint )
        {
            p_motion->linear_velocity = p_motion->linear_velocity_setpoint;
        }
    }

    // Angular Velocity
    if ( p_motion->angular_velocity < p_motion->angular_velocity_setpoint )
    {
        p_motion->angular_velocity += m_angular_acceleration; 

        if ( p_motion->angular_velocity > p_motion->angular_velocity_setpoint )
        {
            p_motion->angular_velocity = p_motion->angular_velocity_setpoint;
        }
    }
    else
    {
        p_motion->angular_velocity -= m_angular_acceleration;

        if ( p_motion->angular_velocity < p_motion->angular_velocity_setpoint )
        {
            p_motion->angular_velocity = p_motion->angular_velocity_setpoint;
        }
    }

    new_velocity = p_motion->linear_velocity + p_motion->angular_velocity;

    Motion_Control_Add_To_Position( new_velocity, p_motion );
}
