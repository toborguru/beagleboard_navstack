#include <stdlib.h>

#include "../util/pid.h"
#include "../util/clamp.h"
#include "../Avr.h"
#include "../i2c_registers.h"
#include "../system_clock.h"

#include "motors.h"
#include "motion_control.h"
#include "shaft_encoders.h"

/* GLOBAL VARIABLES */
volatile Motion_State_t g_left_wheel_motion;
volatile Motion_State_t g_right_wheel_motion;

volatile int32_t g_encoder_total = 0;
volatile int32_t g_stasis_total = 0;

volatile uint8_t g_estop = 0;

/* MODULE VARIABLES */
static uint16_t   m_linear_acceleration;      // u8.8 ticks/update^2
static uint16_t   m_angular_acceleration;     // u8.8 ticks/update^2
static uint16_t   m_max_velocity;             // u16.0 ticks/sec

/* MODULE FUNCTION PROTOTYPES */
static void Motion_Control_Init_State(  volatile Motion_State_t *p_state );

static void Motion_Control_Init_Acceleration( void );

// Returns power_out- runs Add To Encoder, Compute Position then Run PID
static int16_t Motion_Control_Compute_Power(  int8_t new_encoder_ticks,     // s7.0 ticks
                                              volatile Motion_State_t *p_state );      

// Adds new encoder counts
static void Motion_Control_Add_To_Encoder(  int8_t new_encoder_ticks,       // s7.0 ticks 
                                            volatile Motion_State_t *p_state );     

// Computes the current target position
static void Motion_Control_Compute_Target_Position( volatile Motion_State_t *p_motion );

// Returns the correction from the PID controller
static int16_t Motion_Control_Run_PID(  int8_t new_encoder_ticks,           // s7.0 ticks
                                        volatile Motion_State_t *p_state );

// Adds new fixed point counts to the target position
static void Motion_Control_Add_To_Position( int16_t num_ticks,              // s7.8 ticks
                                            volatile Motion_State_t *p_motion);

static void Motion_Control_Set_Update_Velocity( int16_t linear_velocity,    // s7.8 ticks/update
                                                int16_t angular_velocity,   // s7.8 ticks/update
                                                volatile Motion_State_t *p_state );

/* FUNCTIONS */

void Motion_Control_Init(   void )
{
    Motion_Control_Init_Acceleration();
    Motion_Control_Init_State( &g_left_wheel_motion );
    Motion_Control_Init_State( &g_right_wheel_motion );
    Motion_Control_Set_Velocity( 0, 0 );

    // s15.0 -> s7.8
    m_max_velocity = (uint16_t)( (uint32_t)(MOTION_CONTROL_DEFAULT_MAX_VELOCITY * 256) / MOTION_CONTROL_UPDATE_RATE_HZ );
    
    Motors_Set_Direction(MOTORS_L_INDEX, MOTORS_FORWARD);
    Motors_Set_Direction(MOTORS_R_INDEX, MOTORS_FORWARD);
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
    // u16.0 -> u8.8 conversion also scaling for update rate
    m_linear_acceleration = ( linear_acceleration * 256 ) / MOTION_CONTROL_UPDATE_RATE_HZ;
    
    // u16.0 -> u8.8 conversion also scaling for update rate 
    m_angular_acceleration = ( angular_acceleration * 256 ) / MOTION_CONTROL_UPDATE_RATE_HZ;
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

    left_power =    Motion_Control_Compute_Power(   delta_left,   p_l_wheel );
    right_power =   Motion_Control_Compute_Power(   delta_right,  p_r_wheel );

    // Set motor power
    if (!g_estop)
    {
        if (left_power < 0)
        {
            left_power *= -1;
            Motors_Set_Direction(   MOTORS_L_INDEX, MOTORS_REVERSE );
        }
        else if (left_power > 0)
        {
            Motors_Set_Direction(   MOTORS_L_INDEX, MOTORS_FORWARD );
        }
        else
        {
            Motors_Set_Direction(   MOTORS_L_INDEX, MOTORS_COAST );
        }
         
        if (right_power < 0)
        {
            right_power *= -1;
            Motors_Set_Direction(   MOTORS_R_INDEX, MOTORS_REVERSE );
        }
        else if (right_power > 0)
        {
            Motors_Set_Direction(   MOTORS_R_INDEX, MOTORS_FORWARD );
        }
        else
        {
            Motors_Set_Direction(   MOTORS_R_INDEX, MOTORS_COAST );
        }

        Motors_Set_Power(   MOTORS_L_INDEX, left_power & 0xFF  );
        Motors_Set_Power(   MOTORS_R_INDEX, right_power & 0xFF );
    }

    // Update telemetry registers
    gp_telemetry->left_encoder = (int16_t)(p_l_wheel->encoder);
    gp_telemetry->right_encoder = (int16_t)(p_r_wheel->encoder);
    gp_telemetry->stasis_encoder = stasis_total;
    gp_telemetry->encoder_time = measurement_time;
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

    total_velocity = abs( linear_velocity ) + abs( angular_velocity );

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
    gp_telemetry->linear_velocity = new_linear_velocity;
    gp_telemetry->angular_velocity = new_angular_velocity;
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

/* MODULE FUNCTIONS */
/** This function initializes a Motion_State_t structure, using #defined defaults
 *  for the PID tuning parameters.
 */
static void Motion_Control_Init_State( volatile Motion_State_t *p_state )
{
    // Reset motion state
    p_state->angular_velocity = 0;
    p_state->angular_velocity_setpoint = 0;

    p_state->linear_velocity = 0;
    p_state->linear_velocity_setpoint = 0;

    p_state->encoder_setpoint = 0;
    p_state->encoder = 0;

    // PID defaults, scaling PID defaults for float -> u8.8 conversion
    Motion_Control_Set_PID( p_state, 
                            (uint16_t)(MOTION_CONTROL_DEFAULT_KP * 256.0), 
                            (uint16_t)(MOTION_CONTROL_DEFAULT_KD * 256.0), 
                            (uint16_t)(MOTION_CONTROL_DEFAULT_KI * 256.0) );

    p_state->pid.max_output= MOTION_CONTROL_MAX_CORRECTION;
    p_state->pid.min_output= -1 * MOTION_CONTROL_MAX_CORRECTION;

    // Reset PID state
    p_state->pid.setpoint = 0;
    p_state->pid.error = 0;
    p_state->pid.d_error = 0;
    p_state->pid.i_term = 0;
    p_state->pid.prev_input = 0;
}

/** Sets the default accelerations per #defines.
 */
static void Motion_Control_Init_Acceleration( void )
{
    Motion_Control_Set_Acceleration(    MOTION_CONTROL_DEFAULT_LINEAR_ACCEL, 
                                        MOTION_CONTROL_DEFAULT_ANGULAR_ACCEL );
}

static void  Motion_Control_Set_Update_Velocity( int16_t linear_velocity,    // s7.8 ticks/update
                                          int16_t angular_velocity,   // s7.8 ticks/update
                                          volatile Motion_State_t *p_state )
{
    p_state->linear_velocity_setpoint = linear_velocity;
    p_state->angular_velocity_setpoint = angular_velocity;
}

/** This function computes an entire update cycle.
 */
static int16_t Motion_Control_Compute_Power(  int8_t new_encoder_ticks,     // s15.0 ticks
                                              volatile Motion_State_t *p_state )
{
    Motion_Control_Add_To_Encoder( new_encoder_ticks, p_state );

    Motion_Control_Compute_Target_Position( p_state );

    return Motion_Control_Run_PID( new_encoder_ticks, p_state );
}

static void Motion_Control_Add_To_Encoder(  int8_t new_encoder_ticks,  // s7.0 ticks 
                                            volatile Motion_State_t *p_state )
{
    p_state->encoder += new_encoder_ticks;
}

/** This function runs a PID correction.
 */
static int16_t Motion_Control_Run_PID(  int8_t new_encoder_ticks,  // s7.0 ticks 
                                        volatile Motion_State_t *p_state )
{
  int16_t power;
  int16_t error_squared;

  power = Pid_Compute_Output( new_encoder_ticks, &( p_state->pid ) );

  if  ( ( p_state->linear_velocity == 0 ) &&
        ( p_state->angular_velocity == 0 ) &&
        ( ( error_squared = p_state->pid.error * p_state->pid.error ) < 5) ) // 10 should mean within 3 encoder ticks
  {
    power = 0;
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
static void Motion_Control_Add_To_Position( int16_t num_ticks, volatile Motion_State_t *p_motion )
{
  int16_t existing_error;
  int16_t new_pid_setpoint;

  existing_error = ( (int16_t)(p_motion->encoder_setpoint / 256) - (int16_t)(p_motion->encoder) );

  // Prevent run-away setpoints! If there is already more error than the velocity
  // you are trying to achieve, clamp the previous error.
  // Commanded vel is higher than existing error
  if ( abs(num_ticks) > (abs(existing_error) * 256) )
  { 
    p_motion->encoder_setpoint += num_ticks; // Setpoint and num_ticks are sX.8 numbers
  }    
  // Existing error is running past commanded vel- clamp to full vel error plus/minus vel
  else
  { 
    if ( (num_ticks * existing_error) > 0 ) // Matching signs - falling behind commanded point, clamp error
    {	      
    	p_motion->encoder_setpoint = (p_motion->encoder * 256) + (2 * num_ticks); // Setpoint and num_ticks are sX.8 numbers
    }
    else // Mismatched signs or num_ticks == 0 - running ahead of commanded point or we're already there
    {
    	p_motion->encoder_setpoint = (p_motion->encoder * 256); // Setpoint and num_ticks are sX.8 numbers
    }
  }

  new_pid_setpoint = (int16_t)( ( p_motion->encoder_setpoint / 256) - p_motion->encoder );

  p_motion->pid.setpoint = new_pid_setpoint;
}

static void Motion_Control_Compute_Target_Position( volatile Motion_State_t *p_motion )
{
    int16_t new_velocity;

    // Apply acceleration to velocity
    if ( p_motion->linear_velocity < p_motion->linear_velocity_setpoint )
    {
        p_motion->linear_velocity += m_linear_acceleration;

        // We've reached our goal
        if ( p_motion->linear_velocity > p_motion->linear_velocity_setpoint )
        {
            p_motion->linear_velocity = p_motion->linear_velocity_setpoint;
        }
    }
    else if ( p_motion->linear_velocity > p_motion->linear_velocity_setpoint )
    {
        p_motion->linear_velocity -= m_linear_acceleration;

        // We've reached our goal
        if ( p_motion->linear_velocity < p_motion->linear_velocity_setpoint )
        {
            p_motion->linear_velocity = p_motion->linear_velocity_setpoint;
        }
    }

    // Apply acceleration to velocity
    if ( p_motion->angular_velocity < p_motion->angular_velocity_setpoint )
    {
        p_motion->angular_velocity += m_angular_acceleration; 

        // We've reached our goal
        if ( p_motion->angular_velocity > p_motion->angular_velocity_setpoint )
        {
            p_motion->angular_velocity = p_motion->angular_velocity_setpoint;
        }
    }
    else if ( p_motion->angular_velocity > p_motion->angular_velocity_setpoint )
    {
        p_motion->angular_velocity -= m_angular_acceleration;

        // We've reached our goal
        if ( p_motion->angular_velocity < p_motion->angular_velocity_setpoint )
        {
            p_motion->angular_velocity = p_motion->angular_velocity_setpoint;
        }
    }

    new_velocity = p_motion->linear_velocity + p_motion->angular_velocity;

    Motion_Control_Add_To_Position( new_velocity, p_motion );
}

/** Assign u8.8 PID tuning parameters.
 */
void Motion_Control_Set_PID( Motion_State_t *p_state, uint16_t k_p, uint16_t k_i, uint16_t k_d )
{
  // u8.8 -> float conversion and scaling for update rate too
  p_state->pid.kp = k_p / ( 256.0 * MOTION_CONTROL_UPDATE_RATE_HZ );
  p_state->pid.ki = k_i / ( 256.0 * MOTION_CONTROL_UPDATE_RATE_HZ );
  p_state->pid.kd = k_d / ( 256.0 * MOTION_CONTROL_UPDATE_RATE_HZ );
}
