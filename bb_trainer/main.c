#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

//#include <util/delay.h>

#ifdef GCC_MEGA_AVR
/* EEPROM routines used only with the WinAVR compiler. */
#include <avr/eeprom.h> 
#endif

// Needed?
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "base_motion/motors.h"
#include "base_motion/shaft_encoders.h"
#include "base_motion/motion_control.h"
#include "i2c/TWI_slave.h"

#include "Avr.h"
#include "analog_in.h"
#include "i2c_registers.h"
#include "i2c_commands.h"
#include "macros.h"
#include "system_clock.h"

//#define F_CPU 8E6

#define I2C_ADDRESS         0x06

// Voltage reading below which: kill power to the entire bot.
#define VOLTAGE_KILL_LIMIT  300 // 300 ~= 6V, 325 ~= 6.5V, 350 ~= 7V

#define SHELL_RESET_STEPS   250 // 50 Hz Steps (last I checked...)
#define KILL_DELAY_STEPS    500 // 50 Hz Steps (last I checked...)

#define MOTION_STEP_DELAY   20 // millis (50Hz)
#define MAX_STEPS_BETWEEN_COMMANDS  45 // 50 Hz Steps (last I checked...)
#define MOT_CMD_ENABLED 1

#define KILL_PORT C
#define KILL_PIN  3

#define SHELL_POWER_PORT  D
#define SHELL_POWER_PIN   0

#define E_STOP_PORT D
#define E_STOP_PIN  1

#define I2C_INVALID_VELOCITY    (int16_t)(-32768)
#define INVALID_COMMAND         0xFF

// repeating pattern will be run using pid control
#define MOTION_TEST_DELAY   1500 // millis
#define MOTION_TEST_SPEED1  80
#define MOTION_TEST_TURN1   0
#define MOTION_TEST_SPEED2  80
#define MOTION_TEST_TURN2   80

// repeating pattern will be run using pid control
#define PID_TEST_DELAY   1500 // millis
#define PID_TEST_LEFT1   4
#define PID_TEST_RIGHT1  4
#define PID_TEST_LEFT2   0
#define PID_TEST_RIGHT2  0

// repeating pattern will be run using raw motor power
#define MOTOR_TEST_DELAY   1500 // millis
#define MOTOR_TEST_LEFT1   200
#define MOTOR_TEST_RIGHT1  200
#define MOTOR_TEST_LEFT2   100
#define MOTOR_TEST_RIGHT2  0

// ramp motor power up and down
#define RAMP_TEST_DELAY    500  // millis
#define RAMP_TEST_INC      4
#define RAMP_TEST_LOW      -64  // Min -255
#define RAMP_TEST_HIGH     64   // Max 255 

volatile I2C_REGISTERS_t* gp_commands_read;
volatile I2C_REGISTERS_t* gp_commands_write;

volatile I2C_REGISTERS_t* gp_telemetry_read;
volatile I2C_REGISTERS_t* gp_telemetry_write;

static I2C_REGISTERS_t m_commands;
static I2C_REGISTERS_t m_telemetry_a; 
static I2C_REGISTERS_t m_telemetry_b; 
static uint16_t m_steps_since_command = 0;

// Non-zero values override nominal system behavior and perform basic system operation tests
static uint16_t m_bist_running = CMD_BIST_NONE; 

void BaseMotion( void );
void CheckVoltage( void );
void RunTest( void );
void MotionPatternTest( void );
void PidPatternTest( void );
void MotorPatternTest( void );
void Ports_Zero( void );
void ProcessIncomingCommands( void );
void Switch_Telemetry_Buffers( void );
void UpdateTelemetry( void );
void UpdateTelemetryClock( void );

int main( void )
{
  DISABLE_INTERRUPTS();

  gp_commands_read = &m_commands;
  gp_commands_write = &m_commands;
  gp_TWI_receive_buf = (uint8_t*)&m_commands;

  gp_commands_read->command = INVALID_COMMAND;
  gp_commands_read->linear_velocity = I2C_INVALID_VELOCITY;
  gp_commands_read->angular_velocity = I2C_INVALID_VELOCITY;

  // After this these pointers are maintained in Switch_Telemetry_Buffer()
  gp_telemetry_read = &m_telemetry_a;
  gp_telemetry_write = &m_telemetry_b;
  gp_TWI_transmitBuf = (uint8_t*)gp_telemetry_read;

  // Hardware Init
  Ports_Zero();
  BIT_SET(_PORT(SHELL_POWER_PORT), SHELL_POWER_PIN); 
  OUTPUT_PIN(SHELL_POWER_PORT, SHELL_POWER_PIN);
  OUTPUT_PIN(KILL_PORT, KILL_PIN);

  //INPUT_PIN(E_STOP_PORT, E_STOP_PIN);

  AnalogInInit();
  Motors_Init();
  Motion_Control_Init();
  Shaft_Encoders_Init();
  System_Clock_Init();
  TWI_Slave_Initialise( I2C_ADDRESS << TWI_ADR_BITS );
  TWI_Start_Transceiver();

  // Turn on the Motors
#if MOT_CMD_ENABLED
  Motors_Enable();
#else
  Motors_Disable();
#endif

  ENABLE_INTERRUPTS();

  for (;;)
  {
    if ( m_bist_running == CMD_BIST_NONE )
    {
      UpdateTelemetryClock();

      BaseMotion();

      ProcessIncomingCommands();

      UpdateTelemetry();

      CheckVoltage();
    }
    else
    {
      RunTest();
    }
  }
  // Never reached.
  return(0);
}

void CheckVoltage()
{
  static uint_fast8_t limit_reached = 0;
  static SYSTEM_CLOCK_T run_time = 0;

  // Perform Motion Step
  if ( Clock_Diff(run_time, g_system_clock) <= 0 )
  {
    while ( Clock_Diff(run_time, g_system_clock) <= 0 )
    {
      run_time += 100; // Millis
      run_time &= SYSTEM_CLOCK_MASK;
    }

    if ( (gp_telemetry_write->voltage > 0) && (gp_telemetry_write->voltage < VOLTAGE_KILL_LIMIT) )
    { 
      ++limit_reached;
    }
    else
    {
      limit_reached = 0;
    }

    // 10 full seconds of limit reached...
    if ( limit_reached > 100 ) 
    {
      BIT_SET(_PORT(KILL_PORT), KILL_PIN);
    }
  }
}

void UpdateTelemetry()
{
  gp_telemetry_write->current = g_analog_values[ ANALOG_CURRENT_INDEX ];
  gp_telemetry_write->voltage = g_analog_values[ ANALOG_VOLTAGE_INDEX ];
  // TODO Remove !
  //gp_telemetry_write->status_flag = ! READ_PIN( E_STOP_PORT, E_STOP_PIN );
}

void UpdateTelemetryClock()
{
  if ( (uint16_t)g_system_clock != gp_telemetry_read->system_time )
  {
    // Update both if the buffers are not active
    gp_telemetry_write->system_time = g_system_clock;

    if ( !g_TWI_readInProgress )
    {
      gp_telemetry_read->system_time = g_system_clock;
    }
  }
}

void BaseMotion()
{
  static SYSTEM_CLOCK_T  run_time = 0;

  // Perform Motion Step
  if ( Clock_Diff(run_time, g_system_clock) <= 0 )
  {
    while ( Clock_Diff(run_time, g_system_clock) <= 0 )
    {
      run_time += MOTION_STEP_DELAY;
      run_time &= SYSTEM_CLOCK_MASK;
    }

    Motion_Control_Run_Step();

    Switch_Telemetry_Buffers();

    ++m_steps_since_command;

    if ( m_steps_since_command > MAX_STEPS_BETWEEN_COMMANDS )
    { 
      Motion_Control_Stop();
      m_steps_since_command = 0;
    }
  }
}

void ProcessIncomingCommands()
{
  static SYSTEM_CLOCK_T  reset_time = 0;
  static SYSTEM_CLOCK_T  kill_time = 0;

  // Check to see if a kill is scheduled
  if ( kill_time != 0 )
  {
    // Check to see if it's time to kill power (auto-destruct)
    if ( Clock_Diff(kill_time, g_system_clock) <= 0 )
    {
      // Kill power to the entire bot
      BIT_SET(_PORT(KILL_PORT), KILL_PIN); 
    }
  }

  // Check to see if the shells are in reset
  if ( reset_time != 0 )
  {
    // Check to see if it's time to re-apply shell power
    if ( Clock_Diff(reset_time, g_system_clock) <= 0 )
    {
      // Turn shell power back on
      BIT_SET(_PORT(SHELL_POWER_PORT), SHELL_POWER_PIN); 

      // Signal the shells are not in reset
      reset_time = 0;
    }
  }

  // Process incoming commands
  if ( g_TWI_writeComplete )
  {
    if ( CMD_GRP_POWER              == gp_commands_read->command_group )
    {
      // Schedule a power down
      if ( CMD_POWER_KILL           == gp_commands_read->command )
      {
        kill_time = g_system_clock + KILL_DELAY_STEPS;
        kill_time &= SYSTEM_CLOCK_MASK;
      }
      // Cancel a power down
      else if ( CMD_POWER_CANCEL    == gp_commands_read->command )
      {
        kill_time = 0;
      }
      // Power cycle the shells
      else if ( CMD_POWER_SHELL     == gp_commands_read->command )
      {
        BIT_CLEAR(_PORT(SHELL_POWER_PORT), SHELL_POWER_PIN); 

        reset_time = g_system_clock + SHELL_RESET_STEPS;
        reset_time &= SYSTEM_CLOCK_MASK;
      }
      else if ( CMD_POWER_NO_MOTOR  == gp_commands_read->command )
      {
        Motors_Disable();
      }
      else if ( CMD_POWER_MOTOR     == gp_commands_read->command )
      {
        Motors_Enable();
      }
    }
    else if ( CMD_GRP_BIST          == gp_commands_read->command_group )
    {
      if (  CMD_BIST_NONE           == gp_commands_read->command )
      {
        m_bist_running = CMD_BIST_NONE;
      } 
      else if ( CMD_BIST_MOTOR      == gp_commands_read->command )
      {
        m_bist_running = CMD_BIST_MOTOR;
      }
      else if ( CMD_BIST_PID        == gp_commands_read->command )
      {
        m_bist_running = CMD_BIST_PID;
      }
      else if ( CMD_BIST_MOTION     == gp_commands_read->command )
      {
        m_bist_running = CMD_BIST_MOTION;
      }
      else if ( CMD_BIST_RAMP       == gp_commands_read->command )
      {
        m_bist_running = CMD_BIST_RAMP;
      }
    }

    gp_commands_read->command_group = INVALID_COMMAND;
    gp_commands_read->command       = INVALID_COMMAND;
 
    if ( ( gp_commands_read->linear_velocity != I2C_INVALID_VELOCITY ) &&
        ( gp_commands_read->angular_velocity != I2C_INVALID_VELOCITY ) )
    {
      m_steps_since_command = 0;

      Motion_Control_Set_Velocity( gp_commands_read->linear_velocity, 
          gp_commands_read->angular_velocity );

      gp_commands_read->linear_velocity = I2C_INVALID_VELOCITY;
      gp_commands_read->angular_velocity = I2C_INVALID_VELOCITY;
    }

    g_TWI_writeComplete = false;
  }
}

void RunTest()
{
  UpdateTelemetryClock();

  ProcessIncomingCommands();

  if ( m_bist_running == CMD_BIST_MOTOR )
  {
    MotorPatternTest();
    m_steps_since_command = MAX_STEPS_BETWEEN_COMMANDS + 1;
  }
  else if ( m_bist_running == CMD_BIST_RAMP )
  {
    MotorRampTest();
    m_steps_since_command = MAX_STEPS_BETWEEN_COMMANDS + 1;
  }
  else if ( m_bist_running == CMD_BIST_PID )
  {
    PidPatternTest();
    m_steps_since_command = MAX_STEPS_BETWEEN_COMMANDS + 1;
  }
  else if ( m_bist_running == CMD_BIST_MOTION )
  {
    m_steps_since_command = 0;

    BaseMotion();

    MotionPatternTest();
  }
  
  UpdateTelemetry();

  CheckVoltage();
}

void MotionPatternTest()
{
  static SYSTEM_CLOCK_T  run_time = MOTION_TEST_DELAY;
  static uint8_t pattern_state = 1;

  if ( Clock_Diff(run_time, g_system_clock) <= 0 )
  {
    while ( Clock_Diff(run_time, g_system_clock) <= 0 )
    {
      run_time += MOTION_TEST_DELAY;
      run_time &= SYSTEM_CLOCK_MASK;
    }

    if (pattern_state)
    {
      Motion_Control_Set_Velocity( MOTION_TEST_SPEED1, MOTION_TEST_TURN1 );
    }
    else
    {
      Motion_Control_Set_Velocity( MOTION_TEST_SPEED2, MOTION_TEST_TURN2 );
    }

    pattern_state = !pattern_state;
  }
}

void PidPatternTest()
{
  volatile Motion_State_t* p_l_wheel = &g_left_wheel_motion;
  volatile Motion_State_t* p_r_wheel = &g_right_wheel_motion;

  static int16_t stasis_total = 0;

  int16_t delta_left;
  int16_t delta_right;

  int16_t left_power;
  int16_t right_power;

  static SYSTEM_CLOCK_T  run_time = MOTION_TEST_DELAY;
  static uint8_t pattern_state = 1;

  if ( Clock_Diff(run_time, g_system_clock) <= 0 )
  {
    while ( Clock_Diff(run_time, g_system_clock) <= 0 )
    {
      run_time += MOTION_TEST_DELAY;
      run_time &= SYSTEM_CLOCK_MASK;
    }

    if (pattern_state)
    {
      p_l_wheel->pid.setpoint = PID_TEST_LEFT1;
      p_r_wheel->pid.setpoint = PID_TEST_RIGHT1;
    }
    else
    {
      p_l_wheel->pid.setpoint = PID_TEST_LEFT2;
      p_r_wheel->pid.setpoint = PID_TEST_RIGHT2;
    }

    pattern_state = !pattern_state;

    // Get new encoder counts
    DISABLE_INTERRUPTS();
    delta_left = g_shaft_encoders_left_count;
    delta_right = g_shaft_encoders_right_count;
    stasis_total += g_shaft_encoders_stasis_count;

    g_shaft_encoders_left_count = 0;
    g_shaft_encoders_right_count = 0;
    g_shaft_encoders_stasis_count = 0;

    ENABLE_INTERRUPTS();

    // Set motor power
    left_power =    Pid_Compute_Output( delta_left, &( p_l_wheel->pid ) );
    right_power =   Pid_Compute_Output( delta_right, &( p_r_wheel->pid ) );

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
  }
}

void MotorPatternTest()
{
  static SYSTEM_CLOCK_T  run_time = MOTOR_TEST_DELAY;
  static uint8_t pattern_state = 1;

  if ( Clock_Diff(run_time, g_system_clock) <= 0 )
  {
    while ( Clock_Diff(run_time, g_system_clock) <= 0 )
    {
      run_time += MOTION_TEST_DELAY;
      run_time &= SYSTEM_CLOCK_MASK;
    }

    if (pattern_state)
    {
      Motors_Set_Power( MOTORS_L_INDEX, MOTOR_TEST_LEFT1);
      Motors_Set_Power( MOTORS_R_INDEX, MOTOR_TEST_RIGHT1);
    }
    else
    {
      Motors_Set_Power( MOTORS_L_INDEX, MOTOR_TEST_LEFT2);
      Motors_Set_Power( MOTORS_R_INDEX, MOTOR_TEST_RIGHT2);
    }

    pattern_state = !pattern_state;
  }
}

void MotorRampTest()
{
  static SYSTEM_CLOCK_T  run_time = 0;
  static int16_t test_power = 0;
  static uint8_t direction = 0;

  if ( Clock_Diff(run_time, g_system_clock) <= 0 )
  {
		uint8_t motor_power;

    while ( Clock_Diff(run_time, g_system_clock) < 0 )
    { 
      run_time += RAMP_TEST_DELAY;
      run_time &= SYSTEM_CLOCK_MASK;
    }
    
    if ( direction == 0 )
    {
      test_power += RAMP_TEST_INC;
    }
    else
    {
      test_power -= RAMP_TEST_INC;
    }

    if ( test_power >= RAMP_TEST_HIGH )
    {
      test_power = RAMP_TEST_HIGH;
      direction = !direction;
    }
    else if ( test_power <= RAMP_TEST_LOW )
    {
      test_power = RAMP_TEST_LOW;
      direction = !direction;
    }

		if (test_power < 0)
		{   
			motor_power = test_power * -1;
			Motors_Set_Direction(   MOTORS_L_INDEX, MOTORS_REVERSE );
			Motors_Set_Direction(   MOTORS_R_INDEX, MOTORS_REVERSE );
		}
		else
		{   
			motor_power = test_power;
			Motors_Set_Direction(   MOTORS_L_INDEX, MOTORS_FORWARD);
			Motors_Set_Direction(   MOTORS_R_INDEX, MOTORS_FORWARD);
		}
    
    Motors_Set_Power( MOTORS_L_INDEX, (uint8_t)motor_power );
    Motors_Set_Power( MOTORS_R_INDEX, (uint8_t)motor_power );
  }
}

void Ports_Zero()
{
  // Disable interupts
  //cli();

  PORTB = 0;
  DDRB =  0;
  PORTC = 0;
  DDRC =  0;
  PORTD = 0;
  DDRD =  0;

  // Enable interupts
  //sei();
}

void Switch_Telemetry_Buffers( void )
{
  static uint8_t reading_telemetry_a = true;

  DISABLE_INTERRUPTS();

  // Make sure we are not currently being read
  if ( !g_TWI_readInProgress )
  {
    if ( reading_telemetry_a )
    {
      gp_telemetry_read = &m_telemetry_b;
      gp_telemetry_write = &m_telemetry_a;

      reading_telemetry_a = false;
    }
    else
    {
      gp_telemetry_read = &m_telemetry_a;
      gp_telemetry_write = &m_telemetry_b;

      reading_telemetry_a = true;
    }

    // Do this here because of the interrupt wrappers
    gp_TWI_transmitBuf = (uint8_t*)gp_telemetry_read;
  }

  ENABLE_INTERRUPTS();
}

uint8_t HwBitTestAndClear(uint8_t *data, uint8_t bitNumber)
{   
  uint8_t bitClear;
  cli();
  bitClear = BIT_TEST(*data, bitNumber); 
  BIT_CLEAR(*data, bitNumber);
  sei();
  return bitClear;
}

#if COMMENTED_OUT
static void prvIncrementResetCount( void )
{
  unsigned char ucCount;

  eeprom_read_block( &ucCount, mainRESET_COUNT_ADDRESS, sizeof( ucCount ) );
  ++ucCount;
  eeprom_write_byte( mainRESET_COUNT_ADDRESS, ucCount );
}
#endif
