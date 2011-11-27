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

//#define F_CPU 8E6

// Remove
//#include <util/delay.h>
#define MOTION_STEP_DELAY   1000 / MOTION_CONTROL_UPDATE_RATE_HZ

#define MOTOR_TEST_EN 0	// If non-zero a repeating pattern will be run .
#define MOTION_TEST_DELAY   1500
#define MOTION_TEST_SPEED1  80
#define MOTION_TEST_TURN1   0
#define MOTION_TEST_SPEED2  80
#define MOTION_TEST_TURN2   80


#include "base_motion/motors.h"
#include "base_motion/shaft_encoders.h"
#include "base_motion/motion_control.h"
#include "i2c/TWI_slave.h"

#include "Avr.h"
#include "i2c_registers.h"
#include "macros.h"
#include "system_clock.h"

#define MOT_CMD_ENABLED 1
#define MAX_STEPS_BETWEEN_COMMANDS  15 // 50 Hz Steps (last I checked...)

#define I2C_INVALID_VELOCITY    (int16_t)(-32768)

volatile I2C_REGISTERS_t* gp_commands_read;
volatile I2C_REGISTERS_t* gp_commands_write;

volatile I2C_REGISTERS_t* gp_telemetry_read;
volatile I2C_REGISTERS_t* gp_telemetry_write;

static I2C_REGISTERS_t m_commands;
static I2C_REGISTERS_t m_telemetry_a; 
static I2C_REGISTERS_t m_telemetry_b; 

void Ports_Zero( void );

void Switch_Telemetry_Buffers( void );

int main( void )
{
  DISABLE_INTERRUPTS();

  SYSTEM_CLOCK_T  motion_step_time = 0;
  uint8_t TWI_slaveAddress = 0x10;

  uint8_t steps_since_command = 0;

  gp_commands_read = &m_commands;
  gp_commands_write = &m_commands;
  gp_TWI_receive_buf = (uint8_t*)&m_commands;

  // After this these pointers are maintained in Switch_Telemetry_Buffer()
  gp_telemetry_read = &m_telemetry_a;
  gp_telemetry_write = &m_telemetry_b;
  gp_TWI_transmitBuf = (uint8_t*)gp_telemetry_read;

#if MOTOR_TEST_EN
  SYSTEM_CLOCK_T  motion_test_time = MOTION_TEST_DELAY;
  uint8_t motors_state = 1;
#endif

  // Hardware Init
  Ports_Zero();
  Motors_Init();

  Motion_Control_Init();
  Shaft_Encoders_Init();
  System_Clock_Init();
  TWI_Slave_Initialise( TWI_slaveAddress << TWI_ADR_BITS );
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
    if ( (uint16_t)g_system_clock != gp_telemetry_read->system_time )
    {
      // Update both if the buffers are not active
      if ( !g_TWI_writeInProgress ) 
      {
        gp_telemetry_write->system_time = g_system_clock;
      }

      if ( !g_TWI_readInProgress )
      {
        gp_telemetry_read->system_time = g_system_clock;
      }
    }
 
    if ( Clock_Diff(motion_step_time, g_system_clock) <= 0 )
    {
      Motion_Control_Run_Step();

      Switch_Telemetry_Buffers();

      motion_step_time += MOTION_STEP_DELAY;
      motion_step_time %= SYSTEM_CLOCK_MAX;
      steps_since_command++;

      if ( steps_since_command > MAX_STEPS_BETWEEN_COMMANDS )
      { 
        Motion_Control_Stop();
        steps_since_command = 0;
      }
    }

    if ( g_TWI_writeComplete )
    {
      if ( ( gp_commands_read->linear_velocity != I2C_INVALID_VELOCITY ) &&
          ( gp_commands_read->angular_velocity != I2C_INVALID_VELOCITY ) )
      {
        steps_since_command = 0;

        Motion_Control_Set_Velocity( gp_commands_read->linear_velocity, 
            gp_commands_read->angular_velocity );

        gp_commands_read->linear_velocity = I2C_INVALID_VELOCITY;
        gp_commands_read->angular_velocity = I2C_INVALID_VELOCITY;
      }

      g_TWI_writeComplete = false;
    }

#if MOTOR_TEST_EN
    if ( Clock_Diff(motion_test_time, g_system_clock) <= 0 )
    {
      if (motors_state)
      {
        Motion_Control_Set_Velocity( MOTION_TEST_SPEED1, MOTION_TEST_TURN1 );
      }
      else
      {
        Motion_Control_Set_Velocity( MOTION_TEST_SPEED2, MOTION_TEST_TURN2 );
      }

      motors_state = !motors_state;

      motion_test_time += MOTION_TEST_DELAY;
      motion_test_time %= SYSTEM_CLOCK_MAX;
    }
#endif
  }
  // Never reached.
  return(0);
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
  ucCount++;
  eeprom_write_byte( mainRESET_COUNT_ADDRESS, ucCount );
}
#endif
