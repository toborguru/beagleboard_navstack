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

#define F_CPU 16E6

#define I2C_ADDRESS         0x16

#define INVALID_COMMAND     0xFFFF

#include "i2c/TWI_slave.h"

#include "Avr.h"
#include "i2c_registers.h"
#include "macros.h"
#include "system_clock.h"

#define ANALOG_POWER_LOW_PORT   C
#define ANALOG_POWER_LOW_PIN    1

#define ANALOG_POWER_HIGH_PORT  C
#define ANALOG_POWER_HIGH_PIN   2

#define LASER_POWER_RED_PORT    B
#define LASER_POWER_RED_PIN     4

#define LASER_POWER_GREEN_PORT  B
#define LASER_POWER_GREEN_PIN   3

#define SERVO_1_PORT  B
#define SERVO_1_PIN   1

#define SERVO_2_PORT  B
#define SERVO_2_PIN   2

#define BUMPER1_PORT  D
#define BUMPER1_PIN   0

#define BUMPER2_PORT  D
#define BUMPER2_PIN   1

#define BUMPER3_PORT  D
#define BUMPER3_PIN   2

#define BUMPER4_PORT  D
#define BUMPER4_PIN   3

volatile I2C_REGISTERS_t* gp_commands_read;
volatile I2C_REGISTERS_t* gp_commands_write;

volatile I2C_REGISTERS_t* gp_telemetry_read;
volatile I2C_REGISTERS_t* gp_telemetry_write;

static I2C_REGISTERS_t m_commands;
static I2C_REGISTERS_t m_telemetry_a; 
static I2C_REGISTERS_t m_telemetry_b; 

void UpdateTelemetryClock( void );
void CheckBumpers( void );
void ProcessIncomingCommands( void );
void Ports_Zero( void );
void Switch_Telemetry_Buffers( void );

int main( void )
{
  DISABLE_INTERRUPTS();

  gp_commands_read = &m_commands;
  gp_commands_write = &m_commands;
  gp_TWI_receive_buf = (uint8_t*)&m_commands;

  gp_commands_read->command = INVALID_COMMAND;

  // After this these pointers are maintained in Switch_Telemetry_Buffer()
  gp_telemetry_read = &m_telemetry_a;
  gp_telemetry_write = &m_telemetry_b;
  gp_TWI_transmitBuf = (uint8_t*)gp_telemetry_read;

  // Hardware Init
  Ports_Zero();

  BIT_SET( _PORT(ANALOG_POWER_LOW_PORT), ANALOG_POWER_LOW_PIN );
  OUTPUT_PIN( ANALOG_POWER_LOW_PORT, ANALOG_POWER_LOW_PIN );

  BIT_SET( _PORT(ANALOG_POWER_HIGH_PORT), ANALOG_POWER_HIGH_PIN );
  OUTPUT_PIN( ANALOG_POWER_HIGH_PORT, ANALOG_POWER_HIGH_PIN );

  BIT_SET( _PORT(LASER_POWER_RED_PORT), LASER_POWER_RED_PIN );
  OUTPUT_PIN( LASER_POWER_RED_PORT, LASER_POWER_RED_PIN );

  BIT_SET( _PORT(LASER_POWER_GREEN_PORT), LASER_POWER_GREEN_PIN );
  OUTPUT_PIN( LASER_POWER_GREEN_PORT, LASER_POWER_GREEN_PIN );

  // Servo setup
  BIT_SET( _PORT(SERVO_1_PORT), SERVO_1_PIN );
  OUTPUT_PIN( SERVO_1_PORT, SERVO_1_PIN );

  BIT_SET( _PORT(SERVO_2_PORT), SERVO_2_PIN );
  OUTPUT_PIN( SERVO_2_PORT, SERVO_2_PIN );

  // Set Pull-up's
  BIT_SET( _PORT(BUMPER1_PORT), BUMPER1_PIN );
  INPUT_PIN( BUMPER1_PORT, BUMPER1_PIN );

  // Set Pull-up's
  BIT_SET( _PORT(BUMPER2_PORT), BUMPER2_PIN );
  INPUT_PIN( BUMPER2_PORT, BUMPER2_PIN );

  // Set Pull-up's
  BIT_SET( _PORT(BUMPER3_PORT), BUMPER3_PIN );
  INPUT_PIN( BUMPER3_PORT, BUMPER3_PIN );

  // Set Pull-up's
  BIT_SET( _PORT(BUMPER4_PORT), BUMPER4_PIN );
  INPUT_PIN( BUMPER4_PORT, BUMPER4_PIN );

  System_Clock_Init();
  TWI_Slave_Initialise( I2C_ADDRESS << TWI_ADR_BITS );
  TWI_Start_Transceiver();

  ENABLE_INTERRUPTS();

  for (;;)
  {
    UpdateTelemetryClock();

    CheckBumpers();

    ProcessIncomingCommands();

    Switch_Telemetry_Buffers();
  }
  // Never reached.
  return(0);
}

void UpdateTelemetryClock()
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
}

void CheckBumpers()
{
  uint8_t bumpers = _PIN( BUMPER1_PORT ) & 0x0F;

  gp_telemetry_write->command = bumpers;
}

void ProcessIncomingCommands()
{
  static SYSTEM_CLOCK_T  reset_step_time = 0;
  static bool in_reset = false;

  if ( in_reset )
  {
    // Check to see if it's time to re-apply power
    if ( Clock_Diff(reset_step_time, g_system_clock) <= 0 )
    {
      in_reset = false;
      // Turn power back on
      //BIT_SET(_PORT(SHELL_POWER_PORT), SHELL_POWER_PIN); 
    }
  }

  if ( g_TWI_writeComplete )
  {
    if ( gp_commands_read->command != INVALID_COMMAND )
    {
/*
      // Kill power to the whole robot
      if ( 0xDEAD == gp_commands_read->command )
      {
        BIT_SET(_PORT(KILL_PORT), KILL_PIN); 
      }

      // Power Cycle the Shells - If they are powered off we lose I2C comms...
      if ( 0x0601 == gp_commands_read->command )
      {
        BIT_CLEAR(_PORT(SHELL_POWER_PORT), SHELL_POWER_PIN); 

        reset_step_time = g_system_clock + SHELL_RESET_STEPS;
        reset_step_time %= SYSTEM_CLOCK_MAX;
        in_reset = true;
      }
*/
      gp_commands_read->command = INVALID_COMMAND;
    }
    
    g_TWI_writeComplete = false;
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
  ucCount++;
  eeprom_write_byte( mainRESET_COUNT_ADDRESS, ucCount );
}
#endif
