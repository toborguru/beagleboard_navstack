#ifndef __I2C_COMMAND_H_
#define __I2C_COMMAND_H_

// Command definitions for I2C Command Register
// I2C Byte 0 - Cmd Group
// I2C Byte 1 - Cmd Code

#define CMD_GRP_POWER       0xAD // BIST start and stop commands
#define CMD_GRP_BIST        0xFE // BIST start and stop commands

// POWER Commands
#define CMD_POWER_SHELL     0x01 // Power cycle the aux power, shells also kills I2C to BB
#define CMD_POWER_CANCEL    0x1E // Enable motor power
#define CMD_POWER_KILL      0xDE // Kill power to the robot

// BIST Commands
#define CMD_BIST_NONE       0x00 // Nominal operation
#define CMD_BIST_MOTOR      0x01 // Motor Pattern Test
#define CMD_BIST_PID        0x02 // PID Pattern Test
#define CMD_BIST_MOTION     0x03 // Motion Step Pattern Test
#define CMD_BIST_RAMP       0x04 // Motor Power Ramp Test

#endif // __I2C_COMMANDS_H_
