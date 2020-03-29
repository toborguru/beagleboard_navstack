#ifndef __I2C_COMMAND_H_
#define __I2C_COMMAND_H_

// Command definitions for I2C Command Register
// I2C Byte 0 - Cmd Group
// I2C Byte 1 - Cmd Code

#define CMD_GRP_POWER     0xDE // BIST start and stop commands
#define CMD_GRP_BIST      0xFE // BIST start and stop commands

// POWER Commands
#define CMD_POWER_SHELL   0x01
#define CMD_POWER_KILL    0xAD

// BIST Commands
#define CMD_BIST_STOP     0x00
#define CMD_BIST_MOTOR    0x01
#define CMD_BIST_PID      0x02
#define CMD_BIST_MOTION   0x03

#endif // __I2C_COMMANDS_H_
