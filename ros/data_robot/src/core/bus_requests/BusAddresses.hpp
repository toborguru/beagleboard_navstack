/** This file contains the I2C addresses for DATA's various systems.
 */

#include <inttypes.h>

#ifndef GUARD_BusAddresses
#define GUARD_BusAddresses

#define ADDRESS_SIZE  2

namespace data_robot_core
{
enum I2CAddresses_T
{
  BB_TRAINER  = 0x06,
  FRONT_SHELL = 0x16
};

enum BbTrainerOffsets_T
{
  COMMAND_OFFSET        = 0x00,
  TICK_VELOCITY_OFFSET  = 0x02,
  ENCODER_COUNTS_OFFSET = 0x06
};

enum FrontShellOffsets_T
{
  FRONT_TELEMETRY_OFFSET  = 0x00
};
}

#endif
