/** This file contains the I2C addresses for DATA's various systems.
 */
#define ADDRESS_SIZE  2

#define BB_TRAINER  0x10

#define TICK_VELOCITY_INDEX     00
#define ENCODER_COUNTS_INDEX    04

#define TICK_VELOCITY_ADDRESS   0X1000
#define ENCODER_COUNTS_ADDRESS  0X1004

#define SJL_TEST  BB_TRAINER##TICK_VELOCITY_INDEX
