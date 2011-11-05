#ifndef GUARD_RollOVerHelpers
#define GUARD_RollOVerHelpers

#include <inttypes.h>


int8_t  DifferentiateInt8RollOver( int8_t  old_reading,  int8_t new_reading );
int16_t DifferentiateInt16RollOver( int16_t  old_reading,  int16_t new_reading );
int32_t DifferentiateInt32RollOver( int32_t  old_reading,  int32_t new_reading );

uint8_t   DifferentiateUint8RollOver( uint8_t  old_reading,  uint8_t new_reading );
uint16_t  DifferentiateUint16RollOver( uint16_t  old_reading,  uint16_t new_reading );
uint32_t  DifferentiateUint32RollOver( uint32_t  old_reading,  uint32_t new_reading );

#endif /* GUARD_RollOVerHelpers */
