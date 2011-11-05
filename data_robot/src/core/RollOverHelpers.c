#include <limits.h>

#include "RollOverHelpers.h"

/** Provides the difference in values while trying to detect and correct
 *  for roll-over.
 */
int8_t DifferentiateInt8RollOver( int8_t  old_value,  int8_t new_value )
{
  int16_t difference;

  difference = (int16_t)new_value - (int16_t)old_value;

  // Roll over from negative velocity
  if ( difference > SCHAR_MAX )
  {
    difference = ( (new_value - SCHAR_MAX) + (SCHAR_MIN - old_value) - 1 );
  }
  // Roll over from positive velocity
  else if ( difference < SCHAR_MIN )
  {
    difference = ( (SCHAR_MAX - old_value) + (new_value - SCHAR_MIN) + 1 );
  }

  return (int8_t)difference;
}

/** Provides the difference in values while trying to detect and correct
 *  for roll-over.
 */
int16_t DifferentiateInt16RollOver( int16_t  old_value,  int16_t new_value )
{
  int32_t difference;

  difference = (int32_t)new_value - (int32_t)old_value;

  // Roll over from negative velocity
  if ( difference > SHRT_MAX )
  {
    difference = ( (new_value - SHRT_MAX) + (SHRT_MIN - old_value) - 1 );
  }
  // Roll over from positive velocity
  else if ( difference < SHRT_MIN )
  {
    difference = ( (SHRT_MAX - old_value) + (new_value - SHRT_MIN) + 1 );
  }

  return (int16_t)difference;
}

/** Provides the difference in values while trying to detect and correct
 *  for roll-over.
 */
int32_t DifferentiateInt32RollOver( int32_t  old_value,  int32_t new_value )
{
  long difference;

  difference = (long)new_value - (long)old_value;

  // Roll over from negative velocity
  if ( difference > INT_MAX )
  {
    difference = ( (new_value - INT_MAX) + (INT_MIN - old_value) - 1 );
  }
  // Roll over from positive velocity
  else if ( difference < INT_MIN )
  {
    difference = ( (INT_MAX - old_value) + (new_value - INT_MIN) + 1 );
  }

  return (int32_t)difference;
}

/** Provides the difference in values while trying to detect and correct
 *  for roll-over.
 */
uint8_t DifferentiateUint8RollOver( uint8_t  old_value,  uint8_t new_value )
{
  return new_value - old_value;
}

/** Provides the difference in values while trying to detect and correct
 *  for roll-over.
 */
uint16_t DifferentiateUint16RollOver( uint16_t  old_value,  uint16_t new_value )
{
  return new_value - old_value;
}

/** Provides the difference in values while trying to detect and correct
 *  for roll-over.
 */
uint32_t DifferentiateUint32RollOver( uint32_t  old_value,  uint32_t new_value )
{
  return new_value - old_value;
}
