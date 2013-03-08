#include <limits.h>

#include "RollOverHelpers.h"

/** Provides the difference in values while trying to detect and correct
 *  for roll-over.
 */
int8_t DifferentiateInt8RollOver( int8_t  old_value,  int8_t new_value )
{
  int16_t difference;

  difference = (int16_t)new_value - (int16_t)old_value;

  // Roll over from negative over flow
  if ( difference > INT8_MAX )
  {
    difference = ( (new_value - INT8_MAX) + (INT8_MIN - old_value) - 1 );
  }
  // Roll over from positive over flow
  else if ( difference < INT8_MIN )
  {
    difference = ( (INT8_MAX - old_value) + (new_value - INT8_MIN) + 1 );
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

  // Roll over from negative over flow
  if ( difference > INT16_MAX )
  {
    difference = ( (new_value - INT16_MAX) + (INT16_MIN - old_value) - 1 );
  }
  // Roll over from positive over flow
  else if ( difference < INT16_MIN )
  {
    difference = ( (INT16_MAX - old_value) + (new_value - INT16_MIN) + 1 );
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

  // Roll over from negative over flow
  if ( difference > INT32_MAX )
  {
    difference = ( (new_value - INT32_MAX) + (INT32_MIN - old_value) - 1 );
  }
  // Roll over from positive over flow
  else if ( difference < INT32_MIN )
  {
    difference = ( (INT32_MAX - old_value) + (new_value - INT32_MIN) + 1 );
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
