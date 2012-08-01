/************************************************
 Sawyer Larkin
 D.A.T.A.

 clamp.c
 Pid Controller
************************************************/

#include <inttypes.h>

#include "clamp.h"

/** Clamps the value of p_to_clamp to within +-abs_max.
 */
uint8_t Clamp_Abs_Value(int16_t* p_to_clamp, int16_t abs_max)
{
   if (abs_max < 0)
   {
      abs_max *= -1;
   }

   return Clamp_Value(p_to_clamp, (abs_max * -1), abs_max);
}

/** Clamps the value of p_to_clamp to within min_value and max_value.
 */
uint8_t Clamp_Value(int16_t* p_to_clamp, int16_t min_value, int16_t max_value)
{
    uint8_t result = CLAMP_VALUE_IN_RANGE;

    if (*p_to_clamp > max_value)
    {
        *p_to_clamp = max_value;
        result = CLAMP_VALUE_CLAMPED;
    }
    else if (*p_to_clamp < min_value)
    {
        *p_to_clamp = min_value;
        result = CLAMP_VALUE_CLAMPED;
    }

    return result;
}
