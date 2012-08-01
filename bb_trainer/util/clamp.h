/************************************************
 Sawyer Larkin
 D.A.T.A.

 clamp.h
 Provides functions to limit a variable to a range.
************************************************/

#ifndef __CLAMP_H
#define __CLAMP_H

#define CLAMP_VALUE_CLAMPED     1
#define CLAMP_VALUE_IN_RANGE    0
#define CLAMP_VALUE_NOT_CLAMPED CLAMP_VALUE_IN_RANGE

uint8_t Clamp_Abs_Value(int16_t* p_value, int16_t abs_max);

uint8_t Clamp_Value(int16_t* p_value, int16_t min_value, int16_t max_value);

#endif
