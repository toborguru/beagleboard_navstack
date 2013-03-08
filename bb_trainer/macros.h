#ifndef __MACROS_H_
#define __MACROS_H_

#include <inttypes.h>

#if( !defined( bool))
  typedef enum
  {
    false,
    true
  } bool;
#endif

uint8_t HwBitTestAndClear(uint8_t *data, uint8_t bitNumber);

#define BITS_CLEAR( target, data)               target &= (uint8_t)(~( data))
#define BIT_CLEAR( target, bitNumber)           BITS_CLEAR( target, BIT( bitNumber))
#define BITS_SET( target, data)                 target |= (uint8_t)( data)
#define BIT_SET( target, bitNumber)             BITS_SET( target, BIT( bitNumber))
#define BITS_TEST( data1, data2)                ( ( ( data1) & ( data2)) != 0)
#define BIT_TEST( data, bitNumber)              BITS_TEST( ( data), BIT( bitNumber))
#define BIT( bitNumber)                         (uint8_t)( 1 << ( bitNumber))
#define BIT_TEST_AND_CLEAR( data, bitNumber)    HwBitTestAndClear( ( data), ( bitNumber))

#define HIGH_BYTE( data)                        ( ( data & 0xFF00) >> 8)
#define LOW_BYTE( data)                         ( ( data) & 0xFF)

#define HIGH_NIBBLE( data)                      ( ( ( data) & 0xF0) >> 4)
#define LOW_NIBBLE( data)                       ( ( data) & 0x0F)

#define EXIT_SUCCESS    0
#define EXIT_ERROR      (-1)
#endif
