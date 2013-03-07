/** @file checksum.c
 *  
 *
 *  Created:    03/09/2011 11:30:14 PM
 *  Compiler:   gcc
 *  @author:    Sawyer Larkin (SJL), DATAProject@therobotguy.com
 *
 *  Last Changed:   $id$
 *
 */

#include "checksum.h"

// Value needed for x8+x2+x+1
#define POLYVAL 0x8C

/** Calculates the CRC 8 checksum over a range of data.
 *  
 *  @param  *data       Pointer to the data buffer to be read.
 *  @param  num_bytes   Number of bytes to read from @a data.
 *  @param  seed        Starting value for the checksum calculations.
 *
 *  @returns    The calculated checksum.
 */
uint8_t
CRC8_Checksum( uint8_t *data, uint32_t num_bytes, uint8_t seed )
{
    uint8_t crc = seed;

    for (int i = 0; i < num_bytes; ++i)
    {
        CRC8_Update(data[i], &crc); 
    }

    return crc;
}

void 
CRC8_Update( uint8_t new_byte, uint8_t * const current_crc )
{
   uint8_t new_crc;

   new_crc = *current_crc;

   for (uint8_t i = 0; i < 8; ++i) 
   {
      if ((new_crc ^ new_byte) & 1) 
      {
         new_crc = (new_crc >> 1 ) ^ POLYVAL;
      }
      else 
      {
         new_crc >>= 1;
      }

      new_byte >>= 1;
   }

   *current_crc = new_crc;
}
