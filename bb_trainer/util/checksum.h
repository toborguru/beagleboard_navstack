/** @file checksum.h
 *  Provides the functions to calculate checksums.
 *
 *  Currently implemented: @n
 *  CRC8
 *
 *  Created:    03/09/2011 11:18:45 PM
 *  Compiler:   gcc
 *  @author:    Sawyer Larkin (SJL), DATAProject@therobotguy.com
 *
 *  Last Changed:   $id$
 *
 */

#include <inttypes.h>

#ifndef _CHECKSUM_H_
#define _CHECKSUM_H_

uint8_t
CRC8_Checksum( uint8_t *data, uint32_t num_bytes, uint8_t seed );

#endif
