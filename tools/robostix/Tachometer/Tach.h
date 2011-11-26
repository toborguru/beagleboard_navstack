/****************************************************************************
*
*   Copyright (c) 2006 Dave Hylands     <dhylands@gmail.com>
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation.
*
*   Alternatively, this software may be distributed under the terms of BSD
*   license.
*
*   See README and COPYING for more details.
*
****************************************************************************/

#if !defined( TACH_H )
#define TACH_H

// ---- Include Files -------------------------------------------------------

#include <inttypes.h>
#include <avr/io.h>
#include "CBUF.h"

// ---- Constants and Types -------------------------------------------------

#define TACH_NUM_ENTRIES    16  // Must be power of 2

typedef struct
{
    uint8_t     m_getIdx;
    uint8_t     m_putIdx;
    uint32_t    m_entry[ TACH_NUM_ENTRIES ];

} TACH_Buffer_t;

#define TACH_gBuffer_SIZE   TACH_NUM_ENTRIES

// ---- Variable Externs ----------------------------------------------------

extern  volatile    TACH_Buffer_t   TACH_gBuffer;

// ---- Function Prototypes -------------------------------------------------


#endif  // TACH_H

