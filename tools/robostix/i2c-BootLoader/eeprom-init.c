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
/**
*
*   @file   eeprom-init.c 
*
*   @brief  Creates a .hex file which can be used for initializing EEPROM
*
*****************************************************************************/

/* ---- Include Files ----------------------------------------------------- */

#include <avr/eeprom.h>

#include "BootLoader.h"

/* ---- Public Variables -------------------------------------------------- */

// We store the EEPROM data at the end of the EEPROM, so we need to modify
// the BootLoader.lds 
//
// For a size 20 of we use the following line in BootLoader.lds
//
//      eeprom (rw!x) : ORIGIN = 0x810fec, LENGTH = 20
//
//  The following extern declaration will fail if the sizeof 
//  BootLoaderEepromData_t isn't as expected.

extern char FixBootLoader_lds[ sizeof( BootLoaderEepromData_t ) == 20 ? 1 : -1 ];

// ---------------------------------------------------------------------------
// The default data

BootLoaderEepromData_t  __attribute__((section(".eeprom"))) gEepromDefaults =
{
    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },

    0xFF,   // pad
    0xFF,   // bootDelay
    0xFF,   // i2cAddr
    0xFF,   // structSize
};

/* ---- Private Constants and Types --------------------------------------- */
/* ---- Private Variables ------------------------------------------------- */
/* ---- Private Variables ------------------------------------------------- */
/* ---- Functions --------------------------------------------------------- */

