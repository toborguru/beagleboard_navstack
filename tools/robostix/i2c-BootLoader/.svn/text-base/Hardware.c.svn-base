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
*   @file    Hardware.c 
*
*   @brief   Performs hardware initialization
*
*****************************************************************************/

/* ---- Include Files ----------------------------------------------------- */

#include <avr/io.h>

#include "Config.h"
#include "Hardware.h"
#include "Timer.h"
#include "Uart.h"

/* ---- Public Variables -------------------------------------------------- */
/* ---- Private Constants and Types --------------------------------------- */
/* ---- Private Variables ------------------------------------------------- */
/* ---- Private Function Prototypes --------------------------------------- */
/* ---- Functions --------------------------------------------------------- */

/****************************************************************************/
/**
*   Initializes the hardware.
*
*   By havingg all of the hardware intitialization in one function, it allows
*   us to keep track of everything.
*/

void InitHardware( void )
{
    InitTimer();

#if CFG_USE_UART

    // Initialize the UART

    UBRRH = UBRR_INIT >> 8;
    UBRRL = UBRR_INIT & 0xFF;

    UCSRA = UCSRA_INIT;
    UCSRB = UCSRB_INIT;
    UCSRC = UCSRC_INIT;

#endif

} /* InitHardware */


