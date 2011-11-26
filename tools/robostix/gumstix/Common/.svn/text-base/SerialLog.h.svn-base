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
*   @file   SerialLog.h
*
*   @brief  This file contains the interface for a serial logger which allows
*           logging information from a serial port to be logged to the 
*           console.
*
*           In addition, the serial logging information can be colorized,
*           which allows it to be distinguished from the logging from the
*           the main program.
*
****************************************************************************/

#if !defined( SERIALLOG_H )
#define SERIALLOG_H                 ///< Include Guard

// ---- Include Files -------------------------------------------------------

#include <termios.h>

// ---- Constants and Types -------------------------------------------------

#define SERIAL_LOG_FFUART   0
#define SERIAL_LOG_BTUART   1
#define SERIAL_LOG_STUART   2
#define SERIAL_LOG_HWUART   3

#define SERIAL_LOG_NO_COLOR 0
#define SERIAL_LOG_COLORIZE 1

extern  int gSerialLogColorize;

// ---- Functions -----------------------------------------------------------

#if defined( __cplusplus )
extern "C"
{
#endif

int GetSerialLogFileDescriptor( void );

void StartSerialLog( speed_t speed, int port );
void StopSerialLog( void );

#if defined( __cplusplus )
}
#endif

#endif  // SERIALLOG_H

