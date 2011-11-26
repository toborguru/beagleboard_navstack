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
*   @file   SerialLog.c
*
*   @brief  This file contains the implementation for a serial logger which 
*           allows loggging information from a serial port to be logged to 
*           the  console.
*
*           In addition, the serial logging information can be colorized,
*           which allows it to be distinguished from the logging from the
*           the main program.
*
****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <semaphore.h>
#include <pthread.h>

#include "SerialLog.h"
#include "Log.h"

// ---- Public Variables ----------------------------------------------------

int gSerialLogColorize = 1;

// ---- Private Constants and Types -----------------------------------------

// ---- Private Variables ---------------------------------------------------

struct
{
    int         rxdPin;
    const char *rxdCfg;

    int         txdPin;
    const char *txdCfg;

} gPortInfo[] = 
{
    { 34, "AF1\tin",    39, "AF2\tout" },   // FFUART
    { 42, "AF1\tin",    43, "AF2\tout" },   // BTUART
    { 46, "AF2\tin",    47, "AF1\tout" },   // STUART
    { 49, "AF1\tin",    48, "AF1\tout" },   // HWUART
};

int gPortFd = -1;

static  sem_t           gLogMutex;
static  struct termios  gOrgAttr;


// ---- Private Function Prototypes -----------------------------------------

static  void *ReaderThread( void *param );
static  void SerialLog( int logLevel, const char *fmt, va_list args );

// ---- Functions -----------------------------------------------------------

//***************************************************************************
/**
*   Configures a GPIO pin
*/

static void ConfigGpioPin( int pin, const char *cfgStr )
{
    FILE   *fs;
    char    gpioName[ 30 ];
    char    oldCfgStr[ 30 ];
    char    testCfgStr[ 30 ];
    int     len;

    // Check to see if the GPIO module has been loaded, and load it if
    // required.

    if ( access( "/proc/gpio", F_OK ) != 0 )
    {
        system( "modprobe proc_gpio" );
    }

    // Check to see how the pin is currently configured

    snprintf( gpioName, sizeof( gpioName ), "/proc/gpio/GPIO%d", pin );

    if (( fs = fopen( gpioName, "r" )) == NULL )
    {
        LogError( "Unable to open '%s' for reading\n", gpioName );
        return;
    }
    if ( fgets( oldCfgStr, sizeof( oldCfgStr ), fs ) == NULL )
    {
        LogError( "fgets: error reading '%s'\n", gpioName );
        fclose( fs );
        return;
    }
    fclose( fs );

    len = snprintf( testCfgStr, sizeof( testCfgStr ), "%d\t%s", pin, cfgStr );

    if ( strncmp( oldCfgStr, testCfgStr, len ) != 0 )
    {
        // The pin isn't configured the way we want it, so reconfigure

        if (( fs = fopen( gpioName, "w" )) == NULL )
        {
            LogError( "Unable to open '%s' for writing\n", gpioName );
            return;
        }

        fputs( cfgStr, fs );
        fclose( fs );
    }

} // ConfigGpioPin

//***************************************************************************
/**
*   Retrieves the file descriptor currently being logged.
*/

int GetSerialLogFileDescriptor( void )
{
    return gPortFd;

} // GetSerialLogFileDescriptor

//***************************************************************************
/**
*   Starts a logging thread which will log all data received on the 
*   indicated serial port.
*/

void StartSerialLog( speed_t speed, int port )
{
    char            portName[ 20 ];
    struct termios  attr;
    int             rc;
    pthread_t       readerThreadId;

    LogDebug( "StartSerialLog\n" );

    if ( port > 3 )
    {
        LogError( "StartSerialLog: expecting port to be 0-3, found: %d\n", port );
        return;
    }

    // Configure the GPIO pins

    ConfigGpioPin( gPortInfo[ port ].rxdPin, gPortInfo[ port ].rxdCfg );
    ConfigGpioPin( gPortInfo[ port ].txdPin, gPortInfo[ port ].txdCfg );

    // Configure the serial port

    snprintf( portName, sizeof( portName ), "/dev/ttyS%d", port );

    if (( gPortFd = open( portName, O_RDWR | O_EXCL )) < 0 )
    {
        LogError( "Unable to open serial port '%s': %s\n", portName, strerror( errno ));
        return;
    }

    if ( tcgetattr( gPortFd, &gOrgAttr ) < 0 )
    {
        LogError( "Call to tcgetattr failed: %s\n", strerror( errno ));
        close( gPortFd );
        gPortFd = -1;
        return;
    }

    // Turn off flow control

    if ( tcgetattr( gPortFd, &attr ) < 0 )
    {
        LogError( "Call to tcgetattr failed: %s\n", strerror( errno ));
        close( gPortFd );
        gPortFd = -1;
        return;
    }

    // Set the speed and make it raw. This also sets VMIN to 1 and VTIME to 0
    // and sets 8 bits, no parity.

    cfmakeraw( &attr );

    cfsetospeed( &attr, speed );
    cfsetispeed( &attr, speed );

    attr.c_cflag |= ( CLOCAL | CREAD );

    if ( tcsetattr( gPortFd, TCSANOW, &attr ) < 0 )
    {
        LogError( "Call to tcsetattr failed: %s\n", strerror( errno ));
        close( gPortFd );
        gPortFd = -1;
        return;
    }

    // Create the logging semaphore

    if ( sem_init( &gLogMutex, 0, 1 ) != 0 )
    {
        LogError( "Unable to initialize logging semaphore\n" );
        close( gPortFd );
        gPortFd = -1;
        return;
    }

    // And stick in our logging function

    SetLogFunc( SerialLog );

    // Kick off the serial port reader thread.

    rc = pthread_create( &readerThreadId, NULL, ReaderThread, NULL );
    if ( rc != 0 )
    {
        LogError( "Unable to create ReaderThread: %s\n", strerror( rc ));
        close( gPortFd );
        gPortFd = -1;
        return;
    }

} // StartSerialLog

/***************************************************************************/
/**
*   Stop the serial loggin.
*/

void StopSerialLog( void )
{
    LogDebug( "StopSerialLog\n" );

    if ( gPortFd > 0 )
    {
        if ( tcsetattr( gPortFd, TCSANOW, &gOrgAttr ) < 0 )
        {
            LogError( "Call to tcsetattr failed: %s\n", strerror( errno ));
        }

        close( gPortFd );
        gPortFd = -1;
    }

} // StopSerialLog

/***************************************************************************/
/**
*   Thread which processes the incoming serial data.
*/

void *ReaderThread( void *param )
{
    char        line[ 100 ];
    int         lineLen;
    const char *greenStr = "\033[1;32m";
    const char *resetStr = "\033[0m";

    lineLen = 0;

    LogDebug( "ReaderThread\n" );

    while ( 1 )
    {
        char    ch;
        int     bytesRead;

        if (( bytesRead  = read( gPortFd, &ch, 1 )) < 0 )
        {
            LogError( "Serial port read failed: %s\n", strerror( errno ));
            return NULL;
        }

        if (( ch == '\n' ) || ( lineLen >= ( sizeof( line ) - 1 )))
        {
            line[ lineLen++ ] = '\0';

            sem_wait( &gLogMutex );
            if ( gSerialLogColorize )
            {
                printf( "%s%s%s\n", greenStr, line, resetStr );
            }
            else
            {
                printf( "%s\n", line );
            }
            fflush( stdout );
            sem_post( &gLogMutex );

            lineLen = 0;

            if ( ch == '\n' )
            {
                continue;
            }
        }
        line[ lineLen++ ] = ch;
    }

    return NULL;

} // ReaderThread

/***************************************************************************/
/**
*   Function installed to perform logging.
*/

void SerialLog( int logLevel, const char *fmt, va_list args )
{
    // Use a mutex so that lines from the foreground don't get mixed in
    // with the lines from the serial port

    sem_wait( &gLogMutex );

    DefaultLogFunc( logLevel, fmt, args );

    sem_post( &gLogMutex );

} // SerialLog

