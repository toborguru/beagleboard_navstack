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
*   @file   serial-log.c
*
*   @brief  This file is basically a test program for the Serial logging
*           functionality.
*
****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#include "Log.h"
#include "SerialLog.h"

// ---- Public Variables ----------------------------------------------------
// ---- Private Constants and Types -----------------------------------------

enum
{
    // Options assigned a single character code can use that charater code
    // as a short option.

    OPT_HELP            = 'h',
    OPT_BAUD            = 'b',
    OPT_PORT            = 'p',

    // Options from this point onwards don't have any short option equivalents

    OPT_FIRST_LONG_OPT   = 0x80,

};

// ---- Private Variables ---------------------------------------------------

const char *gCmdStr;

struct option gOption[] =
{
    { "baud",       required_argument,  NULL,                   OPT_BAUD },
    { "port",       required_argument,  NULL,                   OPT_PORT },
    { "no-color",   no_argument,        &gSerialLogColorize,    0 },
    { "verbose",    no_argument,        &gVerbose,              1 },
    { "quiet",      no_argument,        &gQuiet,                1 },
    { "debug",      no_argument,        &gDebug,                1 },
    { "help",       no_argument,        NULL,                   OPT_HELP },
    { NULL }
};

#define TRUE    1
#define FALSE   0

struct
{
    speed_t     speed;
    unsigned    baudRate;
} gBaudTable[] =
{
    { B50,          50 },
    { B75,          75 },
    { B110,        110 },
    { B134,        134 },
    { B150,        150 },
    { B200,        200 },
    { B300,        300 },
    { B600,        600 },
    { B1200,      1200 },
    { B1800,      1800 },
    { B2400,      2400 },
    { B4800,      4800 },
    { B9600,      9600 },
    { B19200,    19200 },
    { B38400,    38400 },
    { B57600,    57600 },
    { B115200,  115200 },
    { B230400,  230400 }
};

#define ARRAY_LEN(x)    ( sizeof( x ) / sizeof( x[ 0 ]))

// ---- Private Function Prototypes -----------------------------------------

static  void Usage( void );

// ---- Functions -----------------------------------------------------------

//***************************************************************************
/**
*   ControlC
*/

void ControlC( int sigNum )
{
    fprintf( stderr, "Shutting down...\n" );

    StopSerialLog();

    // Now do whatever the default handling for the signal would be

    signal( sigNum, SIG_DFL );
    raise( sigNum );

} // ControlC

//***************************************************************************
/**
*   Main entry point
*/

int main( int argc, char **argv )
{
    char                shortOptsStr[ sizeof( gOption ) / sizeof( gOption[ 0 ] ) + 1 ];
    char               *shortOpts = shortOptsStr;
    struct option       *scanOpt;
    int                 opt;
    speed_t             baudSpeed = B0;
    int                 baudRate = 0;
    int                 portNum = -1;

    LogInit( stdout );

    // Figure out the short options from our options structure

    for ( scanOpt = gOption; scanOpt->name != NULL; scanOpt++ ) 
    {
        if (( scanOpt->flag == NULL ) && ( scanOpt->val < OPT_FIRST_LONG_OPT ))
        {
            *shortOpts++ = (char)scanOpt->val;

            if ( scanOpt->has_arg != no_argument )
            {
                *shortOpts++ = ':';
            }
        }
    }
    *shortOpts++ = '\0';

    // Parse the command line options

    while (( opt = getopt_long( argc, argv, shortOptsStr, gOption, NULL )) != -1 )
    {
        switch ( opt )
        {
            case OPT_BAUD:
            {
                int baudIdx;
                int testBaud = atoi( optarg );

                for ( baudIdx = 0; baudIdx < ARRAY_LEN( gBaudTable ); baudIdx++ ) 
                {
                    if ( gBaudTable[ baudIdx ].baudRate == testBaud )
                    {
                        baudSpeed = gBaudTable[ baudIdx ].speed;
                        baudRate  = testBaud;
                        break;
                    }
                }

                if ( baudSpeed == B0 )
                {
                    fprintf( stderr, "Unrecognized baud rate: '%s'\n", optarg );
                    exit( 1 );
                }
                break;
            }

            case OPT_PORT:
            {
                portNum = atoi( optarg );
                if (( portNum < 0 ) || ( portNum > 3 ))
                {
                    fprintf( stderr, "Expecting port number 0-3, found: '%s'\n", optarg );
                    exit( 1 );
                }
                break;
            }

            case 0: 
            {
                // getopt_long returns 0 for entries where flag is non-NULL

                break;
            }

            case '?':
            case OPT_HELP:
            default:
            {
                LogError( "opt:%d\n", opt );
                Usage();
                exit( 1 );
            }
        }
    }
    argc -= optind;
    argv += optind;

    if ( baudSpeed == B0 )
    {
        baudSpeed = B38400;
        baudRate  = 38400;
    }
    if ( portNum < 0 )
    {
        portNum = 2;    // /dev/ttyS2 - aka STUART
    }

    gCmdStr = "";
    if ( argc >= 1 )
    {
        gCmdStr = argv[ 0 ];
    }

    signal( SIGINT, ControlC );
    signal( SIGTERM, ControlC );

    StartSerialLog( baudSpeed, portNum );
    sleep( 1 );

    if ( strcasecmp( gCmdStr, "reset" ) == 0 )
    {
        printf( "Resetting the robostix...\n" );

        system( "./robostix reset pulse" );
    }

    // Stick around logging data on the serial port. The user is
    // expected to hit Control-C to exit.

    printf( "Logging /dev/ttyS%d @ %d baud, press Ctrl-C to quit...\n", portNum, baudRate );

    while ( 1 )
    {
        sleep( 10 );
    }

    return 0;

} // main

//***************************************************************************
/**
*   Usage
*/

void Usage( void )
{
    fprintf( stderr, "Usage: serial-log [options] cmd\n" );
    fprintf( stderr, "Communicate with i2c boot loader\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "Only one of the following commands may be specified\n" );
    fprintf( stderr, "Reset         Resets the robostix\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "The following arguments may be used to modify the behaviour\n" );
    fprintf( stderr, "--baud=baud   Set the baudrate\n" );
    fprintf( stderr, "--port=0-3    Set the port to log\n" );
    fprintf( stderr, "--no-color    Turn of colors in the log\n" );
    fprintf( stderr, "--verbose     Print additional information\n" );
    fprintf( stderr, "--quiet       Don't print anything (except errors)\n" );
    fprintf( stderr, "--help        Prints this information\n" );
}

