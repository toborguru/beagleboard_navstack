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
*   @file   i2c-test.c
*
*   @brief  This file implements a simple test program for verifying the state
*           of the i2c bus.
*
****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <sys/timeb.h>
#include <stdint.h>

#include "Log.h"
#include "SerialLog.h"
#include "svn-version.h"

// ---- Public Variables ----------------------------------------------------
// ---- Private Constants and Types -----------------------------------------

// ---- Private Variables ---------------------------------------------------

int gSerialLog  = 0;

enum
{
    OPT_MEM_DEFAULT     = 0,

    // Options assigned a single character code can use that charater code
    // as a short option.

    OPT_BASE_DECIMAL    = 'd',
    OPT_BASE_HEX        = 'x',

    // Options from this point onwards don't have any short option equivalents

    OPT_FIRST_LONG_OPT   = 0x80,

    OPT_VERSION,

    OPT_HELP,
};

struct option gOption[] =
{
    { "version",    no_argument,        NULL,       OPT_VERSION },
    { "verbose",    no_argument,        &gVerbose,  1 },
    { "debug",      no_argument,        &gDebug,    1 },
    { "help",       no_argument,        NULL,       OPT_HELP },
    { NULL }
};

#define TRUE    1
#define FALSE   0

#define MAP_SIZE    4096
#define MAP_MASK    ( MAP_SIZE - 1 )

#define IBMR    0x40301680
#define IBMR_SCL    ( 1 << 1 )
#define IBMR_SDA    ( 1 << 0 )

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
    int                 fd;
    void               *map;
    uint32_t            ibmr_addr = IBMR;
    uint32_t           *ibmr;
    uint32_t            prev_ibmr_val;
    fd_set              readMask;
    int                 logFd;

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
            case 0: 
            {
                // getopt_long returns 0 for entries where flag is non-NULL

                break;
            }

            case OPT_VERSION:
            {   
                printf( "i2c-io: SVN Revision: %d\n", SVN_REVISION );
                exit( 0 );
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

    fd = open( "/dev/mem", O_RDWR | O_SYNC );
    if ( fd < 0 )
    {
        perror( "Unable to open /dev/mem" );
        exit( 1 );
    }

    map = mmap( 0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, ibmr_addr & ~MAP_MASK );
    if ( map == (void *)-1 )
    {
        perror( "mmap() failed" );
        exit( 1 );
    }

    ibmr = (uint32_t *)( (uint8_t *)map + ( ibmr_addr & MAP_MASK ));


    signal( SIGINT, ControlC );

    StartSerialLog( B38400, SERIAL_LOG_STUART );

    prev_ibmr_val = -1;

    // Put stdin & stdout in unbuffered mode.

    setbuf( stdin, NULL );
    setbuf( stdout, NULL );

    // Put stdin in raw mode (i.e. turn off canonical mode). Canonical mode
    // causes the driver to wait for the RETURN character so that line editing
    // can take place. We also want to turn off ECHO.

    {
        struct termios tio;

        if ( tcgetattr( fileno( stdin ), &tio ) < 0 )
        {
            fprintf( stderr, "Unable to retrieve terminal settings: %s\n", strerror( errno ));
            exit( 5 );
        }

        tio.c_lflag &= ~( ICANON | ECHO );
        tio.c_cc[VTIME] = 0;
        tio.c_cc[VMIN] = 1;

        if ( tcsetattr( fileno( stdin ), TCSANOW, &tio ) < 0 )
        {
            fprintf( stderr, "Unable to update terminal settings: %s\n", strerror( errno ));
            exit( 6 );
        }
    }


    logFd = GetSerialLogFileDescriptor();

    while ( 1 )
    {
        uint32_t        ibmr_val = *ibmr;
        struct timeval  timeout;

        timeout.tv_sec = 0;
        timeout.tv_usec = 50 * 1000;    // 50 msec

        FD_ZERO( &readMask );
        FD_SET( fileno( stdin ), &readMask );

        if ( select( fileno( stdin ) + 1, &readMask, NULL, NULL, &timeout ) == 1 )
        {
            char ch = fgetc( stdin );

            //Log( "gumstix: Read: '%c'\n", ch );

            write( logFd, &ch, 1 );

            prev_ibmr_val = -1;
        }

        if ( ibmr_val != prev_ibmr_val )
        {
            Log( "gumstix: SCL: %d SDA: %d\n", ( ibmr_val & IBMR_SCL ) != 0, ( ibmr_val & IBMR_SDA ) != 0 );
            prev_ibmr_val = ibmr_val;
        }
    }

    StopSerialLog();

    munmap( 0, MAP_SIZE );
    close( fd );
    return 0;

} // main

//***************************************************************************
/**
*   Usage
*/

void Usage( void )
{
    fprintf( stderr, "Usage: i2c-test [options]\n" );
    fprintf( stderr, "Test status of i2c pins\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "--version             Print the version information for this program\n" );
    fprintf( stderr, "--verbose             Print additional information\n" );
    fprintf( stderr, "--help                Prints this information\n" );
}

