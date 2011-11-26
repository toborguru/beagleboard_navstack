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
*   @file   roboargs.c
*
*   @brief  Program which packs up command line arguments for storing
*           into EEPROM on the robostix.
*
****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

// ---- Public Variables ----------------------------------------------------
// ---- Private Constants and Types -----------------------------------------
// ---- Private Variables ---------------------------------------------------

enum
{
    // Options assigned a single character code can use that charater code
    // as a short option.

    OPT_ADDR            = 'a',
    OPT_HELP            = 'h',
    OPT_VERBOSE         = 'v',

    // Options from this point onwards don't have any short option equivalents

    OPT_FIRST_LONG_OPT   = 0x80,

};

struct option gOption[] =
{
    { "addr",       required_argument,  NULL,       OPT_ADDR },
    { "help",       no_argument,        NULL,       OPT_HELP },
    { "verbose",    no_argument,        NULL,       OPT_VERBOSE },
    { NULL }
};

static  long    gAddr = 0;
static  int     gVerbose = 0;

// ---- Private Function Prototypes -----------------------------------------

void Usage( void );

// ---- Functions -----------------------------------------------------------

//***************************************************************************
/**
*   Main program
*/

int main( int argc, char **argv )
{
    char            shortOptsStr[ sizeof( gOption ) / sizeof( gOption[ 0 ] ) + 1 ];
    char           *shortOpts = shortOptsStr;
    struct option  *scanOpt;
    int             opt;

    int             i;
    int             argSize;
    int             numBytes;
    uint8_t         offset;
    uint8_t        *offsetPtr;
    uint8_t        *dataPtr;
    void           *buf;
    int             bytesRemaining;

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
            case OPT_ADDR:
            {
                gAddr = strtol( optarg, NULL, 0 );
                break;
            }

            case OPT_VERBOSE:
            {
                gVerbose = 1;
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
                Usage();
                exit( 1 );
            }
        }
    }
    argc -= optind;
    argv += optind;

    // Run through the arguments and add up the sizes.

    argSize = 2;    // 1 for number of arguments and 1 for NULL terminator

    for ( i = 0; i < argc; i++ ) 
    {
        if ( gVerbose )
        {
            printf( "argv[ %d ] = '%s'\n", i, argv[ i ] );
        }
        argSize += 2;   // Add in size of pointer.
        argSize += strlen( argv[ i ] ) + 1;
    }

    if ( gVerbose )
    {
        printf( "%d arguments, size = %d\n", argc, argSize );
    }

    // Allocate a hunk of memory that will be what's stored in EEPROM

    if (( buf = malloc( argSize )) == NULL )
    {
        fprintf( stderr, "Unable to allocate buffer of %d bytes\n", argSize );
        exit( 3 );
    }

    // And initialize the memory we just allocated.

    offsetPtr = buf;
    dataPtr = buf;

    offset = ( argc + 2 );

    *offsetPtr++ = argc;

    for ( i = 0; i < argc; i++ ) 
    {
        *offsetPtr++ = offset;

        strcpy( &dataPtr[ offset ], argv[ i ]);

        offset += strlen( argv[ i ]);
        offset++;
    }
    *offsetPtr = 0;

    // Now create the hex file.

    dataPtr = buf;
    bytesRemaining = argSize;

    while ( bytesRemaining > 0 )
    {
        uint8_t checksum = 0;

        numBytes = bytesRemaining;
        if ( numBytes > 16 )
        {
            numBytes = 16;
        }

        printf( ":%02.2X%04.4X00", numBytes, gAddr );

        checksum = numBytes + (( gAddr >> 8 ) & 0xff ) + ( gAddr & 0xff );

        for ( i = 0; i < numBytes; i++ ) 
        {
            checksum += dataPtr[ i ];
            printf( "%02.2X", dataPtr[ i ] );
        }

        checksum = - checksum;
        printf( "%02.2X\r\n", checksum );

        gAddr += numBytes;
        dataPtr += numBytes;

        bytesRemaining -= numBytes;
    }
    printf( ":00000001FF\r\n" );

    exit( 0 );
    return 0;

} // main

//***************************************************************************
/**
*   Usage
*/

void Usage( void )
{
    fprintf( stderr, "Usage: roboargs [options] arguments\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "Generate an Args compatible hex file.\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "The options may be specified:\n" );
    fprintf( stderr, "--addr=addr   Starting addresses to use\n" );
    fprintf( stderr, "--help        Prints this information\n" );
}

