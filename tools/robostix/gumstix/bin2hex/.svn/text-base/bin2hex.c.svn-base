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
*   @file   bin2hex.c
*
*   @brief  Program which converts a binary file into a hex file.
*
****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <errno.h>
#include <stdint.h>

// ---- Public Variables ----------------------------------------------------
// ---- Private Constants and Types -----------------------------------------
// ---- Private Variables ---------------------------------------------------

enum
{
    // Options assigned a single character code can use that charater code
    // as a short option.

    OPT_ADDR            = 'a',
    OPT_HELP            = 'h',

    // Options from this point onwards don't have any short option equivalents

    OPT_FIRST_LONG_OPT   = 0x80,

};

struct option gOption[] =
{
    { "addr",       required_argument,  NULL,       OPT_ADDR },
    { "help",       no_argument,        NULL,       OPT_HELP },
    { NULL }
};

static  long    gAddr = 0;

// ---- Private Function Prototypes -----------------------------------------

void Usage( void );

// ---- Functions -----------------------------------------------------------

//***************************************************************************
/**
*   Main program
*/

int main( int argc, char **argv )
{
    char                shortOptsStr[ sizeof( gOption ) / sizeof( gOption[ 0 ] ) + 1 ];
    char               *shortOpts = shortOptsStr;
    struct option       *scanOpt;
    int                 opt;

    char   *binFileName;
    char   *hexFileName;
    FILE   *binFs;
    FILE   *hexFs;
    char    buf[ 16 ];
    size_t  numBytes;

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


    if (( argc < 1 ) || ( argc > 2 ))
    {
        Usage();
        exit( 1 );
    }

    binFileName = argv[ 0 ];

    if ( strcmp( binFileName, "-" ) == 0 )
    {
        binFileName = "stdin";
        hexFileName = "stdout";

        binFs = stdin;
        hexFs = stdout;
    }
    else
    {
        hexFileName = argv[ 1 ];
    
        if (( binFs = fopen( binFileName, "rb" )) == NULL )
        {
            fprintf( stderr, "Unable to open '%s' for reading: %s\n", binFileName, strerror( errno ));
            exit( 1 );
        }
        if (( hexFs = fopen( hexFileName, "wb" )) == NULL )
        {
            fprintf( stderr, "Unable to open '%s' for writing: %s\n", hexFileName, strerror( errno ));
            exit( 2 );
        }
    }

    while (( numBytes = fread( buf, 1, sizeof( buf ), binFs )) > 0 )
    {
        uint8_t checksum = 0;
        int     i;

        fprintf( hexFs, ":%02.2X%04.4X00", numBytes, gAddr );

        checksum = numBytes + (( gAddr >> 8 ) & 0xff ) + ( gAddr & 0xff );

        for ( i = 0; i < numBytes; i++ ) 
        {
            checksum += buf[ i ];
            fprintf( hexFs, "%02.2X", buf[ i ] );
        }

        checksum = - checksum;
        fprintf( hexFs, "%02.2X\r\n", checksum );

        gAddr += numBytes;
    }
    fprintf( hexFs, ":00000001FF\r\n" );

    fflush( hexFs );

    exit( 0 );
    return 0;

} // main

//***************************************************************************
/**
*   Usage
*/

void Usage( void )
{
    fprintf( stderr, "Usage: bin2hex [options] file.bin file.hex\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "Convert a binary to intel hex format\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "The options may be specified:\n" );
    fprintf( stderr, "--addr=addr   Starting addresses to use\n" );
    fprintf( stderr, "--help        Prints this information\n" );
}

