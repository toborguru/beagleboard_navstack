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
*   @file   i2c-io.c
*
*   @brief  This file implements commands for performing I/O on the robostix
*           via the i2c bus.
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
#include <sys/timeb.h>

#include "AvrInfo.h"
#include "i2c-dev.h"
#include "i2c-api.h"
#include "i2c-io-api.h"
#include "BootLoader-api.h"
#include "Log.h"
#include "svn-version.h"

// ---- Public Variables ----------------------------------------------------
// ---- Private Constants and Types -----------------------------------------

// ---- Private Variables ---------------------------------------------------

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

enum
{
    CMD_DEFAULT,

    CMD_INFO,
    CMD_GET,
    CMD_SET,
    CMD_GET_DIR,
    CMD_SET_DIR,
    CMD_READ_REG,
    CMD_WRITE_REG,
};

struct 
{
    int         cmd;
    const char *cmdStr;

} gCmdMap[] =
{
    { CMD_INFO,         "Info" },
    { CMD_GET,          "Get" },
    { CMD_SET,          "Set" },
    { CMD_GET_DIR,      "GetDir" },
    { CMD_SET_DIR,      "SetDir" },
    { CMD_READ_REG,     "ReadReg" },
    { CMD_WRITE_REG,    "WriteReg" },
    { CMD_READ_REG,     "rr" },
    { CMD_WRITE_REG,    "wr" },
};

int gNumCmds = sizeof( gCmdMap ) / sizeof( gCmdMap[ 0 ]);

#define REG_IO8(   name, offset )   { name, 0, offset + 0x20 }
#define REG_IO16(  name, offset )   { name, 1, offset + 0x20 }
#define REG_MEM8(  name, offset )   { name, 0, offset }
#define REG_MEM16( name, offset )   { name, 1, offset }

// The following register map is only valid for the ATMega 128

typedef struct
{
    const char *regStr;
    uint8_t     is16Bit;
    uint8_t     reg;

} RegMap_t;

typedef struct
{
    uint16_t    signature;
    uint16_t    numRegs;
    RegMap_t   *regMap;

} Regs_t;

RegMap_t gRegMapATMega8[] =
{
    REG_IO8(  "TWBR",   0x00 ),
    REG_IO8(  "TWSR",	0x01 ),
    REG_IO8(  "TWAR",	0x02 ),
    REG_IO8(  "TWDR",	0x03 ),
    REG_IO16( "ADC", 	0x04 ),
    REG_IO8(  "ADCL",	0x04 ),
    REG_IO8(  "ADCH",	0x05 ),
    REG_IO8(  "ADCSRA",	0x06 ),
    REG_IO8(  "ADMUX",	0x07 ),
    REG_IO8(  "ACSR",	0x08 ),
    REG_IO8(  "UBRRL",	0x09 ),
    REG_IO8(  "UCSRB",	0x0A ),
    REG_IO8(  "UCSRA",	0x0B ),
    REG_IO8(  "UDR",	0x0C ),
    REG_IO8(  "SPCR",	0x0D ),
    REG_IO8(  "SPSR",	0x0E ),
    REG_IO8(  "SPDR",	0x0F ),
    REG_IO8(  "PIND",	0x10 ),
    REG_IO8(  "DDRD",	0x11 ),
    REG_IO8(  "PORTD",	0x12 ),
    REG_IO8(  "PINC",	0x13 ),
    REG_IO8(  "DDRC",	0x14 ),
    REG_IO8(  "PORTC",	0x15 ),
    REG_IO8(  "PINB",	0x16 ),
    REG_IO8(  "DDRB",	0x17 ),
    REG_IO8(  "PORTB",	0x18 ),
    REG_IO8(  "EECR",	0x1C ),
    REG_IO8(  "EEDR",	0x1D ),
    REG_IO16( "EEAR",	0x1E ),
    REG_IO8(  "EEARL",	0x1E ),
    REG_IO8(  "EEARH",	0x1F ),
    REG_IO8(  "UCSRC",	0x20 ),
    REG_IO8(  "UBRRH",	0x20 ),
    REG_IO8(  "WDTCR",	0x21 ),
    REG_IO8(  "ASSR",	0x22 ),
    REG_IO8(  "OCR2",	0x23 ),
    REG_IO8(  "TCNT2",	0x24 ),
    REG_IO8(  "TCCR2",	0x25 ),
    REG_IO16( "ICR1",	0x26 ),
    REG_IO8(  "ICR1L",	0x26 ),
    REG_IO8(  "ICR1H",	0x27 ),
    REG_IO16( "OCR1B",	0x28 ),
    REG_IO8(  "OCR1BL",	0x28 ),
    REG_IO8(  "OCR1BH",	0x29 ), 
    REG_IO16( "OCR1A",	0x2A ),
    REG_IO8(  "OCR1AL",	0x2A ),
    REG_IO8(  "OCR1AH",	0x2B ),
    REG_IO16( "TCNT1",	0x2C ),
    REG_IO8(  "TCNT1L",	0x2C ),
    REG_IO8(  "TCNT1H",	0x2D ),
    REG_IO8(  "TCCR1B",	0x2E ),
    REG_IO8(  "TCCR1A",	0x2F ),
    REG_IO8(  "SFIOR",	0x30 ),
    REG_IO8(  "OSCCAL",	0x31 ),
    REG_IO8(  "TCNT0",	0x32 ),
    REG_IO8(  "TCCR0",	0x33 ),
    REG_IO8(  "MCUCSR",	0x34 ),
    REG_IO8(  "MCUCR",	0x35 ),
    REG_IO8(  "TWCR",	0x36 ),
    REG_IO8(  "SPMCR",	0x37 ),
    REG_IO8(  "TIFR",	0x38 ),
    REG_IO8(  "TIMSK",	0x39 ),
    REG_IO8(  "GIFR",	0x3A ),
    REG_IO8(  "GICR",	0x3B )
};

RegMap_t gRegMapATMega128[] =
{
    REG_IO8( "PINF",      0x00 ),
    REG_IO8( "PINE",      0x01 ),
    REG_IO8( "DDRE",      0x02 ),
    REG_IO8( "PORTE",     0x03 ),
    REG_IO8( "ADCL",      0x04 ),
    REG_IO8( "ADCH",      0x05 ),
    REG_IO8( "ADCSR",     0x06 ),
    REG_IO8( "ADMUX",     0x07 ),
    REG_IO8( "ACSR",      0x08 ),
    REG_IO8( "UBRR0L",    0x09 ),
    REG_IO8( "UCSR0B",    0x0A ),
    REG_IO8( "UCSR0A",    0x0B ),
    REG_IO8( "UDR0",      0x0C ),
    REG_IO8( "SPCR",      0x0D ),
    REG_IO8( "SPSR",      0x0E ),
    REG_IO8( "SPDR",      0x0F ),
    REG_IO8( "PIND",      0x10 ),
    REG_IO8( "DDRD",      0x11 ),
    REG_IO8( "PORTD",     0x12 ),
    REG_IO8( "PINC",      0x13 ),
    REG_IO8( "DDRC",      0x14 ),
    REG_IO8( "PORTC",     0x15 ),
    REG_IO8( "PINB",      0x16 ),
    REG_IO8( "DDRB",      0x17 ),
    REG_IO8( "PORTB",     0x18 ),
    REG_IO8( "PINA",      0x19 ),
    REG_IO8( "DDRA",      0x1A ),
    REG_IO8( "PORTA",     0x1B ),
    REG_IO8( "SFIOR",     0x20 ),
    REG_IO8( "WDTCR",     0x21 ),
    REG_IO8( "OCDR",      0x22 ),
    REG_IO8( "OCR2",      0x23 ),
    REG_IO8( "TCNT2",     0x24 ),
    REG_IO8( "TCCR2",     0x25 ),
    REG_IO8( "ICR1L",     0x26 ),
    REG_IO8( "ICR1H",     0x27 ),
    REG_IO8( "OCR1BL",    0x28 ),
    REG_IO8( "OCR1BH",    0x29 ),
    REG_IO8( "OCR1AL",    0x2A ),
    REG_IO8( "OCR1AH",    0x2B ),
    REG_IO8( "TCNT1L",    0x2C ),
    REG_IO8( "TCNT1H",    0x2D ),
    REG_IO8( "TCCR1B",    0x2E ),
    REG_IO8( "TCCR1A",    0x2F ),
    REG_IO8( "ASSR",      0x30 ),
    REG_IO8( "OCR0",      0x31 ),
    REG_IO8( "TCNT0",     0x32 ),
    REG_IO8( "TCCR0",     0x33 ),
    REG_IO8( "MCUSR",     0x34 ),
    REG_IO8( "MCUCR",     0x35 ),
    REG_IO8( "TIFR",      0x36 ),
    REG_IO8( "TIMSK",     0x37 ),
    REG_IO8( "EIFR",      0x38 ),
    REG_IO8( "EIMSK",     0x39 ),
    REG_IO8( "EICRB",     0x3A ),
    REG_IO8( "RAMPZ",     0x3B ),
    REG_IO8( "XDIV",      0x3C ),

    REG_IO16( "ADC",      0x04 ),
    REG_IO16( "ICR1",     0x26 ),
    REG_IO16( "OCR1B",    0x28 ),
    REG_IO16( "OCR1A",    0x2A ),
    REG_IO16( "TCNT1",    0x2C ),
                          
    REG_MEM8( "DDRF",     0x61 ),
    REG_MEM8( "PORTF",    0x62 ),
    REG_MEM8( "PING",     0x63 ),
    REG_MEM8( "DDRG",     0x64 ),
    REG_MEM8( "PORTG",    0x65 ),
    REG_MEM8( "SPMCR",    0x68 ),
    REG_MEM8( "EICRA",    0x6A ),
    REG_MEM8( "XMCRB",    0x6C ),
    REG_MEM8( "XMCRA",    0x6D ),
    REG_MEM8( "OSCCAL",   0x6F ),
    REG_MEM8( "TWBR",     0x70 ),
    REG_MEM8( "TWSR",     0x71 ),
    REG_MEM8( "TWAR",     0x72 ),
    REG_MEM8( "TWDR",     0x73 ),
    REG_MEM8( "TWCR",     0x74 ),
    REG_MEM8( "OCR1CL",   0x78 ),
    REG_MEM8( "OCR1CH",   0x79 ),
    REG_MEM8( "TCCR1C",   0x7A ),
    REG_MEM8( "ETIFR",    0x7C ),
    REG_MEM8( "ETIMSK",   0x7D ),
    REG_MEM8( "ICR3L",    0x80 ),
    REG_MEM8( "ICR3H",    0x81 ),
    REG_MEM8( "OCR3CL",   0x82 ),
    REG_MEM8( "OCR3CH",   0x83 ),
    REG_MEM8( "OCR3BL",   0x84 ),
    REG_MEM8( "OCR3BH",   0x85 ),
    REG_MEM8( "OCR3AL",   0x86 ),
    REG_MEM8( "OCR3AH",   0x87 ),
    REG_MEM8( "TCNT3L",   0x88 ),
    REG_MEM8( "TCNT3H",   0x89 ),
    REG_MEM8( "TCCR3B",   0x8A ),
    REG_MEM8( "TCCR3A",   0x8B ),
    REG_MEM8( "TCCR3C",   0x8C ),
    REG_MEM8( "UBRR0H",   0x90 ),
    REG_MEM8( "UCSR0C",   0x95 ),
    REG_MEM8( "UBRR1H",   0x98 ),
    REG_MEM8( "UBRR1L",   0x99 ),
    REG_MEM8( "UCSR1B",   0x9A ),
    REG_MEM8( "UCSR1A",   0x9B ),
    REG_MEM8( "UDR1",     0x9C ),
    REG_MEM8( "UCSR1C",   0x9D ),

    REG_MEM16( "OCR1C",   0x78 ),
    REG_MEM16( "ICR3",    0x80 ),
    REG_MEM16( "OCR3C",   0x82 ),
    REG_MEM16( "OCR3B",   0x84 ),
    REG_MEM16( "OCR3A",   0x86 ),
    REG_MEM16( "TCNT3",   0x88 ),
};

#define ARRAY_LEN( x )  ( sizeof( x ) / sizeof( x[0] ))

Regs_t  gRegs[] =
{
    { AVR_ATMEGA_8_SIGNATURE,       ARRAY_LEN( gRegMapATMega8 ),    gRegMapATMega8 },
    { AVR_ATMEGA_128_SIGNATURE,     ARRAY_LEN( gRegMapATMega128 ),  gRegMapATMega128 },
};

int gI2cAddr    = -1;

const char *gCmdStr;
int         gCmd        = CMD_DEFAULT;
const char *gPortPinStr = NULL;
const char *gValStr     = NULL;
const char *gRegStr     = NULL;
int         gBase       = OPT_BASE_HEX;

struct option gOption[] =
{
    { "decimal",    no_argument,        NULL,       OPT_BASE_DECIMAL },
    { "hex",        no_argument,        NULL,       OPT_BASE_HEX },
    { "version",    no_argument,        NULL,       OPT_VERSION },
    { "verbose",    no_argument,        &gVerbose,  1 },
    { "debug",      no_argument,        &gDebug,    1 },
    { "help",       no_argument,        NULL,       OPT_HELP },
    { NULL }
};

#define TRUE    1
#define FALSE   0

typedef enum
{
    NoADC,
    AllowADC,

} AllowADC_t;

#define ADC_PORT    8

// ---- Private Function Prototypes -----------------------------------------

static  int  ParseByte( const char *byteStr, uint8_t *byte );
static  int  ParseUInt16( const char *str, uint16_t *val16 );
static  int  ParsePortPinStr( const char *portPinStr, uint8_t *portNum, uint8_t *pin, AllowADC_t allowAdc );
static  int  ParseReg( int i2cDev, const char *regStr, RegMap_t *regFound );

static  void ProcessInfoCommand( int i2cDev );
static  void ProcessGetCommand( int i2cDev, const char *portPinStr );
static  void ProcessSetCommand( int i2cDev, const char *portPinStr, const char *valStr );
static  void ProcessGetDirCommand( int i2cDev, const char *portPinStr );
static  void ProcessSetDirCommand( int i2cDev, const char *portPinStr, const char *valStr );
static  void ProcessReadRegCommand( int i2cDev, const char *regStr );
static  void ProcessWriteRegCommand( int i2cDev, const char *regStr, const char *valStr );
static  void Usage( void );

// ---- Functions -----------------------------------------------------------

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
    const char         *i2cDevName = "/dev/i2c-0";
    int                 i2cDev;
    int                 cmdIdx;

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

            case OPT_BASE_DECIMAL:
            case OPT_BASE_HEX:
            {
                gBase = opt;
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

    // Verify that an i2c-address was specified

    if ( argc < 1 )
    {
        LogError( "Must specify an i2c address\n\n" );
        Usage();
        exit( 1 );
    }
    gI2cAddr = strtol( argv[ 0 ], NULL, 0 );
    if (( gI2cAddr <= 0 ) || ( gI2cAddr > 127 ))
    {
        LogError( "Expecting i2c address in the range of 1-127, Found: %d\n", gI2cAddr );
        Usage();
        exit( 1 );
    }

    // Verify that a command has been specified

    if ( argc < 2 )
    {
        LogError( "Must specify a command\n" );
        Usage();
        exit( 1 );
    }
    gCmdStr = argv[ 1 ];
    for ( cmdIdx = 0; cmdIdx < gNumCmds; cmdIdx++ ) 
    {
        if ( strcasecmp( gCmdStr, gCmdMap[ cmdIdx ].cmdStr ) == 0 )
        {
            gCmd = gCmdMap[ cmdIdx ].cmd;
            break;
        }
    }
    if ( gCmd == CMD_DEFAULT )
    {
        LogError( "Unrecognized command '%s'\n", gCmdStr );
        exit( 1 );
    }

    // Process command specific arguments

    if ( gCmd == CMD_INFO )
    {
        if ( argc != 2 )
        {
            LogError( "Unexpected extra parameters\n" );
            Usage();
            exit( 1 );
        }
    }
    else
    if (( gCmd == CMD_GET )
    ||  ( gCmd == CMD_GET_DIR ))
    {
        if ( argc < 3 )
        {
            LogError( "Expecting port.pin\n" );
            Usage();
            exit( 1 );
        }

        gPortPinStr = argv[ 2 ];
    }
    else
    if (( gCmd == CMD_SET )
    ||  ( gCmd == CMD_SET_DIR ))
    {
        if ( argc < 4 )
        {
            LogError( "port.pin followed by value\n" );
            Usage();
            exit( 1 );
        }
        gPortPinStr = argv[ 2 ];
        gValStr     = argv[ 3 ];
    }
    else
    if ( gCmd == CMD_READ_REG  )
    {
        if ( argc < 3 )
        {
            LogError( "Expecting register index\n" );
            Usage();
            exit( 1 );
        }
        gRegStr = argv[ 2 ];
    }
    else
    if ( gCmd == CMD_WRITE_REG  )
    {
        if ( argc < 4 )
        {
            LogError( "Expecting register index and value\n" );
            Usage();
            exit( 1 );
        }
        gRegStr = argv[ 2 ];
        gValStr = argv[ 3 ];
    }

    if ( gDebug )
    {
        Log( "i2cAddr:0x%02x Cmd: %s (%d)", gI2cAddr, gCmdStr, gCmd );
        if ( gPortPinStr != NULL )
        {
            Log( " Pin: %s", gPortPinStr );
        }
        if ( gValStr != NULL )
        {
            Log( " Val: %s", gValStr );
        }
        Log( "\n" );
    }

    // Try to open the i2c device

    if (( i2cDev = open( i2cDevName, O_RDWR )) < 0 )
    {
        LogError( "Error  opening '%s': %s\n", i2cDevName, strerror( errno ));
        exit( 1 );
    }

    // Indicate which slave we wish to speak to

    I2cSetSlaveAddress( i2cDev, gI2cAddr, I2C_USE_CRC );

    switch ( gCmd )
    {
        case CMD_INFO:
        {
            ProcessInfoCommand( i2cDev );
            break;
        }

        case CMD_GET:
        {
            ProcessGetCommand( i2cDev, gPortPinStr );
            break;
        }

        case CMD_SET:
        {
            ProcessSetCommand( i2cDev, gPortPinStr, gValStr );
            break;
        }

        case CMD_GET_DIR:
        {
            ProcessGetDirCommand( i2cDev, gPortPinStr );
            break;
        }

        case CMD_SET_DIR:
        {
            ProcessSetDirCommand( i2cDev, gPortPinStr, gValStr );
            break;
        }

        case CMD_READ_REG:
        {
            ProcessReadRegCommand( i2cDev, gRegStr );
            break;
        }

        case CMD_WRITE_REG:
        {
            ProcessWriteRegCommand( i2cDev, gRegStr, gValStr );
            break;
        }
    }

    close( i2cDev );

    return 0;

} // main

//***************************************************************************
/**
*   Parse a string looking for a single byte.
*/

int ParseByte( const char *byteStr, uint8_t *byte )
{
    char       *endPtr;

    *byte = (uint8_t)strtol( byteStr, &endPtr, 0 );

    if ( *endPtr != '\0' )
    {
        LogError( "Expecting numeric value, found '%s'\n", byteStr );
        return FALSE;
    }

    return TRUE;

} // ParseByte

//***************************************************************************
/**
*   Parse a string looking for a 16 bit value.
*/

int ParseUInt16( const char *str, uint16_t *val16 )
{
    char       *endPtr;

    *val16 = (uint16_t)strtol( str, &endPtr, 0 );

    if ( *endPtr != '\0' )
    {
        LogError( "Expecting numeric value, found '%s'\n", str );
        return FALSE;
    }

    return TRUE;

} // ParseUInt16

//***************************************************************************
/**
*   Parse a string looking for a port.pin
*/

int ParsePortPinStr( const char *portPinStr, uint8_t *portNum, uint8_t *pin, AllowADC_t allowAdc )
{
    const char *s;
    char       *endPtr;

    *portNum = 0;
    *pin = 0;

    if (( s = strchr( portPinStr, '.' )) == NULL )
    {
        LogError( "Expecting port.pin, found: '%s'\n", portPinStr );
        return FALSE;
    }
    if (( s - portPinStr ) > 3 )
    {
        // ADC is currently the longest port allowed.

        LogError( "Invalid Port: '%s'\n", portPinStr );
        return FALSE;
    }
    s++;

    *pin = (uint8_t)strtol( s, &endPtr, 10 );

    if ( *endPtr != '\0' )
    {
        LogError( "Expecting numeric pin, found '%s'\n", s );
        return FALSE;
    }

    if ( portPinStr[ 1 ] == '.' )
    {
        if ((( portPinStr[ 0 ] >= 'A' ) && ( portPinStr[ 0 ] <= 'G' ))
        ||  (( portPinStr[ 0 ] >= 'a' ) && ( portPinStr[ 0 ] <= 'g' )))
        {
            if (( portPinStr[ 0 ] >= 'A' ) && ( portPinStr[ 0 ] <= 'G' ))
            {
                *portNum = portPinStr[ 0 ] - 'A';
            }
            else
            {
                *portNum = portPinStr[ 0 ] - 'a';
            }

            if ( *pin <= 7 )
            {
                return TRUE;
            }

            LogError( "Expecting pin to be 0-7, found: %d\n", *pin );
            return FALSE;
        }

        LogError( "Expecting port to be A-G, found: '%c'\n", portPinStr[ 0 ]);
        return FALSE;
    }
    else
    if (( allowAdc == AllowADC ) && ( strncasecmp( portPinStr, "ADC", 3 ) == 0 ) && ( portPinStr[ 3 ] == '.' ))
    {
        *portNum = ADC_PORT;

        if ( *pin <= 31 )
        {
            return TRUE;
        }

        LogError( "Expecting ADC mux to be in range 0-31, found: %d\n", *pin );
        return FALSE;
    }
    LogError( "Unrecognized port: '%s'\n", portPinStr );
    return FALSE;

} // ParsePortPinStr

//***************************************************************************
/**
*   Parses a register name or number.
*/

int ParseReg( int i2cDev, const char *regStr, RegMap_t *regFound )
{
    char               *endPtr;
    int                 i;
    uint8_t             reg;
    BootLoaderInfo_t    bootInfo;
    RegMap_t           *regList = NULL;
    int                 numRegs = 0;

    static  char    regNumStr[ 10 ];

    // Fetch the AVR BootLoader info, so that we can detemine which type of
    // AVR we have.

    if ( !BootLoaderGetInfo( i2cDev, &bootInfo ))
    {
        LogError( "Unable to retrieve BootLoader information from i2c address 0x%02x\n", gI2cAddr );
        return FALSE;
    }

    for ( i = 0; i < ARRAY_LEN( gRegs ); i++ ) 
    {
        if ( bootInfo.partNumber == gRegs[ i ].signature )
        {
            regList = gRegs[ i ].regMap;
            numRegs = gRegs[ i ].numRegs;
        }
    }
    if ( regList == NULL )
    {
        LogError( "Unrecognized signature: 0x%04x\n", bootInfo.partNumber );
        return FALSE;
    }

    // Check to see if it's a register that we know about

    for ( i = 0; i < numRegs; i++ ) 
    {
        if ( strcasecmp( regStr, regList[ i ].regStr ) == 0 )
        {
            *regFound = regList[ i ];
            return TRUE;
        }
    }

    // Not someting we recognize, see if a number was entered.

    reg = (uint8_t)strtol( regStr, &endPtr, 0 );

    if ( *endPtr != '\0' )
    {
        LogError( "Unrecognized register, found '%s'\n", regStr );
        return FALSE;
    }

    snprintf( regNumStr, sizeof( regNumStr ), "0x%02x", reg );

    regFound->reg = reg;
    regFound->regStr = regNumStr;
    regFound->is16Bit = 0;

    return TRUE;


} // ParseReg

//***************************************************************************
/**
*   Retrieves information about the i2c-io code installed on the robostix.
*/

void ProcessInfoCommand( int i2cDev )
{
    I2C_IO_Info_t   info;

    if ( !I2C_IO_GetInfo( i2cDev, &info ))
    {
        LogError( "Unable to retrieve information from i2c address 0x%02x\n", gI2cAddr );
        exit( 1 );
    }

    Log( "     version: %d\n", info.version );
    Log( "  minVersion: %d\n", info.minVersion );
    Log( "SVN Revision: %d\n", info.svnRevision );

    if ( !I2C_IO_CheckVersion( &info ))
    {
        exit( 1 );
    }

} // ProcessInfoCommand

//***************************************************************************
/**
*   Retrieves a pin from the robostix.
*/

void ProcessGetCommand( int i2cDev, const char *portPinStr )
{
    uint8_t portNum;
    uint8_t pin;
    uint8_t pinVal;

    if ( !ParsePortPinStr( portPinStr, &portNum, &pin, AllowADC ))
    {
        return;
    }

    if ( portNum == ADC_PORT )
    {
        uint16_t    adcVal;

        if ( I2C_IO_GetADC( i2cDev, pin, &adcVal ))
        {
            Log( "%d\n", adcVal );
        }
        else
        {
            LogError( "Failed to retrieve value for ADC.%d\n", pin );
        }
    }
    else
    {
        if ( I2C_IO_GetGPIO( i2cDev, portNum, &pinVal ))
        {
            Log( "%d\n", (( pinVal & ( 1 << pin )) != 0 ));
        }
        else
        {
            LogError( "Failed to retrieve value for %c.%d\n",
                      portNum + 'A', pin );
        }
    }

} // ProcessGetCommand

//***************************************************************************
/**
*   Sets a pin on the robostix.
*/

void ProcessSetCommand( int i2cDev, const char *portPinStr, const char *valStr )
{
    uint8_t portNum;
    uint8_t pin;
    uint8_t pinMask;
    uint8_t pinVal;

    if ( !ParsePortPinStr( portPinStr, &portNum, &pin, NoADC ))
    {
        return;
    }
    pinMask = 1 << pin;

    if (( strcmp( valStr, "0" ) == 0 ) || ( strcasecmp( valStr, "off" ) == 0 ))
    {
        pinVal = 0;
    }
    else
    if (( strcmp( valStr, "1" ) == 0 ) || ( strcasecmp( valStr, "on" ) == 0 ))
    {
        pinVal = pinMask;
    }
    else
    {
        LogError( "Unrecognized pin value '%s'\n", valStr );
        return;
    }

    if ( I2C_IO_SetGPIO( i2cDev, portNum, pinMask, pinVal ))
    {
        LogVerbose( "Port %c.%d set to %d\n", 'A' + portNum, pin, pinVal != 0 );
    }
    else
    {
        LogError( "Failed to set value for %c.%d\n", portNum + 'A', pin );
    }

} // ProcessSetCommand

//***************************************************************************
/**
*   Gets the direction of a pin.
*/

void ProcessGetDirCommand( int i2cDev, const char *portPinStr )
{
    uint8_t portNum;
    uint8_t pin;
    uint8_t pinVal;

    if ( !ParsePortPinStr( portPinStr, &portNum, &pin, NoADC ))
    {
        return;
    }

    if ( I2C_IO_GetGPIODir( i2cDev, portNum, &pinVal ))
    {
        Log( "%s\n", (( pinVal & ( 1 << pin )) != 0 ) ? "out" : "in" );
    }
    else
    {
        LogError( "Failed to retrieve direction for %s.%d\n",
                  portNum + 'A', pin );
    }

} // ProcessGetDirCommand

//***************************************************************************
/**
*   Sets the direction of a pin.
*/

void ProcessSetDirCommand( int i2cDev, const char *portPinStr, const char *valStr )
{
    uint8_t portNum;
    uint8_t pin;
    uint8_t pinMask;
    uint8_t pinVal;

    if ( !ParsePortPinStr( portPinStr, &portNum, &pin, NoADC ))
    {
        return;
    }
    pinMask = 1 << pin;

    if (( strcmp( valStr, "0" ) == 0 ) || ( strcasecmp( valStr, "in" ) == 0 ))
    {
        pinVal = 0;
    }
    else
    if (( strcmp( valStr, "1" ) == 0 ) || ( strcasecmp( valStr, "out" ) == 0 ))
    {
        pinVal = pinMask;
    }
    else
    {
        LogError( "Unrecognized pin value '%s'\n", valStr );
        return;
    }

    if ( I2C_IO_SetGPIODir( i2cDev, portNum, pinMask, pinVal ))
    {
        LogVerbose( "Port %c.%d dir set to %d\n", 'A' + portNum, pin, ( pinVal != 0 ) ? "out" : "in" );
    }
    else
    {
        LogError( "Failed to set dir for %c.%d\n", portNum + 'A', pin );
    }

} // ProcessSetDirCommand

//***************************************************************************
/**
*   Reads a register from the AVR.
*
*/

void ProcessReadRegCommand( int i2cDev, const char *regStr )
{
    RegMap_t        regFound;
    I2C_IO_Info_t   info;

    if ( !ParseReg( i2cDev, regStr, &regFound ))
    {
        return;
    }

    // Make sure that we're at version 2 on the robostix

    if ( !I2C_IO_GetInfo( i2cDev, &info ))
    {
        LogError( "Unable to retrieve information from i2c address 0x%02x\n", gI2cAddr );
        exit( 1 );
    }
    if ( info.version < 2 )
    {
        LogError( "i2c-io.hex on the robostix is too old and needs to be updated\n" );
        return;
    }


    if ( regFound.is16Bit )
    {
        uint16_t    regVal16;

        if ( !I2C_IO_ReadReg16( i2cDev, regFound.reg, &regVal16 ))
        {
            LogError( "Error reading from %s\n", regFound.regStr );
            return;
        }

        if ( gBase == OPT_BASE_HEX )
        {
            Log( "0x%04x\n", regVal16 );
        }
        else
        {
            Log( "%d\n", regVal16 );
        }
    }
    else
    {
        uint8_t    regVal8;

        if ( !I2C_IO_ReadReg8( i2cDev, regFound.reg, &regVal8 ))
        {
            LogError( "Error reading from %s\n", regFound.regStr );
            return;
        }

        if ( gBase == OPT_BASE_HEX )
        {
            Log( "0x%02x\n", regVal8 );
        }
        else
        {
            Log( "%d\n", regVal8 );
        }
    }

} // ProcessReadRegCommand

//***************************************************************************
/**
*  Writes a register to the AVR.
*/

void ProcessWriteRegCommand( int i2cDev, const char *regStr, const char *valStr )
{
    RegMap_t        regMap;
    I2C_IO_Info_t   info;

    if ( !ParseReg( i2cDev, regStr, &regMap ))
    {
        return;
    }

    // Make sure that we're at version 2 on the robostix

    if ( !I2C_IO_GetInfo( i2cDev, &info ))
    {
        LogError( "Unable to retrieve information from i2c address 0x%02x\n", gI2cAddr );
        exit( 1 );
    }
    if ( info.version < 2 )
    {
        LogError( "i2c-io.hex on the robostix is too old and needs to be updated\n" );
        return;
    }

    if ( regMap.is16Bit )
    {
        uint16_t    regVal16;

        if ( !ParseUInt16( valStr, &regVal16 ))
        {
            return;
        }

        if ( !I2C_IO_WriteReg16( i2cDev, regMap.reg, regVal16 ))
        {
            LogError( "Error writing to %s\n", regMap.regStr );
            return;
        }
    }
    else
    {
        uint8_t    regVal8;

        if ( !ParseByte( valStr, &regVal8 ))
        {
            return;
        }
        if ( !I2C_IO_WriteReg8( i2cDev, regMap.reg, regVal8 ))
        {
            LogError( "Error writing to %s\n", regMap.regStr );
            return;
        }
    }

} // ProcessWriteRegCommand

//***************************************************************************
/**
*   Usage
*/

void Usage( void )
{
    fprintf( stderr, "Usage: i2c-io [options] i2c-addr cmd [cmd-options]\n" );
    fprintf( stderr, "Manipulate robostix I/O pins\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "The following commands are supported:\n" );
    fprintf( stderr, "Info                  Retrieves information about the robostix program\n" );
    fprintf( stderr, "Get port.pin          Retrieves the value of a pin\n" );
    fprintf( stderr, "Set port.pin val      Sets the value of a pin\n" );
    fprintf( stderr, "SetDir port.pin dir   Sets the direction of a pin\n" );
    fprintf( stderr, "ReadReg regIdx        Reads an 8 bit register\n" );
    fprintf( stderr, "WriteReg regIdx val   Writes a value into an 8 bit register\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "port.pin specifies a port using a letter (A-G) and a pin (0-7)\n" );
    fprintf( stderr, "For example, C.3 means Port C pin 3\n" );
    fprintf( stderr, "The port may also be specified using the special port ADC\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "The following arguments may be used to modify the behaviour\n" );
    fprintf( stderr, "--hex (or -x)         Print ReadReg results in hex\n" );
    fprintf( stderr, "--decimal (or -d)     Print ReadReg results in decimal\n" );
    fprintf( stderr, "--version             Print the version information for this program\n" );
    fprintf( stderr, "--verbose             Print additional information\n" );
    fprintf( stderr, "--help                Prints this information\n" );
}

