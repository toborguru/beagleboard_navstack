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
*   @file   i2c-load.c
*
*   @brief  This file implements the i2c boot loader that runs on the
*           gumstix.
*
****************************************************************************/
/*
*
*   The default fuse settings (from the Robostix-Fuses page) are as follows:
*
*   --wr_fuse_l=0xbf --wr_fuse_h=0xc9 --wr_fuse_e=0xff
*
*   For the i2c-Bootloader, I've chosen a 4K byte (2K word) bootloader section
*   which corresponds to BOOTSZ1=1, and BOOTSZ0=1. The bootloader also requires
*   that the BOOTRST bit be set.
*
*   I'd also recommend setting EESAVE, which causes EEPROM contents to be
*   preserved across Chip Erase cycles.
*
*   From the ATMega128 data sheet (page 290), all four of these fuses are 
*   located in the "Fuse High Byte".
*
*   Their masks are as follows:
*
*                               Default             Desired
*   EESAVE  bit=3   mask=0x08   1 (unprogrammed)    0 - To save EEPROM contents
*   BOOTSZ1 bit=2   mask=0x04   0 (programmed)      0 - Select 2k words boot size
*   BOOTSZ0 bit=1   mask=0x02   0 (programmed)      1 - Select 2k words boot size
*   BOOTRST bit=0   mask=0x01   1 (unprogrammed)    0 - Reset activates boot area
*
*   So, we want to change the High Fuse Byte from 0xc9 to 0xc2.
*
*   If you're using a GUI based program for setting the fuses, checked 
*   typically corresponds to programmed (= 0)
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

#include "i2c-dev.h"
#include "i2c-api.h"
#include "BootLoader-api.h"
#include "BootLoader.h"
#include "DumpMem.h"
#include "FILE_Data.h"
#include "Log.h"
#include "../robostix_drv.h"
#include "SerialLog.h"

// ---- Public Variables ----------------------------------------------------
// ---- Private Constants and Types -----------------------------------------

// ---- Private Variables ---------------------------------------------------

enum
{
    OPT_MEM_DEFAULT     = 0,

    // Options assigned a single character code can use that charater code
    // as a short option.

    OPT_MEM_FLASH       = 'f',
    OPT_MEM_EEPROM      = 'e',
    OPT_MEM_SRAM        = 's',

    // Options from this point onwards don't have any short option equivalents

    OPT_FIRST_LONG_OPT   = 0x80,

    OPT_START_ADDR,
    OPT_END_ADDR,
    OPT_LENGTH,
    OPT_LOAD_ADDR,
    OPT_HELP,
};

enum
{
    CMD_DEFAULT,

    CMD_READ,
    CMD_WRITE,
    CMD_VERIFY,
    CMD_INFO,
    CMD_SET_BOOT_DELAY,
    CMD_SET_I2C_ADDR,
    CMD_SET_NODE_NAME,
    CMD_REBOOT,
    CMD_RUN,
    CMD_SCAN
};

struct 
{
    int         cmd;
    const char *cmdStr;

} gCmdMap[] =
{
    { CMD_READ,             "Read" },
    { CMD_WRITE,            "Write" },
    { CMD_VERIFY,           "Verify" },
    { CMD_INFO,             "Info" },
    { CMD_SET_BOOT_DELAY,   "SetBootDelay" },
    { CMD_SET_I2C_ADDR,     "SetI2cAddr" },
    { CMD_SET_NODE_NAME,    "SetNodeName" },
    { CMD_REBOOT,           "ReBoot" },
    { CMD_RUN,              "Run" },

    // I left scan out since it needs to parsed specially.
};

int gNumCmds = sizeof( gCmdMap ) / sizeof( gCmdMap[ 0 ]);

int gStartAddr  = -1;
int gEndAddr    = -1;
int gLength     = -1;
int gI2cAddr    = -1;
int gBootDelay  = -1;
int gNewI2cAddr = -1;
int gReset      = 0;
int gReboot     = 0;

int gSerialLog  = 0;

const char *gFileName = NULL;
const char *gNodeName = NULL;

const char *gCmdStr;
int         gCmd        = CMD_DEFAULT;
int         gMem        = OPT_MEM_DEFAULT;
uint8_t     gMemType    = BL_MEM_TYPE_FLASH;

struct option gOption[] =
{
    { "flash",      no_argument,        NULL,       OPT_MEM_FLASH },
    { "eeprom",     no_argument,        NULL,       OPT_MEM_EEPROM },
    { "sram",       no_argument,        NULL,       OPT_MEM_SRAM },
    { "start-addr", required_argument,  NULL,       OPT_START_ADDR },
    { "end-addr",   required_argument,  NULL,       OPT_END_ADDR },
    { "load-addr",  required_argument,  NULL,       OPT_LOAD_ADDR },
    { "length",     required_argument,  NULL,       OPT_LENGTH },
    { "verbose",    no_argument,        &gVerbose,  1 },
    { "quiet",      no_argument,        &gQuiet,    1 },
    { "debug",      no_argument,        &gDebug,    1 },
    { "serial-log", no_argument,        &gSerialLog,1 },
    { "reset",      no_argument,        &gReset,    1 },
    { "reboot",     no_argument,        &gReboot,   1 },
    { "help",       no_argument,        NULL,       OPT_HELP },
    { NULL }
};

#define TRUE    1
#define FALSE   0

// ---- Private Function Prototypes -----------------------------------------

static  void ProcessReadCmd( int i2cDev, BootLoaderInfo_t *bootInfo, const char *gFileName );
static  void ProcessScanCmd( int i2cDev );
static  int  ProcessVerifyCmd( int i2cDev, BootLoaderInfo_t *bootInfo, FILE_Data_t *fileData );
static  void ProcessWriteCmd( int i2cDev, BootLoaderInfo_t *bootInfo, FILE_Data_t *fileData );
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
    FILE_Data_t        *fileData = NULL;
    const char         *i2cDevName = "/dev/i2c-0";
    int                 i2cDev;
    BootLoaderInfo_t    bootInfo;
    int                 cmdIdx;
    uint32_t            highAddr;

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
            case OPT_MEM_FLASH:
            case OPT_MEM_EEPROM:
            case OPT_MEM_SRAM:
            {
                if ( gMem != OPT_MEM_DEFAULT )
                {
                    LogError( "Only one memory type can be specified\n" );
                    Usage();
                    exit( 1 );
                }
                gMem = opt;
                break;
            }

            case OPT_START_ADDR:
            {
                gStartAddr = strtol( optarg, NULL, 0 );
                break;
            }

            case OPT_END_ADDR:
            {
                gEndAddr = strtoul( optarg, NULL, 0 );
                if ( gLength >= 0 )
                {
                    LogError( "Can't specify both a length and an end address\n" );
                    exit( 1 );
                }
                break;
            }

            case OPT_LENGTH:
            {
                gLength = strtol( optarg, NULL, 0 );
                if ( gEndAddr >= 0 )
                {
                    LogError( "Can't specify both a length and an end address\n" );
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
                Usage();
                exit( 1 );
            }
        }
    }
    argc -= optind;
    argv += optind;


    if ( argc < 1 )
    {
        Usage();
        exit( 1 );
    }

    // Do a special check for the scan command. It doesn't use an i2c address
    // which is why we need a special check.

    if ( strcasecmp( argv[ 0 ], "scan" ) == 0 )
    {
        gCmd = CMD_SCAN;
    }
    else
    {
        // Verify that an i2c-address was specified

        gI2cAddr = strtol( argv[ 0 ], NULL, 0 );
        if (( gI2cAddr <= 0 ) || ( gI2cAddr > 127 ))
        {
            LogError( "Expecting i2c address in the range of 1-127, Found: %d\n", gI2cAddr );
            Usage();
            exit( 1 );
        }

        // Cerify that a command was specified.

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
    }

    // Process command specific arguments

    gFileName = NULL;
    gNodeName = NULL;

    if ( gCmd == CMD_SCAN )
    {
        if ( argc != 1 )
        {
            LogError( "Unexpected extra parameters\n" );
            Usage();
            exit( 1 );
        }
    }
    else
    if (( gCmd == CMD_INFO )
    ||  ( gCmd == CMD_RUN )
    ||  ( gCmd == CMD_REBOOT ))
    {
        if ( argc != 2 )
        {
            LogError( "Unexpected extra parameters\n" );
            Usage();
            exit( 1 );
        }
    }
    else
    if ( gCmd == CMD_SET_BOOT_DELAY )
    {
        if ( argc < 3 )
        {
            LogError( "Expecting boot delay\n" );
            Usage();
            exit( 1 );
        }
        gBootDelay = strtol( argv[ 2 ], NULL, 0 );
        if (( gBootDelay < 0 ) || ( gBootDelay > 255 ))
        {
            LogError( "Expecting boot delay in the range of 0-255, Found: %d\n", gBootDelay );
            Usage();
            exit( 1 );
        }
    }
    else
    if ( gCmd == CMD_SET_I2C_ADDR )
    {
        if ( argc < 3 )
        {
            LogError( "Expecting node name\n" );
            Usage();
            exit( 1 );
        }
        gNewI2cAddr = strtol( argv[ 2 ], NULL, 0 );
        if (( gNewI2cAddr < 0 ) || ( gNewI2cAddr > 255 ))
        {
            LogError( "Expecting i2c address in the range of 1-127, Found: %d\n", gNewI2cAddr );
            Usage();
            exit( 1 );
        }
    }
    else
    if ( gCmd == CMD_SET_NODE_NAME )
    {
        if ( argc < 3 )
        {
            LogError( "Expecting node name\n" );
            Usage();
            exit( 1 );
        }
        gNodeName = argv[ 2 ];
    }
    else
    {
        if ( argc < 3 )
        {
            LogError( "Expecting filename\n" );
            Usage();
            exit( 1 );
        }
        gFileName = argv[ 2 ];
    }

    // Setup the Start/End address 

    if ( gStartAddr < 0 )
    {
        gStartAddr = 0;
    }
    if ( gLength >= 0 )
    {
        gEndAddr = gStartAddr + gLength - 1;
    }
    else
    if ( gEndAddr >= 0 )
    {
        gLength = gEndAddr - gStartAddr + 1;
    }

    if ( gDebug )
    {
        Log( "i2cAddr:0x%02x Cmd: %s Mem: %c Start:0x%x End:0x%x Len:0x%x\n",
             gI2cAddr, gCmdStr, (char)gMem, gStartAddr, gEndAddr, gLength );
        if ( gFileName != NULL )
        {
            Log( "Filename: '%s'\n", gFileName );
        }
        if ( gNodeName != NULL )
        {
            Log( "NodeName: '%s'\n", gNodeName );
        }
    }

    signal( SIGINT, ControlC );

    // Read in the .hex file for those command that need it

    if (( gFileName != NULL ) && ( gCmd != CMD_READ ))
    {
        // Read in the file

        if (( fileData = FILE_ReadData( gFileName )) == NULL )
        {
            LogError( "Error reading file '%s'\n", gFileName );
            exit( 1 );
        }
    }

    // Reset the robostix, if requested. Note this only works if the robostix
    // is directly plugged into the gumstix.

    if ( gReset )
    {
        int rs;

        if (( rs = open( "/dev/robostix", O_RDWR )) < 0 )
        {
            LogError( "Unable to open /dev/robostix: %s (%d)\n", strerror( errno ), errno );
            exit( 1 );
        }

        ioctl( rs, ROBOSTIX_IOCTL_RESET, ROBOSTIX_PIN_PULSE );
        close( rs );
        usleep( 100000 );    // 100 mSec
    }

    // Try to open the i2c device

    if (( i2cDev = open( i2cDevName, O_RDWR )) < 0 )
    {
        LogError( "Error  opening '%s': %s\n", i2cDevName, strerror( errno ));
        exit( 1 );
    }

    if ( gCmd == CMD_SCAN )
    {
        ProcessScanCmd( i2cDev );
        close( i2cDev );
        exit( 1 );
    }

    // Indicate which slave we wish to speak to

    I2cSetSlaveAddress( i2cDev, gI2cAddr, I2C_USE_CRC );

    if ( gSerialLog )
    {
        StartSerialLog( B38400, SERIAL_LOG_STUART );
    }

    // Request BootLoader information

    if ( !BootLoaderGetInfo( i2cDev, &bootInfo ))
    {
        LogError( "Unable to retrieve boot information from i2c address 0x%02x\n", gI2cAddr );
        StopSerialLog();
        exit( 1 );
    }
    if ( !BootLoaderCheckVersion( &bootInfo ))
    {
        StopSerialLog();
        exit( 1 );
    }

    BootLoaderPrintInfo( &bootInfo, gVerbose || ( gCmd == CMD_INFO ));

    // Reboot the robostix, if requested. This is useful if you're downloading
    // the flash into an AVR which is using the bootloader. Using --reboot
    // causes the boot-loader to start running, so that the code we're downloading
    // isn't writing over top of running code.

    if ( gReboot )
    {
        BootLoaderReboot( i2cDev );

        usleep( 100000 );    // 100 mSec
    }

    switch ( gMem )
    {
        case OPT_MEM_DEFAULT:
        {
            gMem = OPT_MEM_FLASH;

            // Fall thru
        }
        case OPT_MEM_FLASH:     
        {
            gMemType = BL_MEM_TYPE_FLASH;    
            highAddr = bootInfo.flashSize - 1;
            break;
        }

        case OPT_MEM_EEPROM:
        {
            gMemType = BL_MEM_TYPE_EEPROM;   
            highAddr = bootInfo.eepromSize - 1;
            break;
        }

        case OPT_MEM_SRAM:
        {
            gMemType = BL_MEM_TYPE_SRAM;     
            highAddr = bootInfo.ramSize + bootInfo.regSize - 1;
            break;
        }

        default:
        {
            LogError( "Unsupported memory type: '%c'\n", gMem );
            StopSerialLog();
            exit( 1 );
        }
    }

    // Check the start and end address

    if ( gEndAddr < 0 )
    {
        gEndAddr = highAddr;
    }

    if ( gStartAddr > gEndAddr )
    {
        LogError( "Start address (0x%x) must be <= End address (0x%x)\n",
                  gStartAddr, gEndAddr );
        StopSerialLog();
        exit( 1 );
    }

    if ( gEndAddr > highAddr )
    {
        LogError( "End address (0x%x) is too big (Max 0x%x)\n", 
                  gEndAddr, highAddr );
        StopSerialLog();
        exit( 1 );
    }

    if ( gCmd == CMD_INFO )
    {
        char        nodeName[ BL_NODE_NAME_LEN ];
        uint8_t     bootDelay;
        I2C_Addr_t  i2cAddr;

        BootLoaderGetNodeName( i2cDev, &bootInfo, nodeName );

        if ( *nodeName == 0xFF )
        {
            Log( "Node Name: not set\n" );
        }
        else
        {
            Log( "Node Name: '%s'\n", nodeName );
        }

        BootLoaderGetBootDelay( i2cDev, &bootInfo, &bootDelay );

        if ( bootDelay == 0xFF )
        {
            Log( "Boot Delay: unset, defaults to %d seconds\n", BL_DEFAULT_BOOT_DELAY );
        }
        else
        {
            Log( "Boot Delay: %d seconds\n", bootDelay );
        }

        BootLoaderGetI2cAddr( i2cDev, &bootInfo, &i2cAddr );
        if ( i2cAddr == 0xFF )
        {
            Log( "I2C Addr: unset, defaults to 0x%02x\n", BL_DEFAULT_I2C_ADDR );
        }
        else
        {
            Log( "I2C Addr: 0x%02x\n", i2cAddr );
        }
    }
    else
    if ( gCmd == CMD_SET_BOOT_DELAY )
    {
        uint8_t     bootDelay;

        BootLoaderSetBootDelay( i2cDev, &bootInfo, gBootDelay );
        BootLoaderGetBootDelay( i2cDev, &bootInfo, &bootDelay );

        Log( "Boot Delay set to %d secondds\n", bootDelay );
    }
    else
    if ( gCmd == CMD_SET_I2C_ADDR )
    {
        BootLoaderSetI2cAddr( i2cDev, &bootInfo, gNewI2cAddr );

        Log( "I2C Address set to 0x%02x, rebooting....\n", gNewI2cAddr );

        BootLoaderReboot( i2cDev );
    }
    else
    if ( gCmd == CMD_SET_NODE_NAME )
    {
        char    nodeName[ BL_NODE_NAME_LEN ];

        BootLoaderSetNodeName( i2cDev, &bootInfo, gNodeName );
        BootLoaderGetNodeName( i2cDev, &bootInfo, nodeName );

        if ( *nodeName == 0xFF )
        {
            Log( "Node Name: not set\n" );
        }
        else
        {
            Log( "Node Name set to '%s'\n", nodeName );
        }
    }
    else
    if ( gCmd == CMD_READ )
    {
        ProcessReadCmd( i2cDev, &bootInfo, gFileName );
    }
    else
    if ( gCmd == CMD_VERIFY )
    {
        ProcessVerifyCmd( i2cDev, &bootInfo, fileData );
    }
    else
    if ( gCmd == CMD_WRITE )
    {
        ProcessWriteCmd( i2cDev, &bootInfo, fileData );
    }
    else
    if ( gCmd == CMD_REBOOT )
    {
        BootLoaderReboot( i2cDev );
    }
    else
    if ( gCmd == CMD_RUN )
    {
        BootLoaderRunProgram( i2cDev );
    }
    else
    {
        LogError( "Unrecognized command: %d\n", gCmd );
    }

    if ( gSerialLog )
    {
        // Stick around logging data on the serial port. The user is
        // expected to hit Control-C to exit.

        printf( "Logging serial port, press Ctrl-C to quit...\n" );

        while ( 1 )
        {
            sleep( 10 );
        }

    }
    close( i2cDev );

    return 0;

} // main

//***************************************************************************
/**
*   Handles the Read command, which generates a hex file rather than reading
*   one.
*/

void ProcessReadCmd( int i2cDev, BootLoaderInfo_t *bootInfo, const char *gFileName )
{
    int         i;
    uint32_t    addr;
    FILE       *fs;
    uint8_t     checksum;
    struct timeb    startTime;
    struct timeb    endTime;

    // Try to open the file

    if (( fs = fopen( gFileName, "w" )) == NULL )
    {
        fprintf( stderr, "Unable to open '%s' for writing: %s\n", gFileName, strerror( errno ));
        return;
    }

    // Even though we can read more than 16 bytes each time, we only read 16
    // to make it easy to write 16 bytes per line in the .hex file.

    Log( "Read: " );

    ftime( &startTime );
                                                          
    addr = gStartAddr;
    while ( addr <= gEndAddr )
    {
        uint32_t    bytesRemaining = gEndAddr - addr + 1;
        uint8_t     numBytes;
        uint8_t     readBuf[ 16 ];

        if ((( addr - gStartAddr ) % 1024 ) == 0 )
        {
            Log( "#" );
        }

        if ( bytesRemaining > 16 )
        {
            numBytes = 16;
        }
        else
        {
            numBytes = (uint8_t)bytesRemaining;
        }

        if ( !BootLoaderRead( i2cDev, gMemType, addr, numBytes, readBuf ))
        {
            Log( "\n" );
            LogError( "Error reading from 0x%x\n", addr );
            return;
        }

        fprintf( fs, ":%02X%04X00", numBytes, addr );

        checksum = numBytes + (( addr & 0xFF00 ) >> 8 ) + ( addr & 0x00FF );

        for ( i = 0; i < numBytes; i++ ) 
        {
            fprintf( fs, "%02X", readBuf[ i ]);
            checksum += readBuf[ i ];
        }
        fprintf( fs, "%02X\n", (uint8_t)-checksum );

        addr += numBytes;
    }
    ftime( &endTime );

    if ( startTime.millitm > endTime.millitm )
    {
        endTime.millitm += 1000;
        endTime.time--;
    }
    endTime.time -= startTime.time;
    endTime.millitm -= startTime.millitm;

    fprintf( fs, ":00000001FF\n" );

    fclose( fs );

    Log( "\n" );
    Log( "Read %d bytes and saved in %s in %d.%04d seconds\n", gEndAddr - gStartAddr + 1, gFileName, endTime.time, endTime.millitm );

} // ProcessReadCmd

//***************************************************************************
/**
*   Handles the Scan command, which scans the i2c bus looking for devices.
*/

void ProcessScanCmd( int i2cDev )
{
    BootLoaderInfo_t    bootInfo;
    int                 i2cAddr;
    int                 rc;

    // We assume that the gumstix is running with address 1.

    for ( i2cAddr = 2; i2cAddr <= 0x7f; i2cAddr++ ) 
    {
        // Indicate which slave we wish to speak to

        if (( rc = ioctl( i2cDev, I2C_SLAVE, i2cAddr )) < 0 )
        {
            if ( rc == EBUSY )
            {
                // Some other driver is using this device (i.e. RTC or other 
                // chip driver) so we'll skip it.

                Log( "0x%02x - busy - in use\n", i2cAddr );
                continue;
            }
            LogError( "ProcessScanCmd: Error trying to set slave address to 0x%02x (%d %s)\n", 
                      i2cAddr, errno, strerror( errno ));
            break;
        }

        // The following code came from the i2cdetect program, so I decided to
        // follow it. Presumably "normal" devices in these ranges
        // could respond badly if we don't do this.

        if ((( i2cAddr >= 0x30 ) && ( i2cAddr <= 0x37 ))
        ||  (( i2cAddr >= 0x50 ) && ( i2cAddr <= 0x5f )))
        {
            rc = i2c_smbus_read_byte( i2cDev );
        }
        else
        {
            rc = i2c_smbus_write_quick( i2cDev, I2C_SMBUS_WRITE );
        }
        if ( rc < 0 )
        {
            continue;
        }

        // We've found something. See if it's running the Robostix Bootloader

        I2cSetSlaveAddress( i2cDev, i2cAddr, I2C_USE_CRC );

        if ( !BootLoaderGetInfo( i2cDev, &bootInfo ))
        {
            LogError( "Unable to retrieve boot information from i2c address 0x%02x\n", gI2cAddr );
            continue;
        }

        BootLoaderPrintInfo( &bootInfo, 1 );
    }

} // ProcessScanCmd

//***************************************************************************
/**
*   Handles the Verify command, which compares memory to the contents of
*   a hex file.
*/

int ProcessVerifyCmd( int i2cDev, BootLoaderInfo_t *bootInfo, FILE_Data_t *fileData )
{
    uint32_t        addr;
    uint32_t        endAddr;
    uint32_t        bytesRemaining;
    uint32_t        numBytes;
    FILE_Block_t   *block;
    uint8_t         readBuf[ BL_MAX_DATA_BYTES ];
    uint32_t        cmpIdx;
    uint8_t        *cmpBuf;

    Log( "Verify: " );

    block = fileData->head;
    while ( block != NULL )
    {
        endAddr = block->address + block->blockLen - 1;
        if (( block->address <= gEndAddr ) && ( endAddr >= gStartAddr ))
        {
            // Some part of the block is in the address range.

            Log( "#" );

            if ( block->address > gStartAddr )
            {
                addr = block->address;
            }
            else
            {
                addr = gStartAddr;
            }

            if ( endAddr > gEndAddr )
            {
                endAddr = gEndAddr;
            }

            bytesRemaining = endAddr - addr + 1;

            cmpBuf = &block->data[ addr - block->address ];

            while ( bytesRemaining > 0 )
            {
                numBytes = bytesRemaining;
                if ( numBytes > BL_MAX_DATA_BYTES )
                {
                    numBytes = BL_MAX_DATA_BYTES;
                }

                if ( !BootLoaderRead( i2cDev, gMemType, addr, numBytes, readBuf ))
                {
                    Log( "\n" );
                    LogError( "Error reading %d bytes from 0x%x\n", numBytes, addr );
                    return 0;
                }

                for ( cmpIdx = 0; cmpIdx < numBytes; cmpIdx++ ) 
                {
                    if ( cmpBuf[ cmpIdx ] != readBuf[ cmpIdx ] )
                    {
                        Log( "\n" );
                        LogError( "Verify failed at offset 0x%04x, found 0x%02x, expecting 0x%02x\n",
                                  addr + cmpIdx, readBuf[ cmpIdx ], cmpBuf[ cmpIdx ]);
                        return 0;
                    }
                }

                cmpBuf += numBytes;
                addr += numBytes;
                bytesRemaining -= numBytes;
            }
        }

        block = block->next;
    }

    Log( "\n" );
    Log( "Verify sucessful\n" );

    return 1;

} // ProcessVerifyCmd

//***************************************************************************
/**
*   Handles the Write command, which compares memory to the contents of
*   a hex file.
*/

void ProcessWriteCmd( int i2cDev, BootLoaderInfo_t *bootInfo, FILE_Data_t *fileData )
{
    uint32_t        addr;
    uint32_t        endAddr;
    uint32_t        bytesRemaining;
    uint32_t        numBytes;
    FILE_Block_t   *block;
    uint8_t        *writeBuf;
    uint32_t        pageMask = ~( bootInfo->spmPageSize - 1 );

    Log( "Write:  " );

    block = fileData->head;
    while ( block != NULL )
    {
        endAddr = block->address + block->blockLen - 1;
        if (( block->address <= gEndAddr ) && ( endAddr >= gStartAddr ))
        {
            // Some part of the block is in the address range.

            Log( "#" );

            if ( block->address > gStartAddr )
            {
                addr = block->address;
            }
            else
            {
                addr = gStartAddr;
            }

            if ( endAddr > gEndAddr )
            {
                endAddr = gEndAddr;
            }

            bytesRemaining = endAddr - addr + 1;

            writeBuf = &block->data[ addr - block->address ];

            while ( bytesRemaining > 0 )
            {
                numBytes = bytesRemaining;
                if ( numBytes > BL_MAX_DATA_BYTES )
                {
                    numBytes = BL_MAX_DATA_BYTES;
                }

                if ( gMemType == BL_MEM_TYPE_FLASH )
                {
                    // If we're programming flash, we need to make sure that
                    // the data we're writing doesn't cross a page boundary.

                    if (( addr & pageMask ) != (( addr + numBytes - 1 ) & pageMask ))
                    {
                        numBytes = (( addr + bootInfo->spmPageSize ) & pageMask ) - addr;
                    }
                }

                if ( !BootLoaderWrite( i2cDev, gMemType, addr, numBytes, writeBuf ))
                {
                    Log( "\n" );
                    LogError( "Error writing %d bytes to 0x%x\n", numBytes, addr );
                    return;
                }

                writeBuf += numBytes;
                addr += numBytes;
                bytesRemaining -= numBytes;
            }
        }

        block = block->next;
    }

    if ( gMemType == BL_MEM_TYPE_FLASH )
    {
        // We've written all of the data, but in the event of writing flash, there
        // may still be some data sitting in the page buffer. Perform a zero-length
        // write (of any address) to force any dirty data to be written.

        if ( !BootLoaderWrite( i2cDev, gMemType, fileData->head->address, 0, NULL ))
        {
            Log( "\n" );
            LogError( "Error flushing flash write buffer 0x%x\n", fileData->head->address );
            return;
        }
    }
    Log( "\n" );

    // Now go back and verify that the data was written correctly

    if (( gMemType == BL_MEM_TYPE_FLASH ) || ( gMemType == BL_MEM_TYPE_EEPROM ))
    {
        if ( ProcessVerifyCmd( i2cDev, bootInfo, fileData ))
        {
            Log( "Write sucessful, rebooting...\n" );

            if ( gMemType == BL_MEM_TYPE_FLASH )
            {
                BootLoaderReboot( i2cDev );
            }
        }
    }

} // ProcessWriteCmd

//***************************************************************************
/**
*   Usage
*/

void Usage( void )
{
    fprintf( stderr, "Usage: i2c-load [options] i2c-addr cmd file\n" );
    fprintf( stderr, "Communicate with i2c boot loader\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "Only one of the following commands may be specified\n" );
    fprintf( stderr, "Info          Display information about the bootloader\n" );
    fprintf( stderr, "Read          Read from memory from th device\n" );
    fprintf( stderr, "Write         Write to memory on the device (default)\n" );
    fprintf( stderr, "Verify        Verify the contents of memory on the device\n" );
    fprintf( stderr, "SetI2cAddr    Sets the i2c address (and reboots)\n" );
    fprintf( stderr, "SetBootDelay  Sets the time that the boot loader waits for an i2c command\n" );
    fprintf( stderr, "SetNodeName   Sets the node name of the device\n" );
    fprintf( stderr, "Reboot        Reruns the bootloader from the beginning\n" );
    fprintf( stderr, "Run           Runs the user program\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "Only one of the following memory options may be specified\n" );
    fprintf( stderr, "-f, --flash   Perform the desired operation on flash memory (default)\n" );
    fprintf( stderr, "-e, --eeprom  Perform the desired operation on EEPROM memory\n" );
    fprintf( stderr, "-s, --sram    Perform the desired operation on SRAM memory\n" );
    fprintf( stderr, "\n" );
    fprintf( stderr, "The following arguments may be used to modify the behaviour\n" );
    fprintf( stderr, "--start-addr=addr Starting file addresses to use\n" );
    fprintf( stderr, "--end-addr=addr   Ending file address to use (inclusive)\n" );
    fprintf( stderr, "--length=len      Alternate way of specifying the stop address\n" );
    fprintf( stderr, "--verbose         Print additional information\n" );
    fprintf( stderr, "--quiet           Don't print anything (except errors)\n" );
    fprintf( stderr, "--serial-log      Log lines received on /dev/ttyS2\n" );
    fprintf( stderr, "--reset           Resets the robostix (only works if plugged into gumstix)\n" );
    fprintf( stderr, "--reboot          Reboots into the bootloader\n" );
    fprintf( stderr, "--help            Prints this information\n" );
}

