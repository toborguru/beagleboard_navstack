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
*   @file   BootLoader-api.c
*
*   @brief  This file contains the implementation for performing I2C operations
*           on the gumstix.
*
****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "i2c-api.h"
#include "AvrInfo.h"
#include "BootLoader.h"
#include "BootLoader-api.h"
#include "DumpMem.h"
#include "Log.h"

// ---- Public Variables ----------------------------------------------------

// ---- Private Constants and Types -----------------------------------------

#define TRUE    1
#define FALSE   0

// ---- Private Variables ---------------------------------------------------

// ---- Private Function Prototypes -----------------------------------------

int BootLoaderSetStructSize( int i2cDev, const BootLoaderInfo_t *bootInfo );

// ---- Functions -----------------------------------------------------------

//***************************************************************************
/**
*   Checks the version of the bootloader to see if we're compatible.
*/

int BootLoaderCheckVersion( const BootLoaderInfo_t *bootInfo )
{
    // If the range defined by BL_API_MIN_VERSION to BL_API_VERSION
    // overlaps with the info from the bootloader, then we can talk.

    if ( bootInfo->minVersion > BL_API_VERSION )
    {
        LogError( "BootLoader device is too new; Host is (%d-%d) Device is (%d-%d)\n",
                  BL_API_MIN_VERSION, BL_API_VERSION,
                  bootInfo->minVersion, bootInfo->version );

        return 0;
    }

    if ( BL_API_MIN_VERSION > bootInfo->version )
    {
        LogError( "BootLoader device is too old; Host is (%d-%d) Device is (%d-%d)\n",
                  BL_API_MIN_VERSION, BL_API_VERSION,
                  bootInfo->minVersion, bootInfo->version );

        return 0;
    }

    return 1;
}

//***************************************************************************
/**
*   Retrieves the boot delay from the device.
*/

int BootLoaderGetBootDelay( int i2cDev, const BootLoaderInfo_t *bootInfo, uint8_t *bootDelay )
{
    uint32_t    bootDelayAddr;

    bootDelayAddr = bootInfo->eepromSize - sizeof( BootLoaderEepromData_t )
                  + offsetof( BootLoaderEepromData_t, bootDelay );

    return BootLoaderRead( i2cDev, BL_MEM_TYPE_EEPROM, bootDelayAddr, 1, bootDelay );

} // BootLoaderGetBootDelay

//***************************************************************************
/**
*   Retrieves the I2C address that the device is configured with.
*/

int BootLoaderGetI2cAddr( int i2cDev, const BootLoaderInfo_t *bootInfo, I2C_Addr_t *i2cAddr )
{
    uint32_t    i2cAddrAddr;

    LogDebug( "----- BootLoaderGetI2cAddr -----\n" );

    i2cAddrAddr = bootInfo->eepromSize - sizeof( BootLoaderEepromData_t )
                + offsetof( BootLoaderEepromData_t, i2cAddr );

    return BootLoaderRead( i2cDev, BL_MEM_TYPE_EEPROM, i2cAddrAddr, 1, i2cAddr );

} // BootLoaderGetI2cAddr

//***************************************************************************
/**
*   Retrieves information about the bootloader
*/

int BootLoaderGetInfo( int i2cDev, BootLoaderInfo_t *bootInfo )
{
    LogDebug( "----- BootLoaderGetInfo -----\n" );

    if ( I2cReadBlock( i2cDev, BL_REG_GET_INFO, bootInfo, sizeof( *bootInfo ), NULL ) != 0 )
    {
        LogError( "I2cReadBlock failed\n" );
        return FALSE;
    }

    return TRUE;

} // BootLoaderGetInfo

//***************************************************************************
/**
*   Retrieves the node name from the device.
*/

int BootLoaderGetNodeName( int i2cDev, const BootLoaderInfo_t *bootInfo, char *nodeName )
{
    uint32_t    nodeNameAddr;

    nodeNameAddr = bootInfo->eepromSize - sizeof( BootLoaderEepromData_t )
                 + offsetof( BootLoaderEepromData_t, name );

    return BootLoaderRead( i2cDev, BL_MEM_TYPE_EEPROM, nodeNameAddr, BL_NODE_NAME_LEN, nodeName );

} // BootLoaderGetNodeName

//***************************************************************************
/**
*   Prints information about the bootloader
*/

void BootLoaderPrintInfo( const BootLoaderInfo_t *bootInfo, int verbose )
{
    const AvrInfo_t *avrInfo = FindAvrInfoBySignature( bootInfo->partNumber );
    const char      *avrString;

    if ( avrInfo == NULL )
    {
        avrString = "*** Unknown ***";
    }
    else
    {
        avrString = avrInfo->m_string;
    }

    if ( verbose )
    {
        Log( "I2C Dev Addr: 0x%02x\n",      bootInfo->devAddr );
        Log( "     Version: %d\n",          bootInfo->version );
        Log( "  MinVersion: %d\n",          bootInfo->minVersion );
        Log( " Part Number: 0x%04x (%s)\n", bootInfo->partNumber, avrString );
        Log( "    Reg Size: %4d\n",         bootInfo->regSize );
        Log( "    RAM Size: %4d\n",         bootInfo->ramSize );
        Log( " EEPROM Size: %4d\n",         bootInfo->eepromSize );
        Log( "   Page Size: %4d\n",         bootInfo->spmPageSize );
        Log( "  Flash Size: %4dk\n",        bootInfo->flashSize / 1024 );
    }
    else
    {
        Log( "Detected %s\n", avrString );
    }

} // BootLoaderPrintInfo

//***************************************************************************
/**
*   Sets the boot delay for the device.
*/

int BootLoaderSetBootDelay( int i2cDev, const BootLoaderInfo_t *bootInfo, uint8_t bootDelay )
{
    uint32_t    bootDelayAddr;

    bootDelayAddr = bootInfo->eepromSize - sizeof( BootLoaderEepromData_t )
                  + offsetof( BootLoaderEepromData_t, bootDelay );

    BootLoaderSetStructSize( i2cDev, bootInfo );

    return BootLoaderWrite( i2cDev, BL_MEM_TYPE_EEPROM, bootDelayAddr, 1, &bootDelay );

} // BootLoaderSetBootDelay

//***************************************************************************
/**
*   Sets the i2c address for the device.
*/

int BootLoaderSetI2cAddr( int i2cDev, const BootLoaderInfo_t *bootInfo, I2C_Addr_t i2cAddr )
{
    uint32_t    i2cAddrAddr;

    i2cAddrAddr = bootInfo->eepromSize - sizeof( BootLoaderEepromData_t )
                + offsetof( BootLoaderEepromData_t, i2cAddr );

    BootLoaderSetStructSize( i2cDev, bootInfo );

    return BootLoaderWrite( i2cDev, BL_MEM_TYPE_EEPROM, i2cAddrAddr, 1, &i2cAddr );

} // BootLoaderSetI2cAddr

//***************************************************************************
/**
*   Sets the node name for the device.
*/

int BootLoaderSetNodeName( int i2cDev, const BootLoaderInfo_t *bootInfo, const char *nodeName )
{
    uint32_t    nodeNameAddr;
    char        nodeNameBuf[ BL_NODE_NAME_LEN ];

    nodeNameAddr = bootInfo->eepromSize - sizeof( BootLoaderEepromData_t )
                 + offsetof( BootLoaderEepromData_t, name );

    memset( nodeNameBuf, 0, sizeof( nodeNameBuf ));
    strncpy( nodeNameBuf, nodeName, sizeof( nodeNameBuf ));
    nodeNameBuf[ sizeof( nodeNameBuf ) - 1 ] = '\0';

    BootLoaderSetStructSize( i2cDev, bootInfo );

    return BootLoaderWrite( i2cDev, BL_MEM_TYPE_EEPROM, nodeNameAddr, BL_NODE_NAME_LEN, nodeNameBuf );

} // BootLoaderSetNodeName

//***************************************************************************
/**
*   Sets the structure size for the Bootloader EEPROM parameters
*/

int BootLoaderSetStructSize( int i2cDev, const BootLoaderInfo_t *bootInfo )
{
    uint32_t    sizeAddr;
    uint8_t     structSize;
    int         rc;

    sizeAddr = bootInfo->eepromSize - sizeof( BootLoaderEepromData_t )
             + offsetof( BootLoaderEepromData_t, structSize );

    if (( rc = BootLoaderRead( i2cDev, BL_MEM_TYPE_EEPROM, sizeAddr, 1, &structSize )) == TRUE )
    {
        if ( structSize != sizeof( *bootInfo ))
        {
            structSize = sizeof( *bootInfo );
            rc = BootLoaderWrite( i2cDev, BL_MEM_TYPE_EEPROM, sizeAddr, 1, &structSize );
        }
    }

    return rc;

} // BootLoaderStructSize

//***************************************************************************
/**
*   Reads memory from the bootloader device.
*/

int BootLoaderRead( int i2cDev, uint8_t memType, uint32_t memAddr, uint8_t numBytes, void *readBuf )
{
    BootLoaderRead_t    req;
    uint8_t             bytesRead = 0;

    LogDebug( "----- BootLoaderRead memType:%d memAddr:0x%08x numBytes:%d -----\n",
              memType, memAddr, numBytes );

    req.addr.val32 = memAddr;
    req.addr.byte[3] = memType;
    req.len = numBytes;
    req.pad[ 0 ] = 0;
    req.pad[ 1 ] = 0;
    req.pad[ 2 ] = 0;

    if ( I2cProcessBlock( i2cDev, BL_REG_READ_DATA, &req, sizeof( req ), readBuf, numBytes, &bytesRead ) != 0 )
    {
        LogError( "BootLoaderRead: I2cReadBlock failed: %s (%d)\n", strerror( errno ), errno );
        return FALSE;
    }

    if ( gDebug )
    {
        DumpMem( "Read", 0, readBuf, bytesRead );
    }

    return TRUE;

} // BootLoaderRead

//***************************************************************************
/**
*   Causes the device to reboot itself (useful when changing the i2c address)
*/

int BootLoaderReboot( int i2cDev )
{
    if ( I2cWriteBlock( i2cDev, BL_CMD_REBOOT, NULL, 0 ) != 0 )
    {
        LogError( "BootLoaderReboot: I2cWriteBlock failed: %s (%d)\n", strerror( errno ), errno );
        return FALSE;
    }
    return TRUE;

} // BootLoaderReboot

//***************************************************************************
/**
*   Causes the device to run the user program.
*/

int BootLoaderRunProgram( int i2cDev )
{
    if ( I2cWriteBlock( i2cDev, BL_CMD_RUN_APP, NULL, 0 ) != 0 )
    {
        LogError( "BootLoaderRunProgram: I2cWriteBlock failed: %s (%d)\n", strerror( errno ), errno );
        return FALSE;
    }
    return TRUE;

} // BootLoaderRunProgram

//***************************************************************************
/**
*   Writes memory to the bootloader device.
*/

int BootLoaderWrite( int i2cDev, uint8_t memType, uint32_t memAddr, uint8_t numBytes, void *writeBuf )
{
    BootLoaderWrite_t   req;

    LogDebug( "----- BootLoaderWrite memType:%d memAddr:0x%08x numBytes:%d -----\n",
              memType, memAddr, numBytes );

    req.addr.val32 = memAddr;
    req.addr.byte[ 3 ] = memType;

    if ( numBytes > sizeof( req.data ))
    {
        numBytes = sizeof( req.data );
    }
    if ( numBytes > 0 )
    {
        memcpy( &req.data[ 0 ], writeBuf, numBytes );
    }

    if ( I2cWriteBlock( i2cDev, BL_REG_WRITE_DATA, &req, offsetof( BootLoaderWrite_t, data ) + numBytes ) != 0 )
    {
        LogError( "BootLoaderWrite: I2cWriteBlock failed: %s (%d)\n", strerror( errno ), errno );
        return FALSE;
    }

    if ( gDebug )
    {
        DumpMem( "Write", memAddr, writeBuf, numBytes );
    }

    return TRUE;

} // BootLoaderWrite

