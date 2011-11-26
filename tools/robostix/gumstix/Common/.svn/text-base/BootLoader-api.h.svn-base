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
*   @file   BootLoader-api.h
*
*   @brief  This file contains definitions for performing BootLoader operations 
*           on the gumstix.
*
****************************************************************************/

#if !defined( BOOTLOADER_API_H )
#define BOOTLOADER_API_H

// ---- Include Files -------------------------------------------------------

#include <inttypes.h>
#include "BootLoader.h"

// ---- Constants and Types -------------------------------------------------

// ---- Variable Externs ----------------------------------------------------

// ---- Function Prototypes -------------------------------------------------

int BootLoaderCheckVersion( const BootLoaderInfo_t *bootInfo );

int BootLoaderGetBootDelay( int i2cDev, const BootLoaderInfo_t *bootInfo, uint8_t *bootDelay );
int BootLoaderGetI2cAddr( int i2cDev, const BootLoaderInfo_t *bootInfo, I2C_Addr_t *i2cAddr );
int BootLoaderGetInfo( int i2cDev, BootLoaderInfo_t *bootInfo );
int BootLoaderGetNodeName( int i2cDev, const BootLoaderInfo_t *bootInfo, char *nodeName );

void BootLoaderPrintInfo( const BootLoaderInfo_t *bootInfo, int verbose );

int BootLoaderSetBootDelay( int i2cDev, const BootLoaderInfo_t *bootInfo, uint8_t bootDelay );
int BootLoaderSetI2cAddr( int i2cDev, const BootLoaderInfo_t *bootInfo, I2C_Addr_t i2cAddr );
int BootLoaderSetNodeName( int i2cDev, const BootLoaderInfo_t *bootInfo, const char *nodeName );

int BootLoaderReboot( int i2cDev );
int BootLoaderRunProgram( int i2cDev );

int BootLoaderRead( int i2cDev, uint8_t memType, uint32_t memAddr, uint8_t numBytes, void *readBuf );
int BootLoaderWrite( int i2cDev, uint8_t memType, uint32_t memAddr, uint8_t numBytes, void *writeBuf );

#endif  // BOOTLOADER_API_H

