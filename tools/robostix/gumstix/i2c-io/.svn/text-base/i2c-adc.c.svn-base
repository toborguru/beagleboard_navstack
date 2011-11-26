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
*   @file   i2c-adc.c
*
*   @brief  This file test ADC performance through the i2c-io interface.
*
****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>

#include "i2c-api.h"
#include "i2c-io-api.h"
#include "Log.h"

// ---- Functions -----------------------------------------------------------

//***************************************************************************
/**
*   Main entry point
*/

int main( int argc, char **argv )
{
    int                 i2cDev;
    const char         *i2cDevName = "/dev/i2c-0";
    time_t              prevTime;
    time_t              endTime;
    int                 count = 0;

    LogInit( stdout );

    if (( i2cDev = open( i2cDevName, O_RDWR )) < 0 )
    {
        LogError( "Error  opening '%s': %s\n", i2cDevName, strerror( errno ));
        exit( 1 );
    }

    I2cSetSlaveAddress( i2cDev, 0x0B, I2C_USE_CRC );

    // Wait for the time to roll over

    prevTime = time( NULL );
    while ( time( NULL ) == prevTime )
    {
        ;
    }
    endTime = prevTime + 10;

    while ( time( NULL ) <= endTime )
    {
        uint16_t    adcVal;

        if ( I2C_IO_GetADC( i2cDev, 0, &adcVal ))
        {
            count++;
        }
        else
        {
            LogError( "Failed to retrieve ADC value\n" );
            break;
        }
    }
    printf( "Retrieved %d ADC values in 10 seconds\n", count );

    close( i2cDev );

    return 0;
}
