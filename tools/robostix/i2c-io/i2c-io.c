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
*   @brief  This file implements a set of I2C commands which allows the
*           robostix I/O to be controlled by the gumstix.
*
*****************************************************************************/

/* ---- Include Files ----------------------------------------------------- */

#include <avr/io.h>
#if defined( __AVR_LIBC_VERSION__ )
#   include <avr/interrupt.h>
#else
#   include <avr/signal.h>
#endif
#include <compat/twi.h>
#include <stdio.h>
#include <inttypes.h>

#include "i2c-io.h"
#include "Hardware.h"
#include "i2c-slave-boot.h"
#include "Log.h"
#include "Delay.h"
#include "Timer.h"
#include "UART.h"

#include "svn-version.h"

/* ---- Public Variables -------------------------------------------------- */

/* ---- Private Constants and Types --------------------------------------- */

/* ---- Private Variables ------------------------------------------------- */

#define NUM_PORTS   7

// Fake out ports that don't exist

#if !defined( PORTA )
#define PORTA   PORTB
#define DDRA    DDRB
#define PINA    PINB
#endif

#if !defined( PORTE )
#define PORTE   PORTB
#define DDRE    DDRB
#define PINE    PINB
#endif

#if !defined( PORTF )
#define PORTF   PORTB
#define DDRF    DDRB
#define PINF    PINB
#endif

#if !defined( PORTG )
#define PORTG   PORTB
#define DDRG    DDRB
#define PING    PINB
#endif

volatile uint8_t *gPORT[ NUM_PORTS ] =
{
    &PORTA, &PORTB, &PORTC, &PORTD, &PORTE, &PORTF, &PORTG
};

volatile uint8_t *gDDR[ NUM_PORTS ] =
{
    &DDRA, &DDRB, &DDRC, &DDRD, &DDRE, &DDRF, &DDRG
};

volatile uint8_t *gPIN[ NUM_PORTS ] =
{
    &PINA, &PINB, &PINC, &PIND, &PINE, &PINF, &PING
};

/* ---- Private Function Prototypes --------------------------------------- */

#undef  LED_ON
#undef  LED_OFF

#define LED_OFF()   do { CFG_BOOTLOADER_BEAT_PORT &= ~CFG_BOOTLOADER_BEAT_MASK; } while (0)
#define LED_ON()    do { CFG_BOOTLOADER_BEAT_PORT |=  CFG_BOOTLOADER_BEAT_MASK; } while (0)


#define IO_LOG_ENABLED  0

#if IO_LOG_ENABLED
#   define  IO_LOG0( fmt )                      LogBuf0( "IO: " fmt )
#   define  IO_LOG1( fmt, arg1 )                LogBuf1( "IO: " fmt, arg1 )
#   define  IO_LOG2( fmt, arg1, arg2 )          LogBuf2( "IO: " fmt, arg1, arg2 )
#   define  IO_LOG3( fmt, arg1, arg2, arg3 )    LogBuf3( "IO: " fmt, arg1, arg2, arg3 )
#else
#   define  IO_LOG0( fmt )
#   define  IO_LOG1( fmt, arg1 )
#   define  IO_LOG2( fmt, arg1, arg2 )
#   define  IO_LOG3( fmt, arg1, arg2, arg3 )
#endif

int ProcessCommand( I2C_Data_t *packet );

/* ---- Functions --------------------------------------------------------- */

//***************************************************************************
/**
*   Main loop for the I2C I/O program.
*/

int main(void)
{
    int     count;

    InitHardware();

    CFG_BOOTLOADER_BEAT_DDR |= CFG_BOOTLOADER_BEAT_MASK;

    // The first handle opened for read goes to stdin, and the first handle
    // opened for write goes to stdout. So u0 is stdin, stdout, and stderr

#if IO_LOG_ENABLED

#if defined( __AVR_LIBC_VERSION__ )
    fdevopen( UART0_PutCharStdio, UART0_GetCharStdio );
#else
    fdevopen( UART0_PutCharStdio, UART0_GetCharStdio, 0 );
#endif
    LogInit( stdout );

    Log( "*****\n" );
    Log( "***** I2C I/O program (I2C addr: 0x%02x\n", TWAR >> 1 );
    Log( "*****\n" );

#endif // IO_LOG_ENABLED

    if ( !I2C_SlaveBootInit( ProcessCommand ))
    {
        LogError( "I2C_SlaveBootInit failed\n" );

        LED_ON();
        while ( 1 )
        {
            ;
        }
    }

    sei();

    // The main loop just does an interesting heartbeat with a two pulses
    // close together, followed by a longer pause.

    count = 0;

    while ( 1 )
    {
        tick_t prevTick;

        switch ( count )
        {
            case   0:   LED_ON();       break;
            case  10:   LED_OFF();      break;
            case  20:   LED_ON();       break;
            case  30:   LED_OFF();      break;
            case 100:   count = -1;     break;
        }
        count++;

        prevTick = gTickCount;
        while ( gTickCount == prevTick )
        {
#if IO_LOG_ENABLED
            LogBufDump();
#endif
        }
    }

    return 0;

} // main

//***************************************************************************
/**
*   I2C Interrupt service routine
*/

SIGNAL(SIG_2WIRE_SERIAL)
{
    if ( !I2C_SlaveBootHandler() )
    {
        LogError( "Unrecognized status: 0x%x\n", TW_STATUS );
    }

    // Now that we've finished dealing with the interrupt, set the TWINT flag
    // which will stop the clock stretching and clear the interrupt source

    TWCR |= ( 1 << TWINT );

} // SIG_2WIRE_SERIAL

//***************************************************************************
/**
*   Callback called to process incoming i2c commands.
*/

int ProcessCommand( I2C_Data_t *packet )
{
    int     rc;
    uint8_t cmd = packet->m_data[ 0 ];

    switch ( cmd )
    {
        case I2C_IO_GET_INFO:
        {
            I2C_IO_Info_t   *info = (I2C_IO_Info_t *)&packet->m_data[ 1 ];

            IO_LOG0( "GetInfo\n" );

            info->version       = I2C_IO_API_VERSION;
            info->minVersion    = I2C_IO_API_MIN_VERSION;
            info->svnRevision   = SVN_REVISION;

            packet->m_data[ 0 ] = sizeof( *info );

            return sizeof( *info ) + 1; // + 1 for len
        }

        case I2C_IO_GET_GPIO:
        {
            I2C_IO_Get_GPIO_t *req = (I2C_IO_Get_GPIO_t *)&packet->m_data[ 2 ];  // +1 for cmd, +1 for len
            uint8_t         portNum = req->portNum;

            if ( portNum > NUM_PORTS )
            {
                portNum = 0;
            }

            packet->m_data[ 0 ] = 1;
            packet->m_data[ 1 ] = *(gPIN[ portNum ]);

            IO_LOG2( "GetGPIO Port %c: 0x%02x\n", portNum + 'A', packet->m_data[ 1 ]);
            return 2;
        }

        case I2C_IO_SET_GPIO:
        {
            I2C_IO_Set_GPIO_t *req = (I2C_IO_Set_GPIO_t *)&packet->m_data[ 2 ];  // +1 for cmd, +1 for len
            uint8_t     portNum = req->portNum;
            uint8_t     pinMask = req->pinMask;
            uint8_t     pinVal  = req->pinVal;

            if ( portNum > NUM_PORTS )
            {
                portNum = 0;
            }

            IO_LOG3( "SetGPIO Port %c: Mask:0x%02x Val:0x%02x\n", portNum + 'A', pinMask, pinVal );

            *(gPORT[ portNum ]) &= ~pinMask;
            *(gPORT[ portNum ]) |= ( pinVal & pinMask );

            return 0;
        }

        case I2C_IO_GET_GPIO_DIR:
        {
            I2C_IO_Get_GPIO_t *req = (I2C_IO_Get_GPIO_t *)&packet->m_data[ 2 ];  // +1 for cmd, +1 for len
            uint8_t     portNum = req->portNum;

            if ( portNum > NUM_PORTS )
            {
                portNum = 0;
            }

            packet->m_data[ 0 ] = 1;
            packet->m_data[ 1 ] = *(gDDR[ portNum ]);

            IO_LOG2( "GetGPIODir Port %c: 0x%02x\n", portNum + 'A', packet->m_data[ 1 ]);
            return 2;
        }

        case I2C_IO_SET_GPIO_DIR:
        {
            I2C_IO_Set_GPIO_t *req = (I2C_IO_Set_GPIO_t *)&packet->m_data[ 2 ];  // +1 for cmd, +1 for len
            uint8_t     portNum = req->portNum;
            uint8_t     pinMask = req->pinMask;
            uint8_t     pinVal  = req->pinVal;

            if ( portNum > NUM_PORTS )
            {
                portNum = 0;
            }

            IO_LOG3( "SetGPIODir Port %c: Mask:0x%02x Val:0x%02x\n", portNum + 'A', pinMask, pinVal );

            *(gDDR[ portNum ]) &= ~pinMask;
            *(gDDR[ portNum ]) |= ( pinVal & pinMask );

            return 0;
        }

        case I2C_IO_GET_ADC:
        {
            I2C_IO_Get_ADC_t    *req = (I2C_IO_Get_ADC_t *)&packet->m_data[ 2 ];    // +1 for cmd, +1 for len
            uint8_t              mux = req->mux;

            // Set ADMUX but don't mess with REFS0 & REFS1

            ADMUX = ( ADMUX & (( 1 << REFS1 ) | ( 1 << REFS0 ))) | mux;

            // Start the conversion
            ADCSR = ADCSR | ( 1 << ADSC );

            // Wait for it to complete
            while ( ADCSR & ( 1 << ADSC ));

            packet->m_data[ 0 ] = 2;
            packet->m_data[ 1 ] = ADCL;
            packet->m_data[ 2 ] = ADCH;

            IO_LOG3( "GetADC mux:%d read 0x%02x%02x\n", mux, packet->m_data[ 2 ], packet->m_data[ 1 ]);

            return 3;
        }

        case I2C_IO_READ_REG_8:
        {
            I2C_IO_ReadReg8_t  *req = (I2C_IO_ReadReg8_t *)&packet->m_data[ 2 ];    // +1 for cmd, +1 for len
            volatile uint8_t   *regPtr = (volatile uint8_t *)(int)(req->reg);

            packet->m_data[ 0 ] = 1;
            packet->m_data[ 1 ] = *regPtr;

            IO_LOG2( "ReadReg8 reg:0x%02x read 0x%02x\n", (uint8_t)(int)regPtr, packet->m_data[ 1 ]);

            return 2;
        }

        case I2C_IO_READ_REG_16:
        {
            I2C_IO_ReadReg16_t *req = (I2C_IO_ReadReg16_t *)&packet->m_data[ 2 ];    // +1 for cmd, +1 for len
            volatile uint16_t  *regPtr = (volatile uint16_t *)(int)(req->reg);
            uint16_t            regVal = *regPtr; 

            packet->m_data[ 0 ] = 2;
            packet->m_data[ 1 ] = (uint8_t)(  regVal        & 0xFF );
            packet->m_data[ 2 ] = (uint8_t)(( regVal >> 8 ) & 0xFF );

            IO_LOG3( "ReadReg16 reg:0x%02x read 0x%02x%02x\n", (uint8_t)(int)regPtr, packet->m_data[ 2 ], packet->m_data[ 1 ]);

            return 3;
        }

        case I2C_IO_WRITE_REG_8:
        {
            I2C_IO_WriteReg8_t  *req = (I2C_IO_WriteReg8_t *)&packet->m_data[ 2 ];    // +1 for cmd, +1 for len
            volatile uint8_t   *regPtr = (volatile uint8_t *)(int)(req->reg);

            *regPtr = req->val;

            IO_LOG2( "WriteReg8 reg:0x%02x wrote 0x%02x\n", (uint8_t)(int)regPtr, req->val);

            return 0;
        }

        case I2C_IO_WRITE_REG_16:
        {
            I2C_IO_WriteReg16_t *req = (I2C_IO_WriteReg16_t *)&packet->m_data[ 2 ];    // +1 for cmd, +1 for len
            volatile uint8_t   *regPtr = (volatile uint8_t *)(int)(req->reg);
            uint8_t             valH = (( req->val >> 8 ) & 0xFF );
            uint8_t             valL = (  req->val        & 0xFF );

            // For writing 16 bit registers, we need to write the high byte first

            regPtr[ 1 ] = valH;
            regPtr[ 0 ] = valL;

            IO_LOG3( "WriteReg16 reg:0x%02x wrote 0x%02x%02x\n", (uint8_t)(int)regPtr, valH, valL );

            return 0;
        }
    }

    // It wasn't one of our commands, see if it's a bootloader command.

    if (( rc = I2C_SlaveBootProcessCommand( packet )) < 0 )
    {
        LogError( "Unrecognized command: 0x%02x\n", cmd );
    }

    return rc;

} // ProcessCommand

