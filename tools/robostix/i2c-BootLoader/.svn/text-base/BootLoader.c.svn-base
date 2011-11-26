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
*   @file   BootLoader.c 
*
*   @brief  Implements the i2c Boot Loader
*
*****************************************************************************/

/* ---- Include Files ----------------------------------------------------- */

#include <stddef.h>

#include <avr/io.h>
#if defined( __AVR_LIBC_VERSION__ )
#   include <avr/interrupt.h>
#else
#   include <avr/signal.h>
#endif

#include <avr/boot.h>
#include <avr/eeprom.h>
#include <compat/twi.h>
#include <avr/pgmspace.h>

#include "AvrInfo.h"
#include "BootLoader.h"
#include "Config.h"
#include "Hardware.h"
#include "i2c-slave.h"
#include "Delay.h"
#include "Log.h"
#include "Timer.h"

#if CFG_USE_UART
#include "UART.h"
#endif

/* ---- Public Variables -------------------------------------------------- */
/* ---- Private Constants and Types --------------------------------------- */

#if !defined( STANDALONE )
#   define  STANDALONE 0
#endif

#define SPM_PAGEMASK    ~((uint32_t)SPM_PAGESIZE - 1)

#if STANDALONE && 0

#define BOOT_LOG_ENABLED    1

#else

#define BOOT_LOG_ENABLED    0

#endif

#if BOOT_LOG_ENABLED
#   if CFG_LOG_TO_BUFFER
#       define  BOOT_LOG0( fmt )             LogBuf0( "Boot: " fmt )
#       define  BOOT_LOG1( fmt, arg1 )       LogBuf1( "Boot: " fmt, arg1 )
#       define  BOOT_LOG2( fmt, arg1, arg2 ) LogBuf2( "Boot: " fmt, arg1, arg2 )
#   else
#       define  BOOT_LOG0( fmt )             Log( "Boot: " fmt )
#       define  BOOT_LOG1( fmt, arg1 )       Log( "Boot: " fmt, arg1 )
#       define  BOOT_LOG2( fmt, arg1, arg2 ) Log( "Boot: " fmt, arg1, arg2 )
#   endif
#else
#   define  BOOT_LOG0( fmt )
#   define  BOOT_LOG1( fmt, arg1 )
#   define  BOOT_LOG2( fmt, arg1, arg2 )

#endif
#define BOOT_MODE_BOOT  0   // BootLoader launched from Reset
#define BOOT_MODE_APP   1   // BootLoader launched from application

#if ( SPM_PAGESIZE < 256 )

typedef uint8_t PageSize_t;

#else

typedef uint16_t PageSize_t;

#endif

// The addr32 macro returns a byte address of a symbol located anywhere in 
// the 128K space of the ATMega128.

#define addr32(symbol)        \
({                                          \
    uint32_t __result;                      \
    __asm__                                 \
    (                                       \
        "ldi %A0, lo8(pm(" #symbol "))\n\t" \
        "ldi %B0, hi8(pm(" #symbol "))\n\t" \
        "eor %C0, %C0"              "\n\t"  \
        "eor %D0, %D0"              "\n\t"  \
        "add %A0, %A0"              "\n\t"  \
        "adc %B0, %B0"              "\n\t"  \
        "adc %C0, %C0"              "\n\t"  \
        : "=r" (__result)                   \
    );                                      \
    __result;                               \
})

/* ---- Private Variables ------------------------------------------------- */

I2C_Globals_t           gI2cGlobals;
BootLoaderGlobals_t     gBootLoaderGlobals;
BootLoaderEepromData_t  gEeprom;
volatile uint8_t        gStayInBootloader;

/* ---- Private Function Prototypes --------------------------------------- */

void    AppBoot( void );
void    BootLoader( int mode );
int     BootLoaderProcessCommand( BootLoaderGlobals_t *globals, I2C_Data_t *packet );

static  void RunUserProgram( void );
static  int  ProcessCommand( I2C_Data_t *rcvdPacket );

static  void WriteFlashPage( uint8_t *src, uint32_t addr );
static  void WriteFlash( BootLoaderGlobals_t *globals, uint8_t *src, uint32_t addr, uint8_t numBytes );

#if defined( __AVR_ATmega128__ )

// From memcpy_EP.S

void *memcpy_EP( void *dst, uint32_t src, size_t len );

#else

#define  memcpy_EP( dst, src, len )  memcpy_P( dst, (PGM_VOID_P)(uint16_t)src, len )

#endif


/* ---- Functions --------------------------------------------------------- */

#if STANDALONE

int main( void )
{
    AppBoot();
    return 0;
}

#endif  // STANDALONE

//***************************************************************************
/**
*   Entry point from an application (stays in bootloader - no timeout)
*/

void AppBoot( void )
{
    BootLoader(  BOOT_MODE_APP );

} // AppBoot

#if 0
void FlashRed( int numTimes )
{
    CFG_BOOTLOADER_BEAT_DDR |= CFG_BOOTLOADER_BEAT_MASK;

    Delay10mSec( 50 );

    while ( numTimes > 0 )
    {
        CFG_BOOTLOADER_BEAT_PORT &= ~CFG_BOOTLOADER_BEAT_MASK;

        Delay10mSec( 25 );

        CFG_BOOTLOADER_BEAT_PORT |= CFG_BOOTLOADER_BEAT_MASK;

        Delay10mSec( 25 );

        numTimes--;
    }
    Delay10mSec( 50 );
}

void FlashBlue( int numTimes )
{
    CFG_I2C_LED_DDR         |= CFG_I2C_LED_MASK;

    Delay10mSec( 50 );

    while ( numTimes > 0 )
    {
        CFG_I2C_LED_PORT &= ~CFG_I2C_LED_MASK;

        Delay10mSec( 25 );

        CFG_I2C_LED_PORT |= CFG_I2C_LED_MASK;

        Delay10mSec( 25 );

        numTimes--;
    }

    Delay10mSec( 50 );
}

#define PutStr( s )     PutStr_P( PSTR( s ) )

void PutStr_P( const prog_char *str )
{
    uint8_t ch;

    while (( ch = pgm_read_byte_far( str )) != '\0' )
    {
        if ( ch == '\n' )
        {
            UART0_PutChar( '\r' );
        }
        UART0_PutChar( ch );

        str++;
    }

} // PutStr_P
#endif

//***************************************************************************
/**
*   BootLoader
*/

void BootLoader( int mode )
{
    uint8_t beats;
    uint8_t bootDelay;
    uint8_t beatRate;

#if BOOT_LOG_ENABLED
    FILE   *logFs;
#endif

    uint8_t beatCounter = 0;

#if defined( CFG_FORCE_MASK )

    // We configure the pullup as early as we can, to give the pin a chance
    // to respond.

    CFG_FORCE_DDR  &= ~CFG_FORCE_MASK;  // Configure pin for input
    CFG_FORCE_PORT |= CFG_FORCE_MASK;   // Enable Pullup
#endif

    cli();

#if ( !STANDALONE )

    // For the BootLoader, we want the interrupts to goto the BootLoader
    // interrupt vector. Do the special dance.

#if defined( GICR )
    GICR = ( 1 << IVCE );
    GICR = ( 1 << IVSEL );
#else
    MCUCR = ( 1 << IVCE );
    MCUCR = ( 1 << IVSEL );
#endif
#endif // STANDALONE

#if CFG_USE_UART && 0
    InitHardware();
#else
    InitTimer();
#endif

#if BOOT_LOG_ENABLED
    logFs = fdevopen( UART0_PutCharStdio, NULL );

    LogInit( logFs );
#endif

#if defined( CFG_I2C_LED_MASK )
    CFG_I2C_LED_DDR         |= CFG_I2C_LED_MASK;
#endif
#if defined( CFG_BOOTLOADER_BEAT_MASK )
    CFG_BOOTLOADER_BEAT_DDR |= CFG_BOOTLOADER_BEAT_MASK;
#endif

    // Read our i2c address from the EEPROM

    eeprom_read_block( &gEeprom, (void *)( E2END + 1 - sizeof( gEeprom )), sizeof( gEeprom ));

    // Check to see if the FORCE strap has been installed

    if (( gEeprom.structSize == 0xff ) || ( gEeprom.i2cAddr > 0x7f ))
    {
        // EEPROM is uninitialized. Use default data

        gEeprom.i2cAddr = BL_DEFAULT_I2C_ADDR;
    }
    if (( gEeprom.structSize == 0xff ) || ( gEeprom.bootDelay == 0xff ))
    {
        gEeprom.bootDelay = BL_DEFAULT_BOOT_DELAY;
    }

    I2C_SlaveInit( &gI2cGlobals, gEeprom.i2cAddr, ProcessCommand );

    memset( &gBootLoaderGlobals, 0, sizeof( gBootLoaderGlobals ));
    gBootLoaderGlobals.m_pageAddress = ~0uL;

    // Set gStayInBootLoader to zero before we enable interripts. We do this
    // since the interrupt handler will set it to 1 if an Info command is received.

    gStayInBootloader = 0;
    if (( pgm_read_byte_near( 0x0000 ) == 0xFF ) || ( mode == BOOT_MODE_APP ))
    {
        // No user mode program has been downloaded. We really have no choice but to stay in
        // the bootloader.

        gStayInBootloader = 1;
    }

    sei();

    BOOT_LOG0( "\n" );
    BOOT_LOG0( "*****\n" );
    BOOT_LOG2( "***** I2C BootLoader i2cAddr: 0x%02x bootDelay:%d\n", gEeprom.i2cAddr, gEeprom.bootDelay );
    BOOT_LOG0( "*****\n" );

    bootDelay = gEeprom.bootDelay;
    beats = 0;

    // gStayInBootloader is set if the BL_CMD_GET_INFO packet is received or 
    // no user program is found in flash.

    beatRate = 100; // Toggle every 100 msec

// By default, we pick Port E1 (TXD) as the override pin. For the normal bootloader
// we don't use the UART, so this is fine.

#if ( defined( CFG_FORCE_MASK ) && !CFG_USE_UART )

    // We split the test away from the configuration of the pin to give
    // things a chance to settle.   

    gStayInBootloader |= (( CFG_FORCE_PIN & CFG_FORCE_MASK ) == 0 );

#endif

    while ( gStayInBootloader || ( bootDelay > 0 ))
    {
        tick_t prevCount;

        if ( gStayInBootloader  )
        {
            beatRate = 250;
        }

        if ( ++beatCounter >= beatRate )
        {
            beatCounter = 0;

            CFG_BOOTLOADER_BEAT_PORT ^= CFG_BOOTLOADER_BEAT_MASK;

            if ( ++beats >= 10 )
            {
                // This branch taken once per second

                beats = 0;
                if ( bootDelay > 0 )
                {
                    bootDelay--;
                }
            }
        }

        prevCount = gTickCount;
        while ( gTickCount == prevCount )
        {
#if BOOT_LOG_ENABLED
            LogBufDump();
#endif
        }
    }

    RunUserProgram();

} // BootLoader

//***************************************************************************
/**
*   Run the user program
*
*/

void RunUserProgram( void )
{
#if STANDALONE

    BOOT_LOG0(  "*** RunUserProgram ***\n" );
    while ( 1 )
    {
        ;
    }

#else

    // Disable interrupts so we don't have any spurious stuff happening

    cli();

    // Turn off the I2C & timer interrupts

    TWCR = 0;
    TIMSK = 0;

    // Make sure that the interrupt vectors are back to normal

#if defined( GICR )
    GICR = ( 1 << IVCE );
    GICR = 0;
#else
    RAMPZ = 0;
    MCUCR = ( 1 << IVCE );
    MCUCR = 0;
#endif

    // Jump to the user reset vector

#if defined __AVR_ATmega8__

    asm volatile("rjmp	zero_addr\n");
#else
    asm volatile("jmp	zero_addr\n");
#endif

#endif  // STANDALONE

} // RunUserProgram

//***************************************************************************
/**
*   I2C Interrupt service routine
*
*/

SIGNAL(SIG_2WIRE_SERIAL)
{
    if ( !I2C_SlaveHandler( &gI2cGlobals ))
    {
        BOOT_LOG1( "Unrecognized status: 0x%x\n", TW_STATUS );
    }

    // Now that we've finished dealing with the interrupt, set the TWINT flag
    // which will stop the clock stretching and clear the interrupt source

    TWCR |= ( 1 << TWINT );

} // SIG_2WIRE_SERIAL

extern  uint8_t __data_start[];

//***************************************************************************
/**
*   Callback called during bootloader time, when an I2C packet has been 
*   received.
*/

int ProcessCommand( I2C_Data_t *packet )
{
    return BootLoaderProcessCommand( &gBootLoaderGlobals, packet );
}

//***************************************************************************
/**
*   Handles the bootloader commands. This function can be called either at
*   bootloader time, or from the application.
*/

const BootLoaderInfo_t PROGMEM info_P =
{
    0,                          // devAddr
    BL_API_VERSION,             // version
    BL_API_MIN_VERSION,         // minVersion
    { 0, 0, 0 },                // pad
    AVR_SIGNATURE,              // partNumber
    ( RAMEND + 1 ) & 0x01FF,    // regSize
    ( RAMEND + 1 ) & 0xFE00,    // ramSize
    E2END + 1,                  // eepromSize
    SPM_PAGESIZE,               // spmPageSize
    FLASHEND + 1                // flashSize
};

int BootLoaderProcessCommand( BootLoaderGlobals_t *globals, I2C_Data_t *packet )
{
    uint8_t cmd = packet->m_data[ 0 ];

    BOOT_LOG2( "ProcessCommand: 0x%02x, len:%d\n", cmd, packet->m_len );

    // Using explicit if tests instead of a switch statement saves us 28 bytes.

    // Note: For write requests, the length will include the CRC. m_crc
    //       will be equal to zero if the CRC passed.

    if ( cmd == BL_REG_GET_INFO )
    {
        BootLoaderInfo_t   *info = (BootLoaderInfo_t *)&packet->m_data[ 1 ];

        BOOT_LOG0( "BL_REG_GET_INFO\n" );

        memcpy_EP( info, addr32( info_P ), sizeof( *info ));

        info->devAddr       = gEeprom.i2cAddr;

        packet->m_data[ 0 ] = sizeof( *info );

        // We've received a command. Set the StayInBootLoader flag since the
        // master now has control

        gStayInBootloader = 1;

        return sizeof( *info ) + 1; // + 1 for len
    }

    if ( cmd == BL_REG_READ_DATA )
    {
        // The Read command is implemented using Process-Call-Block, so the CRC is
        // sent by us in the response we send back to the master

        BootLoaderRead_t   *req;
        BL_Addr_t           addr;
        uint8_t             numBytes;
        uint8_t             memType;
        uint8_t            *replyData;

        // Read request

        req = (BootLoaderRead_t *)&packet->m_data[ 2 ] ;        // +1 for cmd, +1 for len

        addr.byte[ 0 ] = req->addr.byte[ 0 ];
        addr.byte[ 1 ] = req->addr.byte[ 1 ];
        addr.byte[ 2 ] = req->addr.byte[ 2 ];
        addr.byte[ 3 ] = 0;
        memType  = req->addr.byte[ 3 ] & BL_MEM_TYPE_MASK;
        numBytes = req->len;

        if ( numBytes > BL_MAX_DATA_BYTES )
        {
            numBytes = BL_MAX_DATA_BYTES;
        }

        BOOT_LOG1( "BL_REG_READ_DATA: Reading %d bytes\n", numBytes );
        BOOT_LOG2( "BL_REG_READ_DATA: memAddr of 0x%02x%02x\n", addr.byte[ 1 ], addr.byte[ 0 ]);

        packet->m_data[ 0 ] = numBytes;

        replyData = &packet->m_data[ 1 ];   // +1 for len

        if ( memType == BL_MEM_TYPE_FLASH )
        {
            memcpy_EP( replyData, addr.val32, numBytes );
        }
        else
        if ( memType == BL_MEM_TYPE_EEPROM )
        {
            eeprom_read_block( replyData, (void *)addr.val16, numBytes );
        }
        else
        if ( memType == BL_MEM_TYPE_SRAM )
        {
            memcpy( replyData, (void *)addr.val16, numBytes );
        }
        else
        {
            return 0;
        }
        return packet->m_data[ 0 ] + 1 ;    // +1 for len
    }

    if ( cmd == BL_REG_WRITE_DATA )
    {
        I2C_CRC( if ( packet->m_crc == 0 ) )
        {
            // Write request

            BootLoaderWrite_t   *req;
            BL_Addr_t           addr;
            uint8_t             numBytes;
            uint8_t             memType;

            req = (BootLoaderWrite_t *)&packet->m_data[ 2 ];    // +1 for cmd, +1 for len

            addr.byte[ 0 ] = req->addr.byte[ 0 ];
            addr.byte[ 1 ] = req->addr.byte[ 1 ];
            addr.byte[ 2 ] = req->addr.byte[ 2 ];
            addr.byte[ 3 ] = 0;
            memType  = req->addr.byte[ 3 ] & BL_MEM_TYPE_MASK;

            numBytes = packet->m_data[ 1 ] - offsetof( BootLoaderWrite_t, data[0] );

            if ( numBytes > BL_MAX_DATA_BYTES )
            {
                numBytes = BL_MAX_DATA_BYTES;
            }

            BOOT_LOG1( "BL_REG_WRITE_DATA: Writing %d bytes\n", numBytes );
            BOOT_LOG2( "BL_REG_WRITE_DATA: memAddr of 0x%02x%02x\n", addr.byte[ 1 ], addr.byte[ 0 ]);

            if ( memType == BL_MEM_TYPE_FLASH )
            {
#if STANDALONE
                BOOT_LOG0( "Flash write faked out\n" );
#else
                WriteFlash( globals, &req->data[0], addr.val32, numBytes );
#endif
            }
            else
            if ( memType == BL_MEM_TYPE_EEPROM )
            {
                eeprom_write_block( &req->data[0], (void *)addr.val16, numBytes );
            }
            else
            if ( memType == BL_MEM_TYPE_SRAM )
            {
                memcpy( (void *)addr.val16, &req->data[0], numBytes );
            }

            return 0;
        }
    }

    if ( cmd == BL_CMD_RUN_APP )
    {
        I2C_CRC( if ( packet->m_crc == 0 ))
        {
            BOOT_LOG0( "BL_CMD_RUN_APP\n" );
    
            RunUserProgram();   // Doesn't return
        }
    }

    if ( cmd == BL_CMD_REBOOT )
    {
        I2C_CRC( if ( packet->m_crc == 0 ))
        {
            cli();

            // Use the watchdog timer to reset us

            WDTCR = ( 1 << WDCE ) | ( 1 << WDE );
            WDTCR = ( 1 << WDE );

            for (;;);
        }
    }

    BOOT_LOG1( "Unrecognized command: 0x%02x\n", cmd );

    return -1;

} // ProcessCommand

#if 0
//***************************************************************************
/**
*   Read a block of data from flash
*/

void ReadFlash( BootLoaderGlobals_t *globals, uint8_t *dst, uint32_t addr, PageSize_t numBytes )
{
    PageSize_t i;

    BOOT_LOG0( "ReadFlash\n" );
    //BOOT_LOG( "ReadFlash: addr = 0x%05lx, numBytes = 0x%x\n", addr, numBytes );

    if ( globals->m_pageDirty )
    {
        // There was data in the buffer already. Write it out.

        WriteFlashPage( globals->m_pageBuf, globals->m_pageAddress );
        globals->m_pageDirty = 0;
    }

    for ( i = 0; i < numBytes; i++ ) 
    {
#if defined( __AVR_ATmega128__ )
        *dst++ = pgm_read_byte_far( addr );
#else
        *dst++ = pgm_read_byte( addr );
#endif
        addr++;
    }

} // ReadFlash
#endif

//***************************************************************************
/**
*   Writes a page into flash
*/

void WriteFlashPage( uint8_t *src, uint32_t addr )
{
#if STANDALONE
    BOOT_LOG0( "WritePageFlash\n" );
    //BOOT_LOG( "WritePageFlash: addr = 0x%05lx\n", addr );
#else
    uint16_t   *srcWord = (uint16_t *)src;
    uint32_t    startAddr = addr; 
    uint32_t    endAddr = startAddr + SPM_PAGESIZE;
    uint8_t     sreg;

    // Disable interrpts

    sreg = SREG;
    cli();

    // The datasheet recommends waiting for any eeprom operations to be complete

    eeprom_busy_wait();

    // Erase the page, and wait for the erase to complete

    boot_page_erase( addr );
    boot_spm_busy_wait();

    // Write data to buffer a word at a time. Note incrementing address
    // by 2. SPM_PAGESIZE is defined in the microprocessor IO header file.

    while ( addr < endAddr )
    {
        boot_page_fill( addr, *srcWord++ );

        addr += 2;
    }

    // Write page.

    boot_page_write( startAddr );
    boot_spm_busy_wait();

    // Reenable RWW-section again. We need this if we want to jump back
    // to the application after bootloading.

    boot_rww_enable ();

    // Restore interrupts

    SREG = sreg;
#endif

} // WriteFlashPage

//***************************************************************************
/**
*   Write a block of data to flash, utilizing the page cache.
*/

void WriteFlash( BootLoaderGlobals_t *globals, uint8_t *src, uint32_t addr, uint8_t numBytes )
{
    BOOT_LOG0( "WriteFlash\n" );
    //BOOT_LOG( "WriteFlash: addr = 0x%05lx, numBytes = 0x%x\n", addr, numBytes );

    if ((( addr & SPM_PAGEMASK ) != globals->m_pageAddress ) || ( numBytes == 0 ))
    {
        if ( globals->m_pageDirty )
        {
            // There was data in the buffer already. Write it out.

            WriteFlashPage( globals->m_pageBuf, globals->m_pageAddress );
            globals->m_pageDirty = 0;
        }

        globals->m_pageAddress = addr & SPM_PAGEMASK;

        memcpy_EP( globals->m_pageBuf, globals->m_pageAddress, sizeof( globals->m_pageBuf ));
    }
    if ( numBytes > 0 )
    {
        // Assume that the input data fits entirely within a page. Commenting
        // out the following test saves us 46 bytes of flash

        //if ((( addr + numBytes - 1 ) & SPM_PAGEMASK ) == globals->m_pageAddress )
        {
            // We know that the input data fits entirely within a page

            memcpy( &globals->m_pageBuf[ addr - globals->m_pageAddress ], src, numBytes );
            globals->m_pageDirty = 1;
        }
    }

} // WriteFlash

