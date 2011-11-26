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
*   @file   Config.h
*
*   @brief  Global Configuration information.
*
****************************************************************************/

#if !defined( CONFIG_H )
#define CONFIG_H

#include "../i2c-BootLoader/Config-LED.h"

#if !defined( CFG_CPU_CLOCK )
#   define CFG_CPU_CLOCK   16000000L
#endif

#define CFG_LOG_ENABLED 0

#define CFG_USE_UART0   1

#define CFG_LOG_TO_BUFFER   1
#define CFG_LOG_NUM_BUFFER_ENTRIES  64
#define CFG_LOG_EXTRA_PARAMS    1

#define CFG_UART0_RX_BUFFER_SIZE    128
#define CFG_UART0_TX_BUFFER_SIZE    128
//#define CFG_UART0_LF_TO_CRLF        1

#define CFG_USE_ADC 1

#define CFG_I2C_USE_CRC     1

#define CFG_I2C_MASTER_USE_BOOTLOADER   1

#endif  // CONFIG_H


