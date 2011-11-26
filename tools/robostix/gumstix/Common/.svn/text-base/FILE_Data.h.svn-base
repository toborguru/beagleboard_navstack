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
*   @file   FILE_Data.h
*
*   @brief  This file describes the interface for reading hex/srecord files.
*
****************************************************************************/

#if !defined( FILE_DATA_H )
#define FILE_DATA_H                   ///< Include Guard

// ---- Include Files -------------------------------------------------------

#include <inttypes.h>
#include "FILE_Parser.h"

// ---- Constants and Types -------------------------------------------------

#define FILE_BLOCK_SIZE 1024

typedef uint32_t    FILE_BlockLen_t;

//----------------------------------------------------------------------------

struct FILE_Block_s;
typedef struct FILE_Block_s FILE_Block_t;

struct FILE_Block_s
{
    FILE_Addr_t     address;        ///< Address of the first byte of this block
    FILE_BlockLen_t blockLen;       ///< Number of bytes of data in this block
    uint8_t        *data;           ///< Data in the block.

    FILE_Block_t   *next;           ///< Next block in the chain
};

typedef struct
{
    char           *fileName;       ///< Name of the file the data came from

    FILE_Block_t   *head;           ///< Pointer to the first block of data
    FILE_Block_t   *tail;           ///< Pointer to the last block of data

} FILE_Data_t;

// ---- Functions -----------------------------------------------------------

#if defined( __cplusplus )
extern "C"
{
#endif

FILE_Data_t *FILE_ReadData( const char *fileName );

void FILE_FreeData( FILE_Data_t *fileData );

#if defined( __cplusplus )
}
#endif

/** @} */

#endif  // FILE_DATA_H

