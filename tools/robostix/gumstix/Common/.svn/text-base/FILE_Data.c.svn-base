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
*   @file   FILE_Data.c
*
*   @brief  This file provides the implementation for reading hex/srecord files.
*
****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "FILE_Data.h"

// ---- Public Variables ----------------------------------------------------
// ---- Private Constants and Types -----------------------------------------

// ---- Private Variables ---------------------------------------------------
// ---- Private Function Prototypes -----------------------------------------

static int StartSegment( void *userData, FILE_Addr_t addr );
static int ParsedData( void *userData, FILE_ParsedData_t *parsedData );

// ---- Functions -----------------------------------------------------------

//***************************************************************************
/**
*   Parses the intel hex or srecord file and places the parsed file in memory.
*/

FILE_Data_t *FILE_ReadData( const char *fileName )
{
    FILE_ParserCallback_t   callback;
    FILE_Data_t            *fileData;

    if (( fileData = calloc( 1, sizeof( *fileData ))) == NULL )
    {
        return NULL;
    }
    fileData->fileName = strdup( fileName );

    memset( &callback, 0, sizeof( callback ));

    callback.userData       = fileData;
    callback.StartSegment   = StartSegment;
    callback.ParsedData     = ParsedData;

    if ( !FILE_ParseFile( fileName, &callback ))
    {
        FILE_FreeData( fileData );
        fileData = NULL;
    }

    return fileData;

} // FILE_ReadData

//***************************************************************************
/**
*   Releases previously allocated memory.
*/

void FILE_FreeData( FILE_Data_t *fileData )
{
    FILE_Block_t    *block;

    block = fileData->head;

    while ( block != NULL )
    {
        FILE_Block_t *nextBlock = block->next;

        free( block->data );
        free( block );

        block = nextBlock;
    }

    free( fileData->fileName );
    free( fileData );

} // FILE_FreeData

//***************************************************************************
/**
*   Called at the beginning of a new segment
*/

int StartSegment( void *userData, FILE_Addr_t addr )
{
    FILE_Data_t  *fileData = userData;
    FILE_Block_t *block;

    if (( block = calloc( 1, sizeof( *block ))) == NULL )
    {
        errno = ENOMEM;
        return 0;
    }
    block->address = addr;
    block->blockLen = 0;

    if (( block->data = calloc( 1, FILE_BLOCK_SIZE )) == NULL )
    {
        errno = ENOMEM;
        return 0;
    }
    if ( fileData->tail != NULL )
    {
        fileData->tail->next = block;
    }
    if ( fileData->head == NULL )
    {
        fileData->head = block;
    }
    fileData->tail = block;

    return 1;

} // StartSegment

//***************************************************************************
/**
*   Called when a line of data has been parsed.
*/

int ParsedData( void *userData, FILE_ParsedData_t *parsedData )
{
    FILE_Data_t  *fileData = userData;
    FILE_Block_t *block    = fileData->tail;
    unsigned spaceRemaining = FILE_BLOCK_SIZE - block->blockLen;
    unsigned offset = 0;

    // Copy in as much data as will fit

    unsigned copyLen = parsedData->dataLen;
    if ( copyLen > spaceRemaining )
    {
        copyLen = spaceRemaining;
    }
    if ( copyLen > 0 )
    {
        memcpy( &block->data[ block->blockLen ], &parsedData->data[ 0 ], copyLen );
        block->blockLen += copyLen;
        offset += copyLen;
    }

    // NOTE: The following assumes that FILE_BLOCK_SIZE is bigger than what's
    //       parsed on a single line.

    if ( offset < parsedData->dataLen )
    {
        // We've filled this segment and there's still data left, allocate another

        StartSegment( fileData, parsedData->addr + offset );

        copyLen = parsedData->dataLen - offset;
        block = fileData->tail;
        memcpy( &block->data[ 0 ], &parsedData->data[ offset ], copyLen );
        block->blockLen = copyLen;
    }

    return 1;

} // ParsedData

