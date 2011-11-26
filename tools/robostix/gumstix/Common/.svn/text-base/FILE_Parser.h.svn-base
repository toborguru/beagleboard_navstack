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
*   @file   FILE_Parser.h
*
*   @brief  This file describes the interface for parsing hex/srecord files.
*
****************************************************************************/

#if !defined( FILE_PARSER_H )
#define FILE_PARSER_H                   ///< Include Guard

// ---- Include Files -------------------------------------------------------

#include <stdarg.h>
#include <inttypes.h>

// ---- Constants and Types -------------------------------------------------

typedef uint32_t    FILE_Addr_t;
typedef uint32_t    FILE_SegmentLen_t;
typedef unsigned    FILE_LineNum_t;
typedef uint8_t     FILE_ParsedLen_t;

typedef struct
{
    FILE_LineNum_t      lineNum;    ///< Line number of data record in ASCII file.
    FILE_Addr_t         addr;       ///< Address that was parsed.
    FILE_ParsedLen_t    dataLen;    ///< Number of bytes of data.
    FILE_ParsedLen_t    dataAlloc;  ///< Number of bytes allocated.
    uint8_t            *data;       ///< Data that was parsed.

} FILE_ParsedData_t;

//----------------------------------------------------------------------------

struct  FILE_ParserCallback_s;
typedef struct  FILE_ParserCallback_s   FILE_ParserCallback_t;

struct FILE_ParserCallback_s
{
    void           *userData;   ///< Passed to callback functions.

    //------------------------------------------------------------------------
    /**
    *   Called at the beginning of a segment. A segment is defined to be a
    *   contiguous region of bytes parsed from the file.
    * 
    *   The callback should return zero to stop parsing, non-zero to continue
    *   parsing.
    */

    int (*StartSegment)( void *userData, FILE_Addr_t addr );

    //------------------------------------------------------------------------
    /**
    *   Called with some data parsed from the file (typically one line).
    * 
    *   The callback should return zero to stop parsing, non-zero to continue
    *   parsing.
    */

    int (*ParsedData)( void *userData, FILE_ParsedData_t *parsedData );

    //------------------------------------------------------------------------
    /**
    *   Called at the end of a segment.
    * 
    *   The callback should return zero to stop parsing, non-zero to continue
    *   parsing.
    */

    int (*FinishSegment)( void *userData, FILE_Addr_t addr, FILE_SegmentLen_t segLen );

    //------------------------------------------------------------------------
    /**
    *   Called when an error is encountered while parsing the file.
    */

    void (*ParseError)( void *userData, FILE_LineNum_t lineNum, const char *fmt, va_list args );

};

// ---- Functions -----------------------------------------------------------

#if defined( __cplusplus )
extern "C"
{
#endif

int FILE_ParseFile( const char *fileName, FILE_ParserCallback_t *callback );

#if defined( __cplusplus )
}
#endif

/** @} */

#endif  // FILE_PARSER_H

