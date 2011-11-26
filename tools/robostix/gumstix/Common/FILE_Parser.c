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
*   @file   FILE_Parser.c
*
*   @brief  This file contains the implementation for parsing hex/srecord files.
*
****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

#include "FILE_Parser.h"

// ---- Public Variables ----------------------------------------------------
// ---- Private Constants and Types -----------------------------------------

#define TRUE    1
#define FALSE   0

#define SUPPORT_INTEL_HEX       1
#define SUPPORT_MOTOROLA_SREC   1

/**
 *  Contains state information used by the parser
 */
 
typedef struct
{
    FILE_LineNum_t          lineNum;    ///< Tracks line number we're working on
    FILE_ParserCallback_t  *callback;   ///< User provided callback functions to use
    int                     isSRecord;  ///< Is the file a Motorola S-Record file?
    int                     isHexFile;  ///< Is the file an Intel Hex File?
    FILE_ParsedData_t       parsedData; ///< Last hunk of data parsed
    FILE_Addr_t             segAddr;    ///< Address of the segment in memory
    FILE_SegmentLen_t       segLen;     ///< Length of the segment in memory
    int                     inSeg;      ///< We've started a segment which hasn't been finished yet
    FILE_Addr_t             addrBase;   ///< Added to an offset to get address

} Parser_t;

// ---- Private Variables ---------------------------------------------------
// ---- Private Function Prototypes -----------------------------------------

static  uint8_t *AllocData( FILE_ParsedData_t *parsedData, FILE_ParsedLen_t reqLen );
static  void ClearData( FILE_ParsedData_t *parsedData );
static  void Error( Parser_t *parser, const char *fmt, ... );
static  int Flush( Parser_t *parser );
static  void FreeData( FILE_ParsedData_t *parsedData );
static  int GetByte( Parser_t *parser, const char **s, unsigned char *b, const char *label );
static  int GetWord( Parser_t *parser, const char **s, unsigned short *b, const char *label );
static  int GetNibble( Parser_t *parser, const char **s, unsigned char *b, const char *label );
static  int ParsedData( Parser_t *parser );
static  int ParseIHexLine( Parser_t *parser, const char *line );
static  int ParseLine( Parser_t *parser, const char *line );
static  int ParseSRecordLine( Parser_t *parser, const char *line );

// ---- Functions -----------------------------------------------------------

//***************************************************************************
/**
*   Parses a file, calling the appropriate callbacks along the way
*/

int FILE_ParseFile( const char *fileName, FILE_ParserCallback_t *callback )
{
    FILE       *fs = NULL;
    Parser_t    parser;
    char        line[ 100 ];
    int         rc = FALSE;

    memset( &parser, 0, sizeof( parser ));

    parser.lineNum = 0;
    parser.callback = callback;

    if (( fs = fopen( fileName, "rt" )) == NULL )
    {
        Error( &parser, "Unable to open file '%s' for reading", fileName );
        goto cleanup;
    }

    while ( fgets( line, sizeof( line ), fs ) != NULL )
    {
        parser.lineNum++;

        if ( !ParseLine( &parser, line ))
        {
            goto cleanup;
        }
    }
    if ( !Flush( &parser ))
    {
        goto cleanup;
    }

    // Everything went successfully

    rc = TRUE;

cleanup:

    if ( fs != NULL )
    {
        fclose( fs );
    }

    return rc;

} // FILE_ParseFile

//***************************************************************************
/**
*   Allocates (or reallocates) the data buffer
*/

uint8_t *AllocData( FILE_ParsedData_t *parsedData, FILE_ParsedLen_t reqLen )
{
    if ( reqLen <= parsedData->dataAlloc )
    {
        return parsedData->data;
    }

    FreeData( parsedData );

    if (( parsedData->data = malloc( reqLen )) != NULL )
    {
        parsedData->dataAlloc = reqLen;

        ClearData( parsedData );
    }

    return parsedData->data;

} // AllocData

//***************************************************************************
/**
*   Clears any data stored in the parsed data
*/

void ClearData( FILE_ParsedData_t *parsedData )
{
    if ( parsedData->dataAlloc > 0 )
    {
        memset( parsedData->data, 0, parsedData->dataAlloc );
    }

} // ClearData

//***************************************************************************
/**
*   Called to report an error during the parsing process.
*/

void Error( Parser_t *parser, const char *fmt, ... )
{
    FILE_ParserCallback_t   *callback = parser->callback;

    va_list  args;

    va_start( args, fmt );

    if ( callback->ParseError == NULL )
    {
        vfprintf( stderr, fmt, args );
        fprintf( stderr, "\n" );
    }
    else
    {
        callback->ParseError( callback->userData, parser->lineNum, fmt, args );
    }
    va_end( args );

} // Error

//***************************************************************************
/**
*   Flushes out any buffered data
*/

int Flush( Parser_t *parser )
{
    if ( parser->inSeg )
    {
        parser->inSeg = 0;

        if ( parser->callback->FinishSegment != NULL )
        {
            return parser->callback->FinishSegment( parser->callback->userData, 
                                                    parser->segAddr, 
                                                    parser->segLen );
        }
    }

    return TRUE;

} // Flush

//***************************************************************************
/**
*   Releases any allocated data in the parsed data buffer.
*/

void FreeData( FILE_ParsedData_t *parsedData )
{
    if ( parsedData->dataAlloc > 0 )
    {
        free( parsedData->data );
        parsedData->data = NULL;
        parsedData->dataAlloc = 0;
    }

} // FreeData

//***************************************************************************
/**
*  Parses a single byte from a string containing ASCII Hex characters.
*
*  @return  TRUE, if a byte was parsed successfully, FALSE otherwise.
*/

int GetByte
(
    Parser_t      *parser,  ///< (mod) Parser object that we're reporting about.
    const char   **s,       ///< (mod) Pointer to string. Will be advanced.
    unsigned char *b ,      ///< (out) nibble that was parsed.
    const char    *label    ///< (in)  Error string (used for reporting errors).
)
{
    unsigned char  b1, b2;

    if ( GetNibble( parser, s, &b1, label ) 
    &&   GetNibble( parser, s, &b2, label ))
    {
       *b = b1 << 4 | b2;
       return TRUE;
    }

    return FALSE;

} // GetByte

//***************************************************************************
/**
*  Parses two bytes from a string containing ASCII Hex characters.
*
*  @return  TRUE, if a byte was parsed successfully, FALSE otherwise.
*/

int GetWord
(
    Parser_t      *parser,  ///< (mod) Parser object that we're reporting about.
    const char    **s,       ///< (mod) Pointer to string. Will be advanced.
    unsigned short *b ,      ///< (out) nibble that was parsed.
    const char     *label    ///< (in)  Error string (used for reporting errors).
)
{
    unsigned char  b1, b2;

    if ( GetByte( parser, s, &b1, label ) 
    &&   GetByte( parser, s, &b2, label ))
    {
       *b = (unsigned short)b1 << 8 | b2;
       return TRUE;
    }

    return FALSE;

} // GetWord

//***************************************************************************
/**
*  Parses a single nibble from a string containing ASCII Hex characters.
*
*  @return  TRUE, if a nibble was parsed successfully, FALSE otherwise.
*/

int GetNibble
(
    Parser_t      *parser,  ///< (mod) Parser object that we're reporting about.
    const char   **s,       ///< (mod) Pointer to string. Will be advanced.
    unsigned char *b ,      ///< (out) nibble that was parsed.
    const char    *label    ///< (in)  Error string (used for reporting errors).
)
{
    char ch = **s;

    *s = *s + 1;

    if (( ch >= '0' ) && ( ch <= '9' ))
    {
       *b = ch - '0';
       return TRUE;
    }

    if (( ch >= 'A' ) && ( ch <= 'F' ))
    {
       *b = ch - 'A' + 10;
       return TRUE;
    }

    if (( ch >= 'a' ) && ( ch <= 'f' ))
    {
       *b = ch - 'a' + 10;
       return TRUE;
    }

    Error( parser, "parsing %s, expecting hex digit, found '%c'", label, ch );
    return FALSE;
            
} // GetNibble

//***************************************************************************
/**
*   Called when a data line is parsed. This routine will, 
*   in turn, call StartSegment(), Parsed(), and FinishSegment() as 
*   appropriate.
*
*   @return TRUE if parsing should continue, FALSE if it should stop.
*/

int ParsedData( Parser_t *parser )
{
   if (( parser->inSeg ) && ( parser->parsedData.addr != ( parser->segAddr + parser->segLen )))
   {
      Flush( parser );
   }

   if ( !parser->inSeg )
   {
      parser->inSeg = TRUE;
      parser->segAddr = parser->parsedData.addr;
      parser->segLen  = 0;

      if ( parser->callback->StartSegment != NULL )
      {
          if ( !parser->callback->StartSegment( parser->callback->userData, parser->segAddr ))
          {
             return FALSE;
          }
      }
   }

   if ( !parser->callback->ParsedData( parser->callback->userData, &parser->parsedData ))
   {
      return FALSE;
   }

   parser->segLen += parser->parsedData.dataLen;

   return TRUE;

} // ParsedData

//***************************************************************************
/**
*   Parses a single line from an Intel Hex file.
*
*   The Intel Hex format looks like this:
*
*   : BC AAAA TT HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH CC <CR>
*
*   Note: Number of H's varies with data ..... this is example only.
*   Note: Spaces added for clarity
*
*   :     Start of record character
*   BC    Byte count
*   AAAA  Address to load data.  (tells receiving device where to load)
*   TT    Record type. (00=Data record, 01=End of file. No data in EOF record)
*   HH    Each H is one ASCII hex digit.  (2 hex digits=1 byte)
*   CC    Checksum of all bytes in record ( BC+AAAA+TT+HH......HH+CC=0 )
*
*   See: http://www.xess.com/faq/intelhex.pdf for the complete spec
*/

int ParseIHexLine( Parser_t *parser, const char *line )
{
    int                 i;
    FILE_ParsedData_t  *parsedData = &parser->parsedData;
    uint8_t            *data;

    ClearData( parsedData );

    if ( line[ 0 ] != ':' )
    {
        // In Intel Hex format, lines which don't start with ':' are
        // supposed to be ignored.

        return TRUE;
    }

    const char *s = &line[ 1 ];
    unsigned char  dataLen;

    if ( !GetByte( parser, &s, &dataLen, "count" ))
    {
        return FALSE;
    }

    data = AllocData( parsedData, dataLen );

    unsigned short  addr;
    if ( !GetWord( parser, &s, &addr, "addr" ))
    {
        return FALSE;
    }

    unsigned char recType;
    if ( !GetByte( parser, &s, &recType, "recType" ))
    {
        return FALSE;
    }

    unsigned char checksumCalc = dataLen + (( addr & 0xFF00 ) >> 8 ) + ( addr & 0x00FF ) + recType;

    for ( i = 0; i < dataLen; i++ ) 
    {
        if ( !GetByte( parser, &s, &data[ i ], "data" ))
        {
            return FALSE;
        }
        checksumCalc += data[ i ];
    }

    unsigned char checksumFound;

    if ( !GetByte( parser, &s, &checksumFound, "checksum" ))
    {
        return FALSE;
    }

    if ((unsigned char)( checksumCalc + checksumFound ) != 0 )
    {
        Error( parser, "found checksum 0x%02x, expecting 0x%02x", 
               checksumFound, 0 - checksumCalc );
        return FALSE;
    }

    switch ( recType )
    {
        case 0: // Data
        {
            parsedData->addr    = parser->addrBase + addr;
            parsedData->dataLen = dataLen;

            ParsedData( parser );
            break;
        }

        case 1: // EOF
        {
            Flush( parser );
            break;
        }

        case 2: // Segment
        {
            parser->addrBase = ((unsigned)data[ 0 ] << 12 ) | ((unsigned)data[ 1 ] << 4 );
            break;
        }

        case 3: // Entry Point
        {
            break;
        }

        default:
        {
            Error( parser, "Unrecognized record type: %d", recType );
            return FALSE;
        }
    }

    return TRUE;

} // ParseIHexLine

//***************************************************************************
/**
*   Parses a single line of the file.
*/

int ParseLine( Parser_t *parser, const char *line )
{
    if ( !parser->isSRecord && !parser->isHexFile )
    {
        if ( line[ 0 ] == 'S' )
        {
            parser->isSRecord = TRUE;
        }
        else
        if ( line[ 0 ] == ':' )
        {
            parser->isHexFile = TRUE;
        }
    }

    if ( parser->isSRecord )
    {
        return ParseSRecordLine( parser, line );
    }

    if ( parser->isHexFile )
    {
        return ParseIHexLine( parser, line );
    }

    // Not something we recognize. Treat the line as a comment and ignore it

    return TRUE;

} // ParseLine

//***************************************************************************
/**
*   Parses a single line of an S-Record file.
*/

int ParseSRecordLine( Parser_t *parser, const char *line )
{
    int                 i;
    FILE_ParsedData_t  *parsedData = &parser->parsedData;
    uint8_t            *data;

    ClearData( parsedData );

    if ( line[ 0 ] != 'S' )
    {
        Error( parser, "doesn't start with an 'S'" );
        return FALSE;
    }

    if ( !isdigit( line[ 1 ] ))
    {
        Error( parser, "expecting digit (0-9), found: '%c'", line[ 1 ]);
        return FALSE;
    }

    const char *s = &line[ 2 ];
    unsigned char  lineLen;

    if ( !GetByte( parser, &s, &lineLen, "count" ))
    {
        return FALSE;
    }

    data = AllocData( parsedData, lineLen );

    unsigned char checksumCalc = lineLen;

    for ( i = 0; i < ( lineLen - 1 ); i++ ) 
    {
        if ( !GetByte( parser, &s, &data[ i ], "data" ))
        {
            return FALSE;
        }
        checksumCalc += data[ i ];
    }
    checksumCalc = ~checksumCalc;

    unsigned char checksumFound;

    if ( !GetByte( parser, &s, &checksumFound, "checksum" ))
    {
        return FALSE;
    }

    if ( checksumFound != checksumCalc )
    {
        Error( parser, "found checksum 0x%02x, expecting 0x%02x", 
               checksumFound, checksumCalc );
        return FALSE;
    }

    switch ( line[ 1 ] )
    {
        case '0':
        {
            // Header Record - We ignore this for now

            Flush( parser );
            break;
        }

        case '1':   // 2 byte address
        case '2':   // 3 byte address
        case '3':   // 4 byte address
        {
            int addrIdx;

            int addrLen			= line[ 1 ] - '1' + 2;

            unsigned char *x = data;

            for ( addrIdx = 0; addrIdx < addrLen; addrIdx++ ) 
            {
                parsedData->addr <<= 8;
                parsedData->addr += *x++;
            }
            parsedData->dataLen = lineLen - addrLen - 1;
            memmove( data, x, parsedData->dataLen );

            ParsedData( parser );
            break;
        }

        case '5':   // Count of S1, S2, and S3 records
        {
            Flush( parser );
            break;
        }

        case '7':   // 2 byte address
        case '8':   // 3 byte address
        case '9':   // 4 byte address
        {
            // Start Address - currently ignored

            Flush( parser );
            break;
        }

        default:
        {
            Error( parser, "Unrecognized S-Record: S%c", line[ 1 ] );
            return FALSE;
        }
    }

    return TRUE;

} // ParseSRecordLine


