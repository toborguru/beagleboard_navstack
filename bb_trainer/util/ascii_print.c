/** @file 
 *  Functions to output more user friendly representations.
 *
 *  Created:    03/10/2011 11:11:16 PM
 *  Compiler:   gcc
 *  @author:    Sawyer Larkin (SJL), DATAProject@therobotguy.com
 *
 *  Last Changed:   $id$
 *
 */

/* #####   HEADER FILE INCLUDES   ################################################### */
#include <inttypes.h>
#include <stdio.h>

#include "ascii_print.h"

/* #####   MACROS  -  LOCAL TO THIS SOURCE FILE   ################################### */

/* #####   TYPE DEFINITIONS  -  LOCAL TO THIS SOURCE FILE   ######################### */

/* #####   DATA TYPES  -  LOCAL TO THIS SOURCE FILE   ############################### */

/* #####   VARIABLES  -  LOCAL TO THIS SOURCE FILE   ################################ */

/* #####   PROTOTYPES  -  LOCAL TO THIS SOURCE FILE   ############################### */

/* #####   FUNCTION DEFINITIONS  -  EXPORTED FUNCTIONS   ############################ */

/** Writes a c-style string into @a *ascii_out, which contains an ascii 
 *  hexidecimal representation of @a data_length values present in @a *byte_buf.
 *
 *  If @a *format is not NULL it will be used as the printf format string. 
 *  Please note that the format string must include a single decimal parameter. 
 *  If @a *format is NULL the format string *  this format string will be used: 
 *  <tt>"0x%0.2X "</tt>
 *
 *  @a *ascii_out must be pre-allocated and large enough to hold the ascii text 
 *  string. If @a *format is NULL and the default format string is used, 
 *  @a *ascii_out must be at least 5 times larger than data-length, plus a 
 *  single byte used for c-string termination.
 *
 */
    int
Bin_to_AsciiHex ( char ascii_out[], int data_length, uint8_t byte_buf[], char format[] )
{
    const char default_format[] = {"0x%0.2X "};
    int num_bytes = 0;

    if ((data_length > 0) && (byte_buf != NULL) && (ascii_out != NULL))
    {
        if (NULL == format)
        {
            format = (char*) &default_format;
        }

        for (int i = 0; i < data_length; i++)
        {
            num_bytes += sprintf(&ascii_out[num_bytes], format, (unsigned char)byte_buf[i]);
        }
    }

    return num_bytes;
}		/* Bin_to_AsciiHex */

/* #####   FUNCTION DEFINITIONS  -  LOCAL TO THIS SOURCE FILE   ##################### */

