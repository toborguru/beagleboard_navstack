/** @file checksum_test.c
 *  This program will run a unit test on the checksum module.
 *
 *  Created:    03/10/2011 12:27:19 AM
 *  Compiler:   gcc
 *  @author:    Sawyer Larkin (SJL), DATAProject@therobotguy.com
 *
 *  Last Changed:   $id$
 *
 */

#include <cstdio>
#include <gtest/gtest.h>

#include "checksum.h"

#define TEST_DATA_SIZE 128

TEST( ChecksumTest, expectedChecksums )
{
    char test_data[TEST_DATA_SIZE];
    uint8_t checksum1;
    uint8_t checksum2;
    uint8_t checksum3;
    uint8_t checksum4;

    sprintf(test_data, "testing, testing, 1 2 3\n");

    // CRC 0-3 "test"
    checksum1 = CRC8_Checksum((uint8_t*)test_data, 4, 0);

    // CRC 9-13 "test"
    checksum2 = CRC8_Checksum((uint8_t*)&test_data[9], 4, 0);

    // CRC 1-4 "esti"
    checksum3 = CRC8_Checksum((uint8_t*)&test_data[1], 4, 0);

    // CRC 10-14 "esti"
    checksum4 = CRC8_Checksum((uint8_t*)&test_data[10], 4, 0);

    EXPECT_EQ( checksum1, checksum2 );
    EXPECT_EQ( 0x4C, checksum1 );
    EXPECT_EQ( checksum3, checksum4 );
    EXPECT_EQ( 0x6E, checksum3 );
}
