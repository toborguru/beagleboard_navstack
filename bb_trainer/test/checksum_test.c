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

#include "../include/checksum.h"

    int
main ( int argc, char *argv[] )
{
    char test_data[128];
    uint8_t checksum;

    sprintf(test_data, "testing, testing, 1 2 3\n");

    printf("Echo: %s", test_data);

    checksum = CRC8_Checksum((uint8_t*)test_data, 4, 0);
    printf("Chk  3:0 : %02X\n", checksum);

    checksum = CRC8_Checksum((uint8_t*)&test_data[9], 4, 0);
    printf("Chk 13:9 : %02X\n", checksum);

    checksum = CRC8_Checksum((uint8_t*)&test_data[1], 4, 0);
    printf("Chk  4:1 : %02X\n", checksum);

    checksum = CRC8_Checksum((uint8_t*)&test_data[10], 4, 0);
    printf("Chk 14:10: %02X\n", checksum);

    printf("Echo: %s", test_data);

    return 0;
}		/* main */
