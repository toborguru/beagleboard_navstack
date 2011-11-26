/**********************************************************
 Copyright(C) 2006 Jochen Roessner <jochen@lugrot.de>

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA */


#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
//#include <linux/i2c.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "i2c-host-routines.h"
/* we must not include kernel headers in userspace
 * #include <linux/i2c.h>
 * #include <linux/i2c-dev.h>
 *
 * therefore we define the ioctl codes here ourselves ...
 */


#define I2C_SLAVE       0x0703  /* Change slave address                 */
                                /* Attn.: Slave address is 7 or 10 bits */
#define I2C_SLAVE_FORCE 0x0706  /* Change slave address                 */
                                /* Attn.: Slave address is 7 or 10 bits */
                                /* This changes the address, even if it */
                                /* is already taken!                    */
#define I2C_TENBIT      0x0704  /* 0 for 7 bit addrs, != 0 for 10 bit   */

#define I2C_FUNCS       0x0705  /* Get the adapter functionality */
#define I2C_RDWR        0x0707  /* Combined R/W transfer (one stop only)*/
#define I2C_PEC         0x0708  /* != 0 for SMBus PEC                   */

#define I2C_SMBUS       0x0720  /* SMBus-level access */

#define I2C_SMBUS_BLOCK_MAX     32      /* As specified in SMBus standard */
#define I2C_SMBUS_I2C_BLOCK_MAX 32      /* Not specified but we use same structure */

/* smbus_access read or write markers */
#define I2C_SMBUS_READ  1
#define I2C_SMBUS_WRITE 0

/* SMBus transaction types (size parameter in the above functions)
   Note: these no longer correspond to the (arbitrary) PIIX4 internal codes! */
#define I2C_SMBUS_QUICK             0
#define I2C_SMBUS_BYTE              1
#define I2C_SMBUS_BYTE_DATA         2
#define I2C_SMBUS_WORD_DATA         3
#define I2C_SMBUS_PROC_CALL         4
#define I2C_SMBUS_BLOCK_DATA        5
#define I2C_SMBUS_I2C_BLOCK_DATA    6
#define I2C_SMBUS_BLOCK_PROC_CALL   7           /* SMBus 2.0 */
#define I2C_SMBUS_BLOCK_DATA_PEC    8           /* SMBus 2.0 */
#define I2C_SMBUS_PROC_CALL_PEC     9           /* SMBus 2.0 */
#define I2C_SMBUS_BLOCK_PROC_CALL_PEC  10       /* SMBus 2.0 */
#define I2C_SMBUS_WORD_DATA_PEC    11           /* SMBus 2.0 */



/* To determine what functionality is present */

#define I2C_FUNC_I2C                    0x00000001
#define I2C_FUNC_10BIT_ADDR             0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING      0x00000004 /* I2C_M_{REV_DIR_ADDR,NOSTART,..} */
#define I2C_FUNC_SMBUS_HWPEC_CALC       0x00000008 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_READ_WORD_DATA_PEC  0x00000800 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA_PEC 0x00001000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_PROC_CALL_PEC    0x00002000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL_PEC 0x00004000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL  0x00008000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_QUICK            0x00010000
#define I2C_FUNC_SMBUS_READ_BYTE        0x00020000
#define I2C_FUNC_SMBUS_WRITE_BYTE       0x00040000
#define I2C_FUNC_SMBUS_READ_BYTE_DATA   0x00080000
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA  0x00100000
#define I2C_FUNC_SMBUS_READ_WORD_DATA   0x00200000
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA  0x00400000
#define I2C_FUNC_SMBUS_PROC_CALL        0x00800000
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA  0x01000000
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK   0x04000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK  0x08000000 /* w/ 1-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK_2  0x10000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2 0x20000000 /* w/ 2-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA_PEC  0x40000000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC 0x80000000 /* SMBus 2.0 */

#define I2C_FUNC_SMBUS_BYTE I2C_FUNC_SMBUS_READ_BYTE | \
                            I2C_FUNC_SMBUS_WRITE_BYTE
#define I2C_FUNC_SMBUS_BYTE_DATA I2C_FUNC_SMBUS_READ_BYTE_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_BYTE_DATA
#define I2C_FUNC_SMBUS_WORD_DATA I2C_FUNC_SMBUS_READ_WORD_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_WORD_DATA
#define I2C_FUNC_SMBUS_BLOCK_DATA I2C_FUNC_SMBUS_READ_BLOCK_DATA | \
                                  I2C_FUNC_SMBUS_WRITE_BLOCK_DATA
#define I2C_FUNC_SMBUS_I2C_BLOCK I2C_FUNC_SMBUS_READ_I2C_BLOCK | \
                                  I2C_FUNC_SMBUS_WRITE_I2C_BLOCK
#define I2C_FUNC_SMBUS_I2C_BLOCK_2 I2C_FUNC_SMBUS_READ_I2C_BLOCK_2 | \
                                   I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2
#define I2C_FUNC_SMBUS_BLOCK_DATA_PEC I2C_FUNC_SMBUS_READ_BLOCK_DATA_PEC | \
                                      I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC
#define I2C_FUNC_SMBUS_WORD_DATA_PEC  I2C_FUNC_SMBUS_READ_WORD_DATA_PEC | \
                                      I2C_FUNC_SMBUS_WRITE_WORD_DATA_PEC

union i2c_smbus_data {
        __u8 byte;
        __u16 word;
        __u8 block[I2C_SMBUS_BLOCK_MAX + 3]; /* block[0] is used for length */
        /* one more for read length in block process call */
        /* and one more for PEC */
};

struct i2c_smbus_ioctl_data {
  __u8 read_write;
  __u8 command;
  __u32 size;
  union i2c_smbus_data *data;
};


int 
main(int argc, char **argv)
{
  int devicenr = 0;
  int i2caddr = 0x04;
  /*if(argc < 2){
    printf("falsche Argumentanzahl\n");
    return 1;
  }*/
  
  argv ++; /* skip first arg, which is our program name ... */
  while(*argv) {

    if(! strcmp(*argv, "-d"))
      devicenr = (int) strtoul(argv[1], NULL, 0);

    else if(! strcmp(*argv, "-i2caddr"))
      i2caddr = (int) strtoul(argv[1], NULL, 0);

    argv += 2;
  }

  /* open the i2c port */
  int fd = i2c_open(devicenr, i2caddr);
  if(fd < 0) return 1; /* i2c_open already emitted an error message */
  
  unsigned long funcs;
  if(ioctl(fd, I2C_FUNCS, &funcs)) {
    perror("ioctl I2C_FUNCS failed.\n");
    return 1;
  }
  printf("funcs: %lx\n", funcs);

  /*
  printf("kommando: %s\n", argv[1]);
  char data[20];
  unsigned int datalen = 0;
  if(!strcmp(argv[1], "b")){
    data[0] = 0x00;
    data[1] = 0xFF;
    data[2] = 0x00;
    data[3] = 0xFF;
    datalen = 4;
    printf("sende bootstart\n");
  }
  if(!strcmp(argv[1], "l")){
    data[0] = 0x00;
    data[1] = 0xFF;
    data[2] = 0x01;
    data[3] = 0x02;
    data[4] = 0x03;
    datalen = 5;
    printf("sende bootload\n");

  }

  
  if(i2c_send_to_avr(file, data[0], &data[1], datalen-1)) {
    fprintf(stderr, "couldn't successfully send data to avr. sorry.\n");
    return 1;
  }
  */
  
  return 0;
}

