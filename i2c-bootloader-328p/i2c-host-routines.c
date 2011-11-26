/**********************************************************
 * Copyright (C) 2005 by Stefan Siegl <ssiegl@gmx.de>, Germany
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the Artistic License Version 2 as
 * published by Larry Wall.
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
//#include <linux/i2c.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

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

/* open /dev/i2c device and set things up to be ready for i2c bus 
 * communication.
 * returns unix file descriptor of i2c device or -1 on error
 */
int
i2c_open(int adapter_nr, int addr)
{
  int file;
  char filename[11];
  sprintf(filename, "/dev/i2c-%d", adapter_nr);

  if ((file = open(filename, O_RDWR)) < 0) {
    filename[8] = '/';
    if ((file = open(filename, O_RDWR)) < 0) {
      perror("i2c_open");
      return -1;
    }
  }

  /* When you have opened the device, you must specify with what device
   * address you want to communicate:
   */
  if (ioctl(file, I2C_SLAVE, addr) < 0) {
    perror("i2c_ioctl");
    return -1;
  }

  return file;
}


/* send the differences between passed i2c_control_t structure and the
  backuped one over the i2c bus ...
 * return 0 on success, 1 on failure.
 */
int 
i2c_send_to_avr(int fd, int command, char *txdata, unsigned int datalen) 
{ 
  unsigned long funcs;
  if(ioctl(fd, I2C_FUNCS, &funcs)) {
    perror("ioctl I2C_FUNCS failed.\n");
    return 1;
  }
  //fprintf(stderr, "funcs: %lx\n", funcs);
  
  if((funcs & I2C_FUNC_I2C) != 0){
    char buf[35];
    if(datalen < 36){
      buf[0] = command;
      buf[1] = datalen;
      memcpy(&buf[2], txdata, datalen);
      
      if(write(fd, &buf[0], datalen+2) != (datalen+2)){
        //fprintf(stderr, "write err: %i\n", back);
        return 1; /* didn't work. shall we emit an error message or the caller? */
    
      }
      return 0;
    }
  }

  if((funcs & I2C_FUNC_SMBUS_WRITE_BLOCK_DATA) != 0){

    struct i2c_smbus_ioctl_data arg;
    union i2c_smbus_data data;

    arg.size = I2C_SMBUS_BLOCK_DATA;
    arg.read_write = I2C_SMBUS_WRITE;
    arg.command = command;

    //data.byte = 16;
    memcpy(&data.block[1], txdata, datalen);
    data.block[0] = datalen;

    arg.data = &data;

    if (ioctl(fd, I2C_SMBUS, &arg) < 0) {
      perror("ioctl(I2C_SMBUS)");
      //close(i2c_dev);
      return 1;
    }
    
    //fprintf(stdout, "%02x%02x%02x%02x%02x%02x\n", command, data.block[0], data.block[1], data.block[2], data.block[3],data.block[4]);
    return 0;
  }
  
  return 1; /* umpf */
}



/* read the data structure from the connected avr
 * return 0 on success, 1 on failure
 */
int
i2c_recv_from_avr(int fd, int command, char *rxdata, unsigned int *recvlen)
{
  unsigned long funcs;
  if(ioctl(fd, I2C_FUNCS, &funcs)) {
    perror("ioctl I2C_FUNCS failed.\n");
    return 1;
  }
  //fprintf(stderr, "funcs: %lx\n", funcs);
  
  if((funcs & I2C_FUNC_I2C) != 0){
    char wbuf[1];
    char rbuf[35];
    wbuf[0] = command;
    if(write(fd, &wbuf[0], 1) != 1){
        //fprintf(stderr, "write err: %i\n", back);
      return 1; /* didn't work. shall we emit an error message or the caller? */
    }
    if(read(fd, &rbuf[0], 33) != 33)
      return 1; /* stop. */
    *recvlen = (int) rbuf[0];
    memcpy(rxdata, &rbuf[1], 32);
    /* debug write(2,((unsigned char *)stat),len);*/
    return 0; /* umpf */
  }
  
  if((funcs & I2C_FUNC_SMBUS_READ_BLOCK_DATA) != 0){

    struct i2c_smbus_ioctl_data arg;
    union i2c_smbus_data data;

    arg.size = I2C_SMBUS_BLOCK_DATA;
    arg.read_write = I2C_SMBUS_READ;
    arg.command = command;

    //data.byte = 16;
    //memcpy(&data.block[1], txdata, datalen);
    //data.block[0] = 1;

    arg.data = &data;

    if (ioctl(fd, I2C_SMBUS, &arg) < 0) {
      perror("ioctl(I2C_SMBUS)");
      //close(i2c_dev);
      return 1;
    }
    *recvlen = data.block[0];
    memcpy(rxdata, &data.block[1], *recvlen);
    //fprintf(stdout, "%02x %02x %02x %02x %02x %02x\n", data.block[0], data.block[1], data.block[2], data.block[3],data.block[4],data.block[5]);
    return 0;
  }

  if((funcs & I2C_FUNC_SMBUS_READ_I2C_BLOCK) != 0){

    struct i2c_smbus_ioctl_data arg;
    union i2c_smbus_data data;

    arg.size = I2C_SMBUS_I2C_BLOCK_DATA;//I2C_SMBUS_BLOCK_DATA;
    arg.read_write = I2C_SMBUS_READ;
    arg.command = command;

    //data.byte = 16;
    //memcpy(&data.block[1], txdata, datalen);
    //data.block[0] = 1;

    arg.data = &data;

    if (ioctl(fd, I2C_SMBUS, &arg) < 0) {
      perror("ioctl(I2C_SMBUS)");
      //close(i2c_dev);
      return 1;
    }
    *recvlen = data.block[1];
    memcpy(rxdata, &data.block[2], *recvlen);
    //fprintf(stdout, "%02x %02x %02x %02x %02x %02x\n", data.block[0], data.block[1], data.block[2], data.block[3],data.block[4],data.block[5]);
    return 0;
  }

  return 1; /* umpf */


}



