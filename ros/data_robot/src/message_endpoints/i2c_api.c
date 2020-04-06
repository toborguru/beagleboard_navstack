// Investigating I2C bus control

#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/i2c.h>

#include "i2c_api.h"

#ifndef LOG_MESSAGE
#	include <stdio.h>
#	define LOG_MESSAGE printf
#endif


int i2c_write_read_transfer
(   
    int             i2c_fd,     // handle to i2c-dev file
    uint8_t         i2c_board,  // i2c address of the board
    const uint8_t*  write_data, // data to write
    int             write_len,  // number of bytes to write
    uint8_t*        read_data,  // place to store data read
    int             read_len    // number of bytes to read
)
{
  int msg_index;

  struct  i2c_rdwr_ioctl_data rdwr;
  struct  i2c_msg             msg[ 2 ];

  char    read_buf[I2C_MAX_DATA_LEN];

  int writing = 0;

	// Check pointers
	if ( (read_data == NULL) && (read_len > 0) )
	{
		LOG_MESSAGE( "Read pointer NULL when read_len is: %d", read_len);
		errno = ENOMEM;
		return -1;
	}

	if ( (write_data == NULL) && (write_len > 0) )
	{
		LOG_MESSAGE( "Write pointer NULL when write_len is: %d", read_len);
		errno = ENOMEM;
		return -1;
	}

	// Check lengths
  if ( read_len > I2C_MAX_DATA_LEN )
  {
    LOG_MESSAGE( "Read too big: %d, max is %d\n", read_len, I2C_MAX_DATA_LEN );
    errno = ENOBUFS;
    return -1;
  }

  if ( write_len > I2C_MAX_DATA_LEN )
  {
    LOG_MESSAGE( "Write too big: %d, max is %d\n", write_len, I2C_MAX_DATA_LEN );
    errno = ENOBUFS;
    return -1;
  }

// SJL  // Set I2C Board Address
// SJL	if ( ioctl( i2c_fd, I2C_SLAVE, i2c_board ) < 0 )
// SJL	{   
// SJL		LOG_MESSAGE( "Error trying to set slave address to 0x%02x (%d %s)\n",
// SJL				board_address, errno, strerror(errno) );
// SJL	}

  // Setup write section of transfer
  if ( write_len > 0 )
  {
    msg[ 0 ].addr  = i2c_board;
    msg[ 0 ].flags = 0;
    msg[ 0 ].len   = write_len;
    msg[ 0 ].buf   = (char*)&write_data[ 0 ]; 

    rdwr.nmsgs = 1;

    writing = 1;
  }

	// Setup read section of transfer
	if ( read_len > 0 )
	{
    if ( writing )
    {
      msg_index = 1;
		  rdwr.nmsgs = 2;
    }
    else
    {
      msg_index = 0;
      rdwr.nmsgs = 1;
    }

		msg[ msg_index ].addr  = i2c_board;
		msg[ msg_index ].flags = I2C_M_RD;
		msg[ msg_index ].len   = read_len;
		msg[ msg_index ].buf   = (char*)&read_buf[ 0 ];
	}

	// Start a Write/Read transaction
  rdwr.msgs = msg;

	if ( ioctl( i2c_fd, I2C_RDWR, &rdwr ) < 0 )
	{
		LOG_MESSAGE( "Transfer Failed! Brd: %d W#b: %d R#b: %d - %s (%d)\n", 
				i2c_board, write_len, read_len, strerror( errno ), errno );
		return -1;
	}

	// Copy read data
	if ( read_len > 0 )
	{
		memcpy( read_data, &read_buf[ 0 ], read_len );
	}

	return read_len;
}

int i2c_read_byte
(   
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint8_t*    read_byte     // place to store byte read
)
{
  return i2c_read_bytes(i2c_fd, i2c_board, reg_addr, read_byte, 1);
}

int i2c_read_2bytes
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint16_t*   read_data     // place to store data read
)
{
  return i2c_read_bytes(i2c_fd, i2c_board, reg_addr, (uint8_t*)read_data, 2);
}

int i2c_read_4bytes
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint32_t*   read_data     // place to store data read
)
{
  return i2c_read_bytes(i2c_fd, i2c_board, reg_addr, (uint8_t*)read_data, 4);
}

int i2c_read_bytes
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint8_t*    read_data,    // place to store data read
    int         read_len      // number of bytes to read
)
{
  return i2c_write_read_transfer( i2c_fd, i2c_board, &reg_addr, 1, read_data, read_len);		
}

int i2c_write_byte
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint8_t     write_byte    // place to store byte to write
)
{
  return i2c_write_bytes(i2c_fd, i2c_board, reg_addr, &write_byte, 1);
}

int i2c_write_2bytes
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint16_t    write_data    // place to store data to write
)
{
  return i2c_write_bytes(i2c_fd, i2c_board, reg_addr, (uint8_t*)&write_data, 2);
}

int i2c_write_4bytes
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint32_t    write_data    // place to store data to write
)
{
  return i2c_write_bytes(i2c_fd, i2c_board, reg_addr, (uint8_t*)&write_data, 4);
}

int i2c_write_bytes
(
    int             i2c_fd,     // handle to i2c-dev file
    uint8_t         i2c_board,  // i2c address of the board
    uint8_t         reg_addr,   // starting register address offset
    const uint8_t*  write_data, // place to store data read
    int             write_len   // number of bytes to read
)
{
  uint8_t write_buf [I2C_MAX_DATA_LEN];
  write_buf[0] = reg_addr;

  if ( write_len >= (I2C_MAX_DATA_LEN - 1)) // Accounts for reg_addr byte
  {
    LOG_MESSAGE( "Write too big: %d, max is %d\n", write_len, I2C_MAX_DATA_LEN );
    errno = ENOBUFS;
    return -1;
  }

  // Copy outgoing data above the reg_addr byte
  memcpy(&write_buf[1], write_data, write_len);

  return i2c_write_read_transfer( i2c_fd, i2c_board, &write_buf[0], write_len + 1, NULL, 0);		
}
