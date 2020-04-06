/*
 * I2C Helper functions
 * Sawyer Larkin
 *
 */

#ifndef I2C_API_H
#define I2C_API_H

#ifdef  __cplusplus
extern "C" {
#endif

// ---- Include Files -------------------------------------------------------

#include <inttypes.h>
#include <linux/i2c-dev.h>

// ---- Constants and Types -------------------------------------------------

#define I2C_MAX_DATA_LEN  255

// ---- Variable Externs ----------------------------------------------------

// ---- Function Prototypes -------------------------------------------------

// Returns -1 for errors and number of bytes read otherwise
int i2c_write_read_transfer
(
    int             i2c_fd,     // handle to i2c-dev file
    uint8_t         i2c_board,  // i2c address of the board
    const uint8_t*  write_data, // data to write
    int             write_len,  // number of bytes to write
    uint8_t*        read_data,  // place to store data read
    int             read_len    // number of bytes to read
);

int i2c_read_byte
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint8_t*    read_byte     // place to store byte read
);

int i2c_read_2bytes
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint16_t*   read_data     // place to store data read
);

int i2c_read_4bytes
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint32_t*   read_data     // place to store data read
);

int i2c_read_bytes
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint8_t*    read_data,    // place to store data read
    int         read_len      // number of bytes to read
);

int i2c_write_byte
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint8_t     write_byte    // place to store byte to write
);

int i2c_write_2bytes
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint16_t    write_data    // place to store data to write
);

int i2c_write_4bytes
(
    int         i2c_fd,       // handle to i2c-dev file
    uint8_t     i2c_board,    // i2c address of the board
    uint8_t     reg_addr,     // starting register address offset
    uint32_t    write_data    // place to store data to write
);

int i2c_write_bytes
(
    int             i2c_fd,     // handle to i2c-dev file
    uint8_t         i2c_board,  // i2c address of the board
    uint8_t         reg_addr,   // starting register address offset
    const uint8_t*  write_data, // place to store data read
    int             write_len   // number of bytes to read
);

#ifdef  __cplusplus
}
#endif

#endif  // I2C_API_H

