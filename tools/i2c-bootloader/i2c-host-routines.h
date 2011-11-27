/**********************************************************
 * Copyright (C) 2005 by Stefan Siegl <ssiegl@gmx.de>, Germany
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


#ifndef _I2C_HOST_ROUTINES
#define _I2C_HOST_ROUTINES

/* open /dev/i2c device and set things up to be ready for i2c bus 
 * communication.
 * returns unix file descriptor of i2c device or -1 on error
 */
int i2c_open(int adapter_nr, int address);

/* send the differences between passed i2c_control_t structure and the
 * backuped one over the i2c bus ...
 * return 0 on success, 1 on failure.
 */
int i2c_send_to_avr(int fd, int command, char *data, unsigned int datalen);

/* read the data structure from the connected avr
 * return 0 on success, 1 on failure
 */
int i2c_recv_from_avr(int fd, int command, char *data, unsigned int *recvlen);

#endif /* ... _I2C_HOST_ROUTINES */
