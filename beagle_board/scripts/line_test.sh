#!/bin/bash

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
   i2c -d /dev/i2c-2 0x10 wb4 0 0x00000000
   exit
}

i2c -d /dev/i2c-2 0x10 wb4 0 0x6000F8FF
sleep 10
i2c -d /dev/i2c-2 0x10 wb4 0 0x00000000
