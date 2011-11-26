#!/bin/bash

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
   i2c -d /dev/i2c-2 0x10 wb4 0 0x00000000
   exit
}

while [ true ]
do
   i2c -d /dev/i2c-2 0x10 wb4 0 0x60000000

   sleep 2

   i2c -d /dev/i2c-2 0x10 wb4 0 0x60004000

   sleep 2
done
