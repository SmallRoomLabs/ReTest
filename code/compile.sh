#!/bin/bash

MCU=atmega328p

# avr-gcc -c $1.c -Os -mmcu=$MCU -D F_CPU=16000000 #-I /usr/local/avr/include/
# avr-gcc -g -mmcu=$MCU -o $1.elf $1.o
# avr-objcopy -j .text -j .data -O ihex $1.elf $1.hex

avrdude -q -q -p$MCU -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:$1.hex:i
