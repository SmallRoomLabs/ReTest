//
// Copyright (c) 2016 SmallRoomLabs / Mats Engstrom - mats.engstrom@gmail.com
// 
// Hardware and Firmware released under the MIT License (MIT) that can be
// found in full in the file named LICENCE in the git repository.
//

#include "spi.h"

//
//
//
void SpiInit() {
  DDRB=(1<<5)|(1<<3)|(1<<2); // Set MOSI, SCK and SS as output
  SPCR=(1<<SPE)|(1<<MSTR)|(1<<SPR0);
}


//
//
//
uint8_t SpiSend(uint8_t data) {
  SPDR=data;
  while (!(SPSR & (1<<SPIF)));
  return SPDR;
}

