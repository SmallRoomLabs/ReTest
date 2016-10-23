//
// Copyright (c) 2016 SmallRoomLabs / Mats Engstrom - mats.engstrom@gmail.com
// 
// Hardware and Firmware released under the MIT License (MIT) than can be
// found in full in the file named LICENCE in the git repository.
//

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "spi.h"
#include "lcd.h"
#include "rotaryenc.h"

//#include "fonts/TomThumb.h"
    
// Some macros that make the code more readable
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

#define BUT1 !(PINC & (1<<PC1))
#define BUT2 !(PINC & (1<<PC2))
#define BUT_A !(PINC & (1<<PC3))
#define QUAD_A !(PINC & (1<<PC4))
#define QUAD_B !(PINC & (1<<PC5))

//
// databits  ???? ???? ???? ??dd
// <<14      dd00 0000 0000 0000
// ticks     00tt tttt tttt tttt (0..16383)
// Samples[] ddtt tttt tttt tttt
//
// With a tick rate at 10uS the maximum time is 
// 16383*10us=163830us=163mS
//
volatile uint16_t samples[512];
volatile uint16_t sampleCnt;

volatile uint16_t encValue,cnt=0;
uint16_t encMin,encMax;

//
// Timer0 overflow interrupt
//
ISR (TIMER0_COMPA_vect) {  
  // static uint16_t tick;
  // static uint8_t oldPins=0;
  // uint8_t pins;

  // if (sampleCnt==512) {
  //   tick=0;
  // } else {
  //   pins=PINC;
  //   if (pins!=oldPins) {
  //     oldPins=pins;
  //     samples[sampleCnt]=tick|(pins<<14);
  //     sampleCnt++;
  //   }
  //   tick++;
  // }
    uint8_t dir;

    cnt++;
    dir=PollRotaryEncoder(QUAD_A, QUAD_B);
    if (dir==DIR_CCW) {
        if (encValue<encMax) {
            encValue++;
        } else {
            encValue=encMax;
        }
    }
    if (dir==DIR_CW) {
        if (encValue>encMin) {
            encValue--;
        } else {
            encValue=encMin;
        }
    }
}

int main(void) {

  TCCR0A |= (1 << WGM01);   // Timer Mode CTC
  OCR0A = 160;              // 16MHz/160 ticks = 10uS
  TIMSK0 |= (1 << OCIE0A);  // Set the ISR COMPA vect
//  TCCR0B |= (1 << CS00);    // No Prescaler, Start the timer
  TCCR0B |= (1 << CS00)|(1 << CS01);    // /64, Start the timer
  sei();                    // Enable interrupts
  
  DDRC=(1<<PC0);  // PC0 drives the relay (inverted)
  PORTC=0x3F;     // Turns on pullup on PC1..PC5 and deactivates Relay 
  encMin=0;
  encMax=65535;

  SpiInit();
  LcdInit(40);
//  LcdSetFont(&TomThumb);
  LcdSetFont(NULL);
  setTextSize(1);
  LcdRefresh();
  _delay_ms(500);

  char s[10];
  volatile uint8_t v;


  for(;;) {
    LcdFillRect(0,0,84,48,WHITE);
    setTextSize(2);
    setTextColor(BLACK,WHITE);
    // strcpy(s,"-----");
    // if (BUT1) s[0]='x';
    // if (BUT2) s[1]='x';
    // if (BUT_A) s[2]='x';
    // if (QUAD_A) s[3]='x';
    // if (QUAD_B) s[4]='x';
    setCursor(0,0);
    sprintf(s,"%05d",encValue);
    LcdPrint(s);

    setCursor(0,20);
    sprintf(s,"%05d",cnt);
    LcdPrint(s);

    LcdRefresh();
    _delay_ms(10);
  
  }

}


