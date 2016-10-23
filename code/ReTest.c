//
// Copyright (c) 2016 SmallRoomLabs / Mats Engstrom - mats.engstrom@gmail.com
// 
// Hardware and Firmware released under the MIT License (MIT) than can be
// found in full in the file named LICENCE in the git repository.
//

#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "spi.h"
#include "lcd.h"
#include "rotaryenc.h"

//#include "fonts/FreeSans9pt7b.h"
#include "fonts/FreeSansBold9pt7b.h"
    
//
// D8   PB0 
// D9   PB1 
// D10  PB2 SPI SS   > SPI Select for SD
// D11  PB3 SPI MOSI > SPI Data Out for LCD & SD
// D12  PB4 SPI MISO < SPI Data In for SD
// D13  PB5 SPI SCK  > SPI Clock for LCD & SD
// PB6  XTAL1          ---
// PB7  XTAL2          ---
// A0   PC0          > Relay coil
// A1   PC1          < Button 1
// A2   PC2          < Button 2
// A3   PC3          < Encoder Button
// A4   PC4          < Encoder Quadrature B
// A5   PC5          < Encoder Quadrature A
// A6   ADC6
// A7   ADC7
// A6   PC6 RESET      ---
// D0   PD0 UART RX  < USB/Serial converter
// D1   PD1 UART TX  > USB/Serial converter
// D2   PD2          > LCD Control line D/C
// D3   PD3          > LCD SPI Enable (CE)
// D4   PD4          > LCD Reset
// D5   PD5 
// D6   PD6          < Relay NO pin
// D7   PD7          < Relay NC pin
//

#define RELAY_COIL_PIN    PC0
#define BUT1_STATUS       !(PINC & (1<<PC1))    // Push button
#define BUT2_STATUS       !(PINC & (1<<PC2))    // Push button
#define QUAD_BUTTON       !(PINC & (1<<PC3))    // Encoder button
#define QUAD_B_STATUS     !(PINC & (1<<PC4))    // Encoder Out1
#define QUAD_A_STATUS     !(PINC & (1<<PC5))    // Encoder Out2
#define RELAY_NONC_STATUS (PIND & ((1<<PD6)|(1<<PD7)))

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

volatile uint16_t tickMs;         // RTC updated by Timer2
volatile uint16_t encValue;       // Rotary Encoder value upd by Timer2
uint16_t encMin,encMax;           // Min & Max limits for Rotary Encoder
volatile uint16_t speed;
#define ENC_THRESHOLD       2     // Threshold for fast turning
#define ENC_SPEEDMULTIPLIER 20    // Increment when turning fast

//
// TimerCounter0 Compare Match A
// Samples the Relay NC/NO pins every 10uS and puts
// a timestamp in the relay sample/event table if
// any change in the pins are detected
//
ISR (TIMER0_COMPA_vect) {  
  static uint16_t tick;
  static uint8_t oldPins=0;
  uint8_t pins;

  if (sampleCnt==512) {
    tick=0;
  } else {
    pins=RELAY_NONC_STATUS;
    if (pins!=oldPins) {
      oldPins=pins;
      samples[sampleCnt]=tick|(pins<<14);
      sampleCnt++;
    }
    tick++;
  }
}


//
// TimerCounter2 Compare Match A
//
ISR (TIMER2_COMPA_vect, ISR_NOBLOCK) {  
    uint8_t dir;
    static uint16_t lastValue;
    static uint8_t step=1;

    dir=PollRotaryEncoder(QUAD_A_STATUS, QUAD_B_STATUS);
    if (dir==DIR_CCW) {
        if (encValue+step<encMax) {
            encValue+=step;
        } else {
            encValue=encMax;
        }
    }
    if (dir==DIR_CW) {
        if ((encValue-step>encMin) && (encValue>step)) {
            encValue-=step;
        } else {
            encValue=encMin;
        }
    }

    tickMs++;
    // 256 ms has passed, check is user is turning the knob fast
    if (!(tickMs&0xFF)) {
      speed=abs(lastValue-encValue);
      if (speed>ENC_THRESHOLD*step) {
        step=ENC_SPEEDMULTIPLIER;
      } else {
        step=1;
      }
      lastValue=encValue;
    }
}


int main(void) {

  // Setup Timer0 to generate 10us interrupts
  // used for sampling relay
  TCCR0A |= (1 << WGM01);   // Timer Mode CTC
  OCR0A = 160;              // 16MHz/160 ticks = 10us
  TIMSK0 |= (1 << OCIE0A);  // Set the ISR COMPA vect
  TCCR0B |= (1 << CS00);    // No Prescaler, Start the timer

  // Setup Timer2 to generate 1ms interrupts
  // used for RTC and user interface
  TCCR2A |= (1 << WGM21);   // Timer Mode CTC
  OCR2A = 250;              // 16MHz/(250*64) ticks = 1ms
  TIMSK2 |= (1 << OCIE2A);  // Set the ISR COMPA vect
  TCCR2B |= (1 << CS22);    // Prescale /64, Start the timer

  sei();                    // Enable interrupts
  
  DDRC=(1<<PC0);  // PC0 drives the relay (inverted)
  PORTC=0x3F;     // Turns on pullup on PC1..PC5 and deactivates Relay 

  encMin=10;
  encMax=10000;
  encValue=1000;

  SpiInit();
  LcdInit(40);
  LcdSetFont(NULL);
  LcdSetFont(&FreeSansBold9pt7b);
  setTextSize(1);
  LcdRefresh();
  _delay_ms(500);
  LcdClear();

  for(;;) {
    LcdClear();
    char s[10];
    setTextSize(1);
    setTextColor(BLACK,WHITE);
    setCursor(0,15);
    sprintf(s,"%u ms",encValue);
    LcdPrint(s);


    LcdRefresh();
    _delay_ms(100);
  }

}


