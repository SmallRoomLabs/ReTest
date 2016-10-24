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

// #include "fonts/FreeSans9pt7b.h"
// LcdSetFont(&FreeSansBold9pt7b);
    
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

#define SLEEP(MS)     tickMs=0; while (tickMs<(MS)){}

#define RELAY_COIL_PIN    PC0
#define QUAD_BUTTON       !(PINC & (1<<PC1))    // Encoder button
#define BUT1_STATUS       !(PINC & (1<<PC3))    // Push button
#define BUT2_STATUS       !(PINC & (1<<PC2))    // Push button
#define QUAD_B_STATUS     !(PINC & (1<<PC4))    // Encoder Out1
#define QUAD_A_STATUS     !(PINC & (1<<PC5))    // Encoder Out2

#define GRAPHHEIGTH 4

#define HIGH 0x8000
#define MID  0xC000
#define LOW  0x4000

#define RELAY_NONC_STATUS (PIND & ((1<<PD6)|(1<<PD7)))

#define RELAYOFF           (PORTC|=(1<<PC0))
#define RELAYON            (PORTC&=(~(1<<PC0)))

//
// databits  ???? ???? ???? ??dd
// <<14      dd00 0000 0000 0000
// ticks     00tt tttt tttt tttt (0..16383)
// Samples[] ddtt tttt tttt tttt
//
// With a tick rate at 10uS the maximum time is 
// 16383*10us=163830us=163mS
//

#define MAXSAMPLES  255
volatile uint8_t sampleCnt;
volatile uint16_t samples1[MAXSAMPLES];
volatile uint16_t samples2[MAXSAMPLES];

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
// sampleCnt is used as a flag for "Start" and "Done" when set 
// to 0x00 and 0xFF (MAXSAMPLES) for respective function
//
ISR (TIMER0_COMPA_vect) {  
  static uint16_t tick;
  static uint8_t oldPins=0;
  uint8_t pins;
  uint16_t pins16;

  // If stopped then just exit
  if (sampleCnt==MAXSAMPLES) return;

  // If just started then store an initial reading and exit
  if (sampleCnt==0) {
    tick=0;
    pins=RELAY_NONC_STATUS;
    oldPins=pins;
    pins16=pins<<8;
    samples1[sampleCnt++]=tick|pins16;
    return;
  }

  // If not first time then increment tick, and if pins has 
  // changed since last reading then store the sample
  tick++;
  pins=RELAY_NONC_STATUS;
  if (pins!=oldPins) {
    oldPins=pins;
    pins16=pins<<8;
    samples1[sampleCnt++]=tick|pins16;
  }
  // If enough time has elapsed then signal beging ready
  if (tick>5000) { // 5000 ticks = 50 000 us = 50ms
    sampleCnt=MAXSAMPLES;
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


//
//
//
void PlotLine(uint8_t x, uint8_t yBase, uint16_t v, uint16_t lastV) {
  uint8_t yh, ym, yl;

  yh=yBase-GRAPHHEIGTH;
  ym=yBase;
  yl=yBase+GRAPHHEIGTH;

  if (v==HIGH) {
    if (v==lastV) {
      LcdSetPixel(x,yh, BLACK);
      return;
    }
    if (lastV==MID) {
      LcdDrawLine(x,yh, x,ym, BLACK);
    } else {
      LcdDrawLine(x,yh, x,yl, BLACK);
    }
    return;
  }

  if (v==MID) {
    if (v==lastV) {
      LcdSetPixel(x,ym, BLACK);
      return;      
    } 
    if (lastV==HIGH) {
      LcdDrawLine(x,ym, x,yh, BLACK);
    } else {
      LcdDrawLine(x,ym, x,yl, BLACK);
    }
    return;
  }

  if (v==LOW) {
    if (v==lastV) {
      LcdSetPixel(x,yl, BLACK);      
      return;
    }
    if (lastV==MID) {
      LcdDrawLine(x,yl, x,ym, BLACK);
    } else {
      LcdDrawLine(x,yl, x,yh, BLACK);
    }
    return;
  }
}


//
// Scans through the sample and finds out the times until the first
// and last events. I.E how long it took for the relay to start moving
// and how long it took until fully stable.
//
// Returns a suggestion for scaling factor for the plotting of data onto 
// the LCD.
//
// Also return the start and stable times in 0.1us resolution for easier
// printing.
//
uint16_t AnalyseSamples(uint16_t *samples, uint16_t *time1, uint16_t *time2) {
  uint8_t i;
  uint16_t *p;
  uint16_t maxt;
  uint16_t t;
  uint16_t factor;

  *time1=0;     // Quench compiler warning

  maxt=0;
  p=samples;
  for (i=0; i<MAXSAMPLES; i++) {
    t=(*p)&0x3FFF;
    if (i==1) *time1=t/10;
    p++;
    if (t>maxt) {
      maxt=t;
    }
  }
  *time2=maxt/10;
  factor=(maxt+maxt/10)/84;
  return factor;
}


//
//
//
void PlotGraph(uint16_t *samples, uint8_t yBase, uint16_t factor) {
  uint8_t x;
  uint16_t *p;
  uint16_t v,lastV;
  uint16_t t,t1,t2;
  uint16_t i;

  LcdDrawLine(0,yBase-GRAPHHEIGTH-1, 0,yBase+GRAPHHEIGTH+1, BLACK);
  LcdDrawLine(83,yBase-GRAPHHEIGTH-1, 83,yBase+GRAPHHEIGTH+1, BLACK);

  p=samples;
  for (i=0; i<MAXSAMPLES; i++) {
    v=(*p)&0xC000;
    if (i==0) lastV=v;
    t1=(*p)&0x3FFF;
    t2=(*(p+1))&0x3FFF;
    if (t2==0) {
      i=MAXSAMPLES; // Exits the outer loop at next iteration
      t2=83*factor; // Make sure the last value goes all the way
    }
    for (t=t1; t<t2; t++){
      x=t/factor;
      if (x>83) x=83;
      PlotLine(x,yBase,v,lastV);
      lastV=v;
    }
    p++;
  }
}


void DrawGraph(uint16_t *samples, uint8_t y, uint16_t useFactor, char *caption) {
    char s[20];
    uint16_t factor;
    uint16_t time1,time2;
    factor=AnalyseSamples((uint16_t *)samples, &time1, &time2);
    if (useFactor!=0) factor=useFactor; 
    PlotGraph(samples, y, factor);
    setCursor(0,y+8);
    sprintf(s,"%s %2d.%d/%2d.%d",caption,time1/10,time1%10,time2/10,time2%10);
    LcdPrint(s);
}

//
// Stop the sampling engine by setting the sample counter
// to its maximium value. Also stop the 10us timer
//
void StopSampler(void) {
  sampleCnt=MAXSAMPLES;
  TCCR0B &= ~((1<<CS00)|(1<<CS01)|(1<<CS02)); 
}


//
// Set sample counter to 0 to indicate a new set 
// of samples. Also start the 10us timer.
//
void StartSampler(void) {
  memset((uint16_t *)samples1,0,sizeof(samples1));
  sampleCnt=0;
  TCCR0B |= (1<<CS00);    // No Prescaler, Start the timer
}


//
//
//
int main(void) {
  // Setup Timer0 to generate 10us interrupts
  // used for sampling the relay, but don't start it
  TCCR0A |= (1 << WGM01);   // Timer Mode CTC
  OCR0A = 160;              // 16MHz/160 ticks = 10us
  TIMSK0 |= (1 << OCIE0A);  // Set the ISR COMPA vect
  TCCR0B &= ~((1<<CS00)|(1<<CS01)|(1<<CS02)); // Timer is stopped

  // Setup Timer2 to generate 1ms interrupts
  // used for RTC and user interface
  TCCR2A |= (1 << WGM21);   // Timer Mode CTC
  OCR2A = 250;              // 16MHz/(250*64) ticks = 1ms
  TIMSK2 |= (1 << OCIE2A);  // Set the ISR COMPA vect
  TCCR2B |= (1 << CS22);    // Prescale /64, Start the timer

  sei();                    // Enable interrupts
  
  DDRC=(1<<PC0);  // PC0 drives the relay (inverted)
  PORTC=0x3F;     // Turns on pullup on PC1..PC5 and deactivates Relay 
  PORTD=0XC0;     // Turns on pullup on PD6..PD7

  RELAYOFF;

  encMin=1;
  encMax=150;
  encValue=10;

  SpiInit();
  LcdInit(50);
  LcdRefresh();

//  LcdSetFont(&FreeSansBold9pt7b);
  LcdSetFont(NULL);
  setTextSize(1);
  setTextColor(BLACK,WHITE);

  for(;;) {
    LcdClear();

    SLEEP(100);
    RELAYON;
    StartSampler();
    while(sampleCnt!=MAXSAMPLES);
    StopSampler();
    DrawGraph((uint16_t *)samples1,5,encValue,"On ");

    SLEEP(100);
    RELAYOFF;
    StartSampler();
    while(sampleCnt!=MAXSAMPLES);
    DrawGraph((uint16_t *)samples1,30,encValue,"Off");

    LcdRefresh();
   }
}


