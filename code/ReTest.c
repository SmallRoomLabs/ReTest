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

#define HIGH 0x8000
#define MID  0x8000+0x4000
#define LOW  0x4000

void FakeDataOff() {
  uint16_t p;
  uint16_t t;
  uint8_t i;
  memset((void *)samples,0,sizeof(samples));
  p=0;
  t=0;
  samples[p++]=HIGH+t;

  // After a random time go middle
  t+=4*100+(rand()%(2*100));
  samples[p++]=MID+t;

  // Bounce between HIGH and MID for a while
  for (i=0; i<1+rand()%3; i++) {
    t+=10+(rand()%(100));
    samples[p++]=HIGH+t;
    t+=10+(rand()%(100));
    samples[p++]=MID+t;
  }

  // Stay at MID for a while
  t+=400+(rand()%(400));

  // Bounce between LOW and MID for a while
  for (i=0; i<2+rand()%10; i++) {
    t+=10+(rand()%(100));
    samples[p++]=LOW+t;
    t+=10+(rand()%(100));
    samples[p++]=MID+t;
  }

  // Finish up at LOW
    t+=10+(rand()%(100));
    samples[p++]=LOW+t;
  // Finish up at LOW
    t+=10+(rand()%(100));
    samples[p++]=LOW+t;
  // Finish up at LOW
    t+=10+(rand()%(100));
    samples[p++]=LOW+t;
}


void FakeDataOn() {
  uint16_t p;
  uint16_t t;
  uint8_t i;
  memset((void *)samples,0,sizeof(samples));
  p=0;
  t=0;
  samples[p++]=LOW+t;

  // After a random time go middle
  t+=4*100+(rand()%(2*100));
  samples[p++]=MID+t;

  // Bounce between LOW and MID for a while
  for (i=0; i<1+rand()%3; i++) {
    t+=10+(rand()%(100));
    samples[p++]=LOW+t;
    t+=10+(rand()%(100));
    samples[p++]=MID+t;
  }

  // Stay at MID for a while
  t+=400+(rand()%(400));

  // Bounce between HIGH and MID for a while
  for (i=0; i<2+rand()%10; i++) {
    t+=10+(rand()%(100));
    samples[p++]=HIGH+t;
    t+=10+(rand()%(100));
    samples[p++]=MID+t;
  }

  // Finish up at HIGH
    t+=10+(rand()%(100));
    samples[p++]=HIGH+t;
  // Finish up at HIGH
    t+=10+(rand()%(100));
    samples[p++]=HIGH+t;
  // Finish up at HIGH
    t+=10+(rand()%(100));
    samples[p++]=HIGH+t;
}

#define GRAPHHEIGTH 4
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
//8550

///
///
///
void PlotGraph(uint8_t yBase, uint16_t *time1, uint16_t *time2) {
  uint8_t x;
  uint16_t p;
  uint16_t v,lastV;
  uint16_t maxt;
  uint16_t t,t1,t2;
  uint16_t factor;

  maxt=0;
  for (p=0; p<512; p++) {
    t=samples[p]&0x3FFF;
    if (t>maxt) {
      maxt=t;
    }
  }
  *time1=samples[1]&0x3FFF;
  *time2=maxt;

  maxt=2500;
  factor=((maxt+maxt/5)/84);

  LcdDrawLine(0,yBase-GRAPHHEIGTH-1, 0,yBase+GRAPHHEIGTH+1, BLACK);
  LcdDrawLine(83,yBase-GRAPHHEIGTH-1, 83,yBase+GRAPHHEIGTH+1, BLACK);

  for (p=0; p<511; p++) {
    v=samples[p]&0xC000;
    if (p==0) lastV=v;
    t1=samples[p]&0x3FFF;
    t2=samples[p+1]&0x3FFF;
    if (t2==0) {
      p=512;        // Exits the outer loop at next iteration
      t2=83*factor; // Make sure the last value goes all the way
      v=lastV;
    }
    for (t=t1; t<t2; t++){
      x=t/factor;
      if (x>83) x=83;
      PlotLine(x,yBase,v,lastV);
      lastV=v;
    }
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

  LcdSetFont(NULL);
  setTextSize(1);
  setTextColor(BLACK,WHITE);

    uint16_t cnt=0;
  for(;;) {
    char s[10];
    uint16_t t1,t2;

    LcdClear();

    FakeDataOn();
    PlotGraph(5,&t1,&t2);
    setCursor(0,5+8);
    t1/=10;
    t2/=10;
    sprintf(s,"On  %2d.%d/%2d.%d",t1/10,t1%10,t2/10,t2%10);
    LcdPrint(s);

    FakeDataOff();
    PlotGraph(30,&t1,&t2);
    setCursor(0,30+8);
    t1/=10;
    t2/=10;
    if (tickMs<10000) {
      cnt+=10;
    }
    t1=cnt;
    sprintf(s,"Off %2d.%d/%2d.%d",t1/10,t1%10,t2/10,t2%10);
    LcdPrint(s);

    LcdRefresh();
  }

}


