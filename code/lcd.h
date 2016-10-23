#ifndef _LCD_H_
#define _LCD_H_

#include <avr/pgmspace.h>

#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

// Pins for the LCD control lines
#define LCD_DC_PIN                 2    //PD2
#define LCD_CE_PIN                 3    //PD3
#define LCD_RST_PIN                4    //PD4

// Some macros for manipulation of the LCD control lines
#define SET_DC_PIN                 PORTD |= _BV(LCD_DC_PIN)  
#define CLEAR_DC_PIN               PORTD &= ~_BV(LCD_DC_PIN)
#define SET_SCE_PIN                PORTD |= _BV(LCD_CE_PIN)
#define CLEAR_SCE_PIN              PORTD &= ~_BV(LCD_CE_PIN)
#define SET_RST_PIN                PORTD |= _BV(LCD_RST_PIN)
#define CLEAR_RST_PIN              PORTD &= ~_BV(LCD_RST_PIN)

#define LCDWIDTH 84
#define LCDHEIGHT 48

#define BLACK 1
#define WHITE 0

#define PCD8544_POWERDOWN 0x04
#define PCD8544_ENTRYMODE 0x02
#define PCD8544_EXTENDEDINSTRUCTION 0x01

#define PCD8544_DISPLAYBLANK 0x0
#define PCD8544_DISPLAYNORMAL 0x4
#define PCD8544_DISPLAYALLON 0x1
#define PCD8544_DISPLAYINVERTED 0x5

// H = 0
#define PCD8544_FUNCTIONSET 0x20
#define PCD8544_DISPLAYCONTROL 0x08
#define PCD8544_SETYADDR 0x40
#define PCD8544_SETXADDR 0x80

// H = 1
#define PCD8544_SETTEMP 0x04
#define PCD8544_SETBIAS 0x10
#define PCD8544_SETVOP 0x80

typedef struct { // Data stored PER GLYPH
	uint16_t bitmapOffset;     // Pointer into GFXfont->bitmap
	uint8_t  width, height;    // Bitmap dimensions in pixels
	uint8_t  xAdvance;         // Distance to advance cursor (x axis)
	int8_t   xOffset, yOffset; // Dist from cursor pos to UL corner
} GFXglyph;

typedef struct { // Data stored for FONT AS A WHOLE:
	uint8_t  *bitmap;      // Glyph bitmaps, concatenated
	GFXglyph *glyph;       // Glyph array
	uint8_t   first, last; // ASCII extents
	uint8_t   yAdvance;    // Newline distance (y axis)
} GFXfont;


void LcdInit(uint8_t contrast);
void LcdCommand(uint8_t v);
void LcdData(uint8_t v);
void LcdContrast(uint8_t contrast);
void LcdClear(void);
void LcdRefresh();
void LcdSetPixel(int16_t x, int16_t y, uint8_t color);
uint8_t LcdGetPixel(int8_t x, int8_t y);
void LcdDrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void LcdDrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void LcdDrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void LcdFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void LcdSetFont(const GFXfont *f);
void setCursor(int16_t x, int16_t y);
void setTextSize(uint8_t s);
void setTextColor(uint16_t c, uint16_t b);
void LcdWriteChar(uint8_t c);
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
void LcdPrint(const char *s);

#endif  //_LCD_H_
