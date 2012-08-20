/*
 * LCD_ST7565.h
 *
 * Created: 8/13/2012 8:39:05 PM
 *  Author: hefny
 */ 


#ifndef LCD_ST7565_H_
#define LCD_ST7565_H_

#define BLACK 1
#define WHITE 0

#define LCDWIDTH 128
#define LCDHEIGHT 64

#define CMD_DISPLAY_OFF   0xAE
#define CMD_DISPLAY_ON    0xAF

#define CMD_SET_DISP_START_LINE  0x40
#define CMD_SET_PAGE  0xB0

#define CMD_SET_COLUMN_UPPER  0x10
#define CMD_SET_COLUMN_LOWER  0x00

#define CMD_SET_ADC_NORMAL  0xA0			// RTL or LTR
#define CMD_SET_ADC_REVERSE 0xA1

#define CMD_SET_DISP_NORMAL 0xA6
#define CMD_SET_DISP_REVERSE 0xA7			// negative display

#define CMD_SET_ALLPTS_NORMAL 0xA4
#define CMD_SET_ALLPTS_ON  0xA5
#define CMD_SET_BIAS_9 0xA2 
#define CMD_SET_BIAS_7 0xA3

#define CMD_RMW  0xE0
#define CMD_RMW_CLEAR 0xEE
#define CMD_INTERNAL_RESET  0xE2
#define CMD_SET_COM_NORMAL  0xC0	
#define CMD_SET_COM_REVERSE  0xC8	// Reverse makes (0,0) from down-left and invert the font [Verticl FLIP]
#define CMD_SET_POWER_CONTROL  0x28
#define CMD_SET_RESISTOR_RATIO  0x20
#define CMD_SET_VOLUME_FIRST  0x81
#define  CMD_SET_VOLUME_SECOND  0
#define CMD_SET_STATIC_OFF  0xAC
#define  CMD_SET_STATIC_ON  0xAD
#define CMD_SET_STATIC_REG  0x0
#define CMD_SET_BOOSTER_FIRST  0xF8
#define CMD_SET_BOOSTER_234  0
#define  CMD_SET_BOOSTER_5  1
#define  CMD_SET_BOOSTER_6  3
#define CMD_NOP  0xE3
#define CMD_TEST  0xF0
  
#include "../Include/glyphs.h"
#include "../Include/Fonts.h"
#include <avr/pgmspace.h>
  

//class LCD_ST7565 
//{
	//public:

void lcdInit();
void lcdClear();
void lcdSetPos(uint8_t line, uint8_t column);
void lcdWriteChar(char c);
void lcdWriteString(char *s);
void lcdWriteString_P(PGM_P s);
void lcdReverse(uint8_t reversed);
void lcdSetContrast(uint8_t contrast);
void lcdSetPixel(uint8_t x, uint8_t y, uint8_t on);
void lcdLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void lcdEnable();
void lcdDisable();
void lcdSelectFont(const fontdescriptor_t *font);
void lcdXY(uint8_t x, uint8_t y);
void lcdWriteGlyph_P(const glyph_t *glyph, uint8_t mode);

#define ROP_COPY	0
#define ROP_PAINT	1
#define ROP_INVERT	2

//};


#endif /* LCD_ST7565_H_ */