/*
 * lcd.h
 *
 * Created: 27.07.2012 13:13:11
 *  Author: OliverS
 *
 * $Id: lcd.h 25 2012-08-15 16:18:33Z olischulz24@googlemail.com $
 */ 


#ifndef LCD_H_
#define LCD_H_

#include "fonts.h"
#include "glyphs.h"
#include <avr/pgmspace.h>

void LCD_Init();
void LCD_Clear();
void LCD_SetPos(uint8_t line, uint8_t column);
void lcdWriteChar(char c);
void LCD_WriteString(char *s);
void LCD_WriteStringex(uint8_t x, uint8_t y, char *str, BOOL LCDReverse);
void LCD_WriteString_P(PGM_P s);
void LCD_WriteString_Pex(uint8_t x, uint8_t y, PGM_P str, uint8_t len, BOOL LCDReverse);
void LCD_WriteSpace(uint8_t len);
void LCD_WritePadded(char *s, uint8_t len);
void LCD_WritePadded_P(const char *s, uint8_t len);
void LCD_WriteValue(uint8_t x, uint8_t y, int16_t value, uint8_t len, BOOL LCDReverse);
void LCD_WriteValue_double(uint8_t x, uint8_t y, double value, BOOL LCDReverse);
void lcdReverse(uint8_t reversed);
void lcdSetContrast(uint8_t contrast);
void lcdSetPixel(uint8_t x, uint8_t y, uint8_t on);
void LCD_Line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void LCD_Rectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void LCD_FillRectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void LCD_Enable();
void LCD_Disable();
void LCD_SelectFont(const fontdescriptor_t *font);
void lcdXY(uint8_t x, uint8_t y);
void lcdWriteGlyph_P(const glyph_t *glyph, uint8_t mode);

#define ROP_COPY	0
#define ROP_PAINT	1
#define ROP_INVERT	2

#endif /* LCD_H_ */