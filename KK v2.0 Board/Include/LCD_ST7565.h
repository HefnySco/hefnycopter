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
  

class LCD_ST7565 
{
	public:
	void ClearScreen ();
	void Init();
	
    
	void st7565_Command(uint8_t c);
	void st7565_Data(uint8_t c);
	void Set_Brightness(uint8_t val);
	void ClearDisplay(void);
	void Clear();
	void Display();

	void SetPixel(uint8_t x, uint8_t y, uint8_t color);
	uint8_t GetPixel(uint8_t x, uint8_t y);
	void FillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
	void DrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
	
  void DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
  void FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
  void DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
  void DrawChar(uint8_t x, uint8_t line, char c);
  void DrawString(uint8_t x, uint8_t line, char *c);
  void DrawString_P(uint8_t x, uint8_t line, const char *c);
  void DrawBitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t color);

 private:
  int8_t sid, sclk, a0, rst, cs;
  void my_setpixel(uint8_t x, uint8_t y, uint8_t color); 
  void ShiftOut(uint8_t c);
  void LCDdWriteSprite_P(PGM_P sprite, uint8_t sizeX, uint8_t sizeY, uint8_t mode);
};


#endif /* LCD_ST7565_H_ */