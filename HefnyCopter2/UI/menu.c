/*
 * menu.c
 *
 * Created: 30.07.2012 18:52:51
 *  Author: OliverS
 *
 * $Id: menu.c 27 2012-08-15 18:18:23Z olischulz24@googlemail.com $
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

#include "../Include/typedefs.h"
#include "../Include/GlobalValues.h"
#include "../Include/IO_config.h"
#include "../Include/Lcd.h"
#include "../Include/Keyboard.h"
#include "../Include/ADC_PORT.h"
#include "../Include/Receiver.h"
//#include "rx.h"
#include "../Include/Beeper.h"

#include <avr/pgmspace.h>
#include <stdlib.h>
#include <avr/wdt.h>

BOOL	ReStart=false;
uint8_t _mykey;
#define KEY_INIT	1
#define ISINIT		(_mykey & KEY_INIT)
#define KEY1		(_mykey & KEY_1)
#define KEY2		(_mykey & KEY_2)
#define KEY3		(_mykey & KEY_3)
#define KEY4		(_mykey & KEY_4)
#define ANYKEY		(_mykey)
#define KEYPRESS	(_mykey & (KEY_1|KEY_2|KEY_3|KEY_4))
#define NOKEYRETURN {if (!_mykey) return;}

typedef const prog_char screen_t[7][22];
typedef struct  
{
	uint8_t posX, posY;
	const prog_char *text;
} PROGMEM element_t;


// Pointer to menu item function handler.
typedef void (pageHandler)(void);
// Menu Item Page Layout Structure
typedef struct  
{
	const char *softkeys;   // softkeys function text  
	pageHandler *handler;	// function to execute
	const char *screen;		// screen contents
} page_t;

typedef struct  
{
	uint8_t len;
	PGM_P (*textSelector)(uint8_t);
	uint8_t top;
	uint8_t marked;
} menu_t;


//////////////////////////////////////////////////////////////////////////
#include "../Include/menu_text.h"
#include "../Include/menu_screen.h"






static void writeList(const element_t list[], uint8_t len)
{
	for (uint8_t i = 0; i < len; i++)
	{
		element_t e;
		memcpy_P(&e, &list[i], sizeof(e));
		LCD_SetPos(e.posX, e.posY);
		LCD_WriteString_P(e.text);
	}
}

static void writeSoftkeys(const char* sk)
{
	if (!sk)
		sk = currentPage.softkeys;
	if (sk)
	{
		LCD_SetPos(7, 0);
		LCD_WriteString_P(sk);
	}
}

/*
//	Put screen into CurrentPage ==> MemoryScreen
*/
void loadPage(uint8_t pageIndex)
{
	memcpy_P(&currentPage, &pages[pageIndex], sizeof(currentPage));
	page = pageIndex;
}

void defaultHandler()
{
	if (ISINIT) // if first 
	{	// 1- display screen content
		if (currentPage.screen)  
			LCD_WriteString_P(currentPage.screen);
		// 2- Display control buttons... always be last to overwrite any graphics on it.
		writeSoftkeys(currentPage.softkeys); 
	}
		
	if (currentPage.handler)
	{		// Now call screen handler.
			currentPage.handler();
	}			
}


/*
//	Draw the menu
*/
uint8_t doMenu(menu_t *menu)
{
	if (!_mykey) return 0;
	
	// key handling
	if (KEY2)		// UP
	{
		if (menu->marked > 0) 
			menu->marked--;
	}
	else if (KEY3)		// DOWN
	{
		if (menu->marked < menu->len - 1) 
			menu->marked++;
	}
	else if (KEY4)		// ENTER
		return 1;

	if (menu->marked < menu->top)
		menu->top = menu->marked;
	else if (menu->marked - menu->top >= 5)
		menu->top = menu->marked - 4;
	
	// text output
	LCD_SetPos(0, 58);
	if (menu->top > 0)
		lcdWriteGlyph_P(&glyArrowUp, 0);
		
	for (uint8_t i = 0; i < 5 && i < menu->len; i++)
	{
		LCD_SetPos(i + 1, 0);
		PGM_P item = menu->textSelector(menu->top + i);
		if (menu->top + i == menu->marked)
			lcdReverse(1);
		else
			lcdReverse(0);
		LCD_WriteString_P(item);
		for (uint8_t j = 0; j < 21 - strlen_P(item); j++)
			lcdWriteChar(32);
	}

	lcdReverse(0);
	LCD_SetPos(6, 58);
	if (menu->top < menu->len - 5)
		lcdWriteGlyph_P(&glyArrowDown, 0);
	
	return 0;
}

void _hMenu()
{
	if (doMenu(&mnuMain))
		loadPage(mnuMain.marked + 2);
}

static void showMotor(uint8_t motor, uint8_t withDir)
{
	//uint8_t x = 96;
	//uint8_t y = 32;
	//mixer_channel_t *channel = &Config.Mixer.Channel[motor];
	//
	//if (channel->flags & FLAG_ESC)
	//{
		//x += (channel->Aileron >> 2);
		//y -= (channel->Elevator >> 2);
	//
		//lcdLine(x, y, 96, 32);
		//lcdXY(x - 4, y - 4);
		//lcdWriteGlyph_P(&glyBall, ROP_PAINT);
		//lcdXY(x - 4, y - 7);
		//if (channel->Rudder >= 0)
			//lcdWriteGlyph_P(&glyDirCW, ROP_PAINT);
		//else
			//lcdWriteGlyph_P(&glyDirCCW, ROP_PAINT);
		//
		//lcdXY(x - 2, y - 2);
		//lcdReverse(1);
		//lcdSelectFont(&font4x6);
		//lcdWriteChar(motor + '1');
		//lcdSelectFont(NULL);
		//lcdReverse(0);
		//
		//if (withDir)
		//{
			//LCD_SetPos(2, 0);
			//LCD_WriteString_P(strDirSeen);
			//LCD_SetPos(5, 0);
			//if (channel->Rudder >= 0)
				//LCD_WriteString_P(strCW);
			//else
				//LCD_WriteString_P(strCCW);
		//}
	//}
	//else if (withDir)
	//{
		//LCD_SetPos(3, 64);
		//if (channel->flags == FLAG_NONE)
			//LCD_WriteString_P(strUnused);
		//else
			//LCD_WriteString_P(strServo);
	//}
}

void _hShowModelLayout()
{
	if (ANYKEY)
	{
		if (KEY2)	// NEXT
			subpage = (subpage + 1) % 9;
		
		LCD_Clear();
		LCD_WriteString_P(strOutput);
		lcdWriteChar(32);
		if (subpage == 0)
		{
			LCD_WriteString_P(strALL);
			for (uint8_t i = 0; i < 8; i++)
				showMotor(i, 0);
		}			
		else
		{
			lcdWriteChar(subpage + '0');
			showMotor(subpage - 1, 1);
		}
		writeSoftkeys(NULL);
	}
}

void _hLoadModelLayout()
{
	//if (ISINIT)
		//mnuMLayout.marked = Config.MixerIndex;
//
	//if (subpage == 0)
	//{
		//if (doMenu(&mnuMLayout))
		//{
			//LCD_Clear();
			//LCD_SetPos(3, 18);
			//LCD_WriteString_P(strAreYouSure);
			//writeSoftkeys(_skCANCELYES);
			//subpage = 1;
		//}
	//}		
	//else if (KEY4)		// YES
	//{
		//mixerLoadTable(mnuMLayout.marked);
		//configSave();
		//loadPage(PAGE_SHOW_LAYOUT);
	//}
}

void _hHomePage()
{
	char s[7];
	if (KEY4)	// MENU
	{
		loadPage(PAGE_MENU);
		return;
	}
	
	if (ISINIT)
	{
		//LCD_SetPos(0, 36);
		//lcdSelectFont(&font12x16);
		//LCD_WriteString_P(strSAFE);
		//lcdSelectFont(NULL);
		//LCD_SetPos(3, 0);
		//LCD_WriteString_P(strSelflevel);
		//LCD_WriteString_P(strSpIsSp);
		//LCD_SetPos(5, 0);
		//LCD_WriteString_P(strBattery);
	}
	
	LCD_SetPos(3, 84);
	if (Config.IsCalibrated==0)
	{
		LCD_SetPos(5, 20);
		LCD_WriteString_P(PSTR("Not Calibrated !"));
	}
	//LCD_WriteString_P(strON);
	//else		
		//LCD_WriteString_P(strOFF);
	//
	//LCD_SetPos(4, 0);
	//if (State.Error)
	//{
		//LCD_WriteString_P(strError);
		//lcdWriteChar(32);
		//if (State.Error & ERR_NOT_CALIBRATED)
			//LCD_WriteString_P(strSensorNotCal);
		//else 
		//{
			//LCD_WriteString_P(PSTR("no "));
			//if (State.Error & ERR_NO_ROLL)
				//LCD_WriteString_P(strRoll);
			//else if (State.Error & ERR_NO_PITCH)
				//LCD_WriteString_P(strPitch);
			//else if (State.Error & ERR_NO_YAW)
				//LCD_WriteString_P(strYaw);
			//else 
				//LCD_WriteString_P(strThro);
			//LCD_WriteString_P(PSTR(" input"));
		//}
	//}	
	//else
		//LCD_WriteString_P(PSTR("Ready for departure!"));
	//
	//LCD_SetPos(5, 9*6);
	//utoa(BATT / 10, s, 10);
	//LCD_WriteString(s);
	//lcdWriteChar('.');
	//utoa(BATT % 10, s, 10);
	//LCD_WriteString(s);
	//LCD_WriteString_P(PSTR(" V "));
}

void _hSensorTest()
{
	
	LCD_SetPos(0, 48);
	LCD_WriteString(Sensors_Gyro_Test(GYRO_X_PNUM));
	LCD_SetPos(1, 48);
	LCD_WriteString(Sensors_Gyro_Test(GYRO_Y_PNUM));
	LCD_SetPos(2, 48);
	LCD_WriteString(Sensors_Gyro_Test(GYRO_Z_PNUM));
	
	LCD_SetPos(3, 48);
	LCD_WriteString(Sensors_Acc_Test(ACC_X_PNUM));
	LCD_SetPos(4, 48);
	LCD_WriteString(Sensors_Acc_Test(ACC_Y_PNUM));
	LCD_SetPos(5, 48);
	LCD_WriteString(Sensors_Acc_Test(ACC_Z_PNUM));
	
	
	/*
	utoa(ADCPort_Get(V_BAT_PNUM), s, 10);
	LCD_SetPos(6, 48);
	LCD_WriteString(s);
	lcdWriteChar(32);
	*/
}

void _hReceiverTest()
{
	char _t[10];

	for (uint8_t i = 0; i < RXChannels; i++)
	{
		LCD_SetPos(i, 40);
		//if (RX_good & _BV(i))
		//{
			utoa(RX_GetReceiverValues (i), _t, 10);
			LCD_WriteString(_t);
		//}
		//else
		//	LCD_WriteString_P(strNoSignal);
	}			
}


void _hStickCentering()
{
	char _t[10];
	BOOL bError = false; 
	if (ISINIT)
	{
		RX_StickCenterCalibrationInit();
	}
	
	if (KEY4 & (!bError))
	{
		// Save config
		for (uint8_t i = 0; i < RXChannels; i++)
		{
				Config.RX_Max[i] = RX_MAX_raw[i];
				Config.RX_Min[i] = RX_MIN_raw[i];
		}		
		//Save_Config_to_EEPROM();
		Beeper_Beep(700,1);
	}
	
	RX_StickCenterCalibration();
	for (uint8_t i = 0; i < RXChannels; i++)
	{
		LCD_SetPos(i, 35);
		utoa(RX_MAX_raw[i], _t, 10);
		LCD_WriteString(_t);
		LCD_WriteString(" ");
		utoa(RX_MIN_raw[i], _t, 10);
		LCD_WriteString(_t);	
		if ((RX_MAX_raw[i]< RX_MIN_raw[i]) || (RX_MIN_raw[i]==0))  // RX_MIN_raw[i]=0 if the Remote is OFF when entering the test
		{
			LCD_WriteString_P(PSTR(" err"));	
			bError = TRUE;
		}
		else
		{
			LCD_WriteString_P(PSTR("    "));	
		}
		
	}		
	//else if (KEY4)
		//loadPage(PAGE_MENU);
}

void _hSensorCalibration()
{

	if (ISINIT)
	{
		ReStart = false;
		uint8_t i;
		for (i=0; i<10;++i)
		{
			Beeper_Beep(70,1);
			_delay_ms (1500); // delay to avoid click vibration.	
		
		}
	
		Sensors_Calibrate ();
		
		for (i=0; i<6;++i)
		{
			LCD_SetPos(i, 48);	
			utoa(nResult[i],Result,10);
			LCD_WriteString(&Result);
		}	
		

		Config.IsCalibrated = true;
		for (i=0;i<6;++i)
		Config.Sensor_zero[i] = nResult[i];
		Save_Config_to_EEPROM();
	
		Beeper_Beep(700,1);
	}
	
	if (KEY4)
	{
		_mykey |= KEY_INIT;
	}
	
}

void _hESCCalibration()
{
	if (ANYKEY)
	{
		if (subpage >= length(scrESCCal))
			loadPage(PAGE_MENU);
		else
		{
			LCD_Clear();
			PGM_P s = (PGM_P)pgm_read_word(&scrESCCal[subpage]);
			LCD_WriteString_P(s);
			writeSoftkeys(NULL);
			subpage++;
		}		
	}
}


void _hDebug()
{
	//LCD_SetPos(0, 0);
	//LCD_WriteString_P(PSTR("MixerIndex: "));
	//char s[7];
	//utoa(Config.MixerIndex, s, 10);
	//LCD_WriteString(s);
}

void _hFactoryReset()
{
	//if (ISINIT)
	//{
		//LCD_SetPos(3, 18);
		//LCD_WriteString_P(strAreYouSure);
	//}
	//else if (KEY4)	// Yes
	//{
		//configReset();
		//configSave();
		//cli();
		//wdt_enable(WDTO_15MS);
		//for(;;);
	//}
}

void menuShow()
{
	static uint8_t oldPage = 0xFF;
	
	_mykey = Keyboard_Read();
		
	if (KEY1)	// BACK
	{
		if (page > PAGE_MENU) // if any page then go to main menu
			loadPage(PAGE_MENU);
		else if (page == PAGE_MENU)  // if menu page then goto HomePage
			loadPage(PAGE_HOME);
	}
	
	LCD_Disable();
	if (oldPage != page)
	{	// if this is a new page then KEY_INIT = true
		_mykey |= KEY_INIT;
		subpage = 0;
		LCD_Clear();
		oldPage = page;
	}
	defaultHandler();
	LCD_Enable();

	if (KEYPRESS)
		Beeper_Beep(70,1);
}

void menuInit()
{
	_mykey |= KEY_INIT;
	loadPage(PAGE_HOME);
}

PGM_P tsmMain(uint8_t index)
{
	return (PGM_P)pgm_read_word(&lstMenu[index]);
}

PGM_P tsmLoadModelLayout(uint8_t index)
{
	//return (PGM_P)pgm_read_word(&mixerTable[index].Name);
}
