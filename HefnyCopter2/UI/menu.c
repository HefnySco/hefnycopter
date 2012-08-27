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
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <avr/wdt.h>

#include "../Include/typedefs.h"
#include "../Include/GlobalValues.h"
#include "../Include/IO_config.h"
#include "../Include/Lcd.h"
#include "../Include/Keyboard.h"
#include "../Include/ADC_PORT.h"
#include "../Include/Receiver.h"
//#include "rx.h"
#include "../Include/Beeper.h"


BOOL	ReStart=false;
static uint8_t oldPage;

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
		//LCD_SelectFont(&font4x6);
		//lcdWriteChar(motor + '1');
		//LCD_SelectFont(NULL);
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
		
		LCD_SetPos(5, 40);
		if (!(Config.IsCalibrated & CALIBRATED_SENSOR)) 
		{
			LCD_WriteString_P(PSTR("SN-Err"));
		}
		else
		{
			LCD_WriteString_P(PSTR("SN-OK "));
		}
		if (!(Config.IsCalibrated & CALIBRATED_Stick)) 
		{
			
			LCD_WriteString_P(PSTR(" ST-Err"));
		}
		else
		{
			
			LCD_WriteString_P(PSTR(" ST-OK "));
		}
		
	}

	LCD_SetPos(5, 0);
	if (RX_Good==false)
	{
		LCD_WriteString_P(PSTR("RX-Err"));
	}
	else
	{
		LCD_WriteString_P(PSTR("RX-OK "));
	}
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
}

void _hReceiverTest()
{
	
	RX_CopyLatestReceiverValues();
		
		
	for (uint8_t i = 0; i < RXChannels; i++)
	{
		LCD_SetPos(i, 40);
			
		itoa(RX_Latest[i], sXDeg, 10);
		LCD_WriteString(sXDeg);
		LCD_WriteString("    ");
	}			
	
	LCD_SetPos(6, 40);
		
	if (RX_Good==false)
	{
		LCD_WriteString_P(strNoSignal);
	}	
	else
	{
		LCD_WriteString_P("         ");
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
	
	if (KEY4)
	{
		if (!bError)
		{
			// Save config
			for (uint8_t i = 0; i < RXChannels; i++)
			{
					Config.RX_Mid[i] = (RX_MAX_raw[i]+RX_MIN_raw[i])/2;
					Config.RX_Min[i] = RX_MIN_raw[i];
			}		
			
			Config.IsCalibrated= (Config.IsCalibrated | CALIBRATED_Stick);
			Save_Config_to_EEPROM();
			Beeper_Beep(700,1);	
		}
		else
		{
			Beeper_Beep(200,3);	
		}
		
	}
	
	RX_StickCenterCalibration();
	for (uint8_t i = 0; i < RXChannels; i++)
	{
		LCD_SetPos(i, 30);
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
			delay_ms (1500); // delay to avoid click vibration.	
		
		}
	
		Sensors_Calibrate ();
		
		for (i=0; i<6;++i)
		{ // order is aligned with ACC_X_Index & GYRO_X_Index
			LCD_SetPos(i, 48);	
			utoa(nResult[i],Result,10);
			LCD_WriteString(&Result);
		}	
		

		Config.IsCalibrated = (Config.IsCalibrated | CALIBRATED_SENSOR);
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



#define aX A3
#define aY A4
#define aZ A5

#define RAD_TO_DEG 57.295779513082320876798154814105

//gyros
float gyroXadc;
float gyroXrate;
float gyroXangle;

float gyroYadc;
float gyroYrate;
float gyroYangle;

float gyroZadc;
float gyroZrate;
float gyroZangle;


//accelerometers

int accZeroX;//x-axis
float accXadc;
float accXval;
float accXangle;

int accZeroY;//y-axis
float accYadc;
float accYval;
float accYangle;

int accZeroZ;//z-axis
float accZadc;
float accZval;
float accZangle;

//Results
float xAngle;
float yAngle;
float compAngleX;
float compAngleY;

float R;//force vector
//Used for timing
uint16_t timer=0;
double dtime=0;  
uint16_t dt;

void _hDebug()
{
	if (ISINIT)
	{
  	    //timer = TCNT1;	
		  
		dtime = 1;
		dt=150;
		LCD_Clear();
		LCD_SetPos(1,6);
		LCD_WriteString_P(PSTR("AR")); // A Rate
		LCD_SetPos(2,6);
		LCD_WriteString_P(PSTR("AD")); // A Deg
		LCD_SetPos(3,6);
		LCD_WriteString_P(PSTR("GV")); 
		LCD_SetPos(4,6);
		LCD_WriteString_P(PSTR("GD"));
		LCD_SetPos(5,6);
		LCD_WriteString_P(PSTR("D"));
		LCD_SetPos(6,6);
		LCD_WriteString_P(PSTR("Time"));
		gyroXangle=0;
	}
	else
	{
	
		if (KEY2)
		{
			dt+=1;
			gyroXangle=0;
			compAngleX=0;
			gyroYangle=0;
			compAngleY=0;
		}
		if (KEY3)
		{
			dt-=1;
			gyroXangle=0;
			compAngleX=0;
			gyroYangle=0;
			compAngleY=0;
		}	
	
  //timer = TCNT1;	
  gyroXadc = ADCPort_Get(GYRO_X_PNUM)/10;
  gyroXrate = (gyroXadc-Config.Sensor_zero[GYRO_X_Index]) ;//* 1.0323;//(gyroXadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  if ((gyroXrate<=1) && (gyroXrate>=-1))
  {
	  gyroXrate=0;
  }
  gyroXangle=gyroXangle+gyroXrate * dtime;//Without any filter
  
  gyroYadc = ADCPort_Get(GYRO_Y_PNUM);
  gyroYrate = (gyroYadc-Config.Sensor_zero[GYRO_Y_Index]) ;//* 1.0323;//(gyroYadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  if ((gyroYrate<=1) && (gyroYrate>=-1))
  {
	  gyroYrate=0;
  }
 gyroYangle=gyroYangle+gyroYrate;//Without any filter
  
  gyroZadc = ADCPort_Get(GYRO_Z_PNUM);
  gyroZrate = (gyroZadc-Config.Sensor_zero[GYRO_Z_Index]);///1.0323;//(gyroZadc-gryoZeroX)/Sensitivity - in quids              Sensitivity = 0.00333/3.3*1023=1.0323
  //gyroZangle=gyroZangle+gyroZrate*dtime/1000;//Without any filter
  
  accXadc = ADCPort_Get(ACC_X_PNUM);
  accXval = (accXadc-Config.Sensor_zero[ACC_X_Index]);//(accXadc-accZeroX)/Sensitivity - in quids              Sensitivity = 0.33/3.3*1023=102,3
  
  accYadc = ADCPort_Get(ACC_Y_PNUM);
  accYval = (accYadc-Config.Sensor_zero[ACC_Y_Index]);///102;//(accXadc-accZeroX)/Sensitivity - in quids              Sensitivity = 0.33/3.3*1023=102,3
  
  accZadc = ADCPort_Get(ACC_Z_PNUM);
  accZval = (accZadc-Config.Sensor_zero[ACC_Z_Index]);///102;//(accXadc-accZeroX)/Sensitivity - in quids              Sensitivity = 0.33/3.3*1023=102,3
  //accZval++;//1g in horizontal position
  
  //R = sqrt(pow(accXval,2)+pow(accYval,2)+pow(accZval,2));//the force vector
  accXangle = accXval * 0.72; //acos(accXval/R)*RAD_TO_DEG-90;
  accYangle = accYval * 0.72; //acos(accYval/R)*RAD_TO_DEG-90;
  //accZangle = acos(accZval/R)*RAD_TO_DEG;
 
  
  compAngleX = (0.98*(compAngleX+(gyroXrate)*dt/1000))+(0.02*(accYangle));
  compAngleY = (0.6*(compAngleY-(gyroYrate)*dt/1000))+(0.4*(accXangle));
 for (int c=0;c<dt;++c)
 {
   delay_ms(1);
  }  
 // utoa(compAngleX,sXDeg,10);
 
 
  dtostrf( accXval, 3, 4, sXDeg);
  LCD_SetPos(1,48);
  LCD_WriteString(sXDeg);
  LCD_WriteString_P(PSTR("    "));

  dtostrf( accXangle, 3, 4, sXDeg);
  LCD_SetPos(2,48);
  LCD_WriteString(sXDeg);
  LCD_WriteString_P(PSTR("    "));

  
  //sprintf(sXDeg,"%i",gyroXrate);
  dtostrf( gyroYrate, 3, 4, sXDeg);
  LCD_SetPos(3,48);
  LCD_WriteString(sXDeg);
  LCD_WriteString_P(PSTR("    "));

   //sprintf(sXDeg,"%i",gyroXrate);
  dtostrf( gyroYangle, 3, 4, sXDeg);
  LCD_SetPos(4,48);
  LCD_WriteString(sXDeg);
  LCD_WriteString_P(PSTR("    "));

 //sprintf(sXDeg,"%i",gyroXrate);
  dtostrf( compAngleY, 3, 4, sXDeg);
  LCD_SetPos(5,48);
  LCD_WriteString(sXDeg);
  LCD_WriteString_P(PSTR("    "));


  dtostrf( dt, 3, 4, sXDeg);
  LCD_SetPos(6,48);
  LCD_WriteString(sXDeg);
  LCD_WriteString_P(PSTR("    "));


 	}
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

void Menu_MenuShow()
{
	
	
	_mykey = Keyboard_Read();
	_mykey = _mykey | _TXKeys;
	// Throttle is not low to avoid conflict with other Arming/Disarming TX commands
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
		
	_TXKeys = 0; // No Key Pressed
}

void Menu_MenuInit()
{
	oldPage=0xff;
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
