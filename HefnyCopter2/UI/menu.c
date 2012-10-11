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
#include "../Include/Sensors.h"
#include "../Include/Beeper.h"
#include "../Include/IMU.h"


static uint8_t oldPage;
BOOL bValueChanged=false; 

typedef const prog_char screen_t[7][22];
typedef struct  
{
	uint8_t posX, posY;
	const prog_char *text;
} PROGMEM element_t;



//////////////////////////////////////////////////////////////////////////
#include "../Include/menu_text.h"
#include "../Include/menu_screen.h"

void _helper_DisplayBiStateText(uint8_t Row, uint8_t Col, PGM_P strTrue, PGM_P strFalse, bool Condition )
{

		LCD_SetPos(Row, Col);
		if (Condition==true) 
		{
			LCD_WriteString_P(strTrue);
		}
		else
		{
			LCD_WriteString_P(strFalse);
		}	
}


void _helper_SaveinEEPROM_ifChanged()
{
	if (bValueChanged==true)
		{
			Save_Config_to_EEPROM();
			bValueChanged = false;
			return ;
	}
}

void _helper_DisplayRXStatus(uint8_t Row)
{
	// Write RX Status
	lcdReverse((ActiveRXIndex!=0));
	_helper_DisplayBiStateText(Row,18,str1,strX,IS_TX1_GOOD);
	
	lcdReverse((ActiveRXIndex!=1));
	_helper_DisplayBiStateText(Row,30,str2,strX,IS_TX2_GOOD);
	
	lcdReverse(false);
	
}


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
void Menu_LoadPage(uint8_t pageIndex)
{
	memcpy_P(&currentPage, &pages[pageIndex], sizeof(currentPage));
	page = pageIndex;
}


void defaultHandler()
{
	
	if (editMode==true)
	{		// edit mode?
		editModeHandler();
	}
	else
	{
		if (IS_INIT | IS_KEYREFRESH) 
		{
			LCD_Clear();
			// 1- display screen content
			if (currentPage.screen)
				LCD_WriteString_P(currentPage.screen);
			// 2- Display control buttons... always be last to overwrite any graphics on it.
			writeSoftkeys(currentPage.softkeys);
			//LCD_SetPos(0, 0);
		}
		
		if (currentPage.handler)

			currentPage.handler();
		
	}			
}


void PageKey(uint8_t num)
{
	if (KEY2)	// PREV
		subpage = subpage == 0 ? num - 1 : subpage - 1;
	else if (KEY3) // NEXT
		subpage = (subpage + 1) % num;
}


void editModeHandler()
{
	if (KEY4)	// DONE;
	{
		editMode = false;
		_mykey = KEY_REFRESH;
		
		if (editValueType == TYPE_UINT8)
			*((uint8_t*)editValuePtr) = (uint8_t) editValue;
		else if (editValueType == TYPE_INT8)
			*((int8_t*)editValuePtr) = (int8_t) editValue;
		else if (editValueType == TYPE_INT16)
			*(int16_t*)editValuePtr = (int16_t) editValue;
		else if (editValueType == TYPE_UINT16)
			*(uint16_t*)editValuePtr =(uint16_t) editValue;
		
		//Save_Config_to_EEPROM();
		
		LCD_SelectFont(NULL);
		defaultHandler();
		return;
	}
	if (ANYKEY)
	{
		if (KEY2)	// DOWN?
		{
			
				if (_keyrepeat == KEYBOARD_REPEAT)
				{
					editValue-=10;
				}
				else
				{
					editValue--;	
				}
				
				if (editValue < editLoLimit)				
				{
					editValue = editLoLimit;
				}
		}
		else if (KEY3)	// UP?
		{
				if (_keyrepeat == KEYBOARD_REPEAT)
				{
					editValue+=10;
				}
				else
				{
					editValue++;	
				}
				
				if (editValue > editHiLimit)
				{
					editValue=editHiLimit;
				}					
		}
		else if (KEY1)	// CLR?
		{
			editMode = false;
			_mykey = KEY_REFRESH;
	
			LCD_SelectFont(NULL);
			defaultHandler();
			return;	
		}
			
		LCD_WriteValue(2, 34, editValue, 5, -1);
	}
}

static void startEditMode(void* valuePtr, int16_t loLimit, int16_t hiLimit, uint8_t valueType)
{
	editMode = true;
	_mykey = KEY_INIT;
	editValuePtr = valuePtr;
	editValueType = valueType;
	
	if (valueType == TYPE_UINT8)
		editValue = *(uint8_t*)valuePtr;
	else if (valueType == TYPE_INT8)
		editValue = *(int8_t*)valuePtr;
	else if (valueType == TYPE_INT16)
 		editValue = *(int16_t*)valuePtr;
	else if (valueType == TYPE_UINT16)
 		editValue = *(uint16_t*)valuePtr;
	
	editLoLimit = loLimit;
	editHiLimit = hiLimit;
	
	LCD_FillRectangle(30, 11, 98, 34, 0);
	LCD_Rectangle(30, 11, 98, 34, 1);
	LCD_Rectangle(31, 12, 97, 33, 1);
	writeSoftkeys(_skEDIT);
	LCD_SelectFont(NULL);//&font12x16);
	editModeHandler();
}


/*
//	Draw the menu
*/
uint8_t doMenu(menu_t *menu)
{
	UIEnableStickCommands=true;
	
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
		lcdReverse(menu->top + i == menu->marked);
		LCD_WritePadded_P(item, 21);
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
		Menu_LoadPage(mnuMain.marked + MENU_START_INDEX);
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
			{
			}				
				//showMotor(i, 0);
		}			
		else
		{
			lcdWriteChar(subpage + '0');
			//showMotor(subpage - 1, 1);
		}
		writeSoftkeys(NULL);
	}
}

void _hLoadModelLayout()
{
	//if (IS_INIT)
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
		//Menu_LoadPage(PAGE_SHOW_LAYOUT);
	//}
}

void _hHomePage()
{
	
	if (IS_INIT)
	{
		// Version
		LCD_SetPos(0,0);
		LCD_WriteString_P(strVersionInfo);
		// Sensors
		_helper_DisplayBiStateText(5, 60, strOK, strErr, (Config.IsCalibrated & CALIBRATED_SENSOR));
		
		//Stick Centering & Calibration
		_helper_DisplayBiStateText(5, 102, str1, strX, (Config.IsCalibrated & CALIBRATED_Stick_PRIMARY));
		
		_helper_DisplayBiStateText(5, 114, str2, strX, (Config.IsCalibrated & CALIBRATED_Stick_SECONDARY));
		
	}
	
	if (KEY4)	// MENU
	{
		Menu_LoadPage(PAGE_MENU);
		return;
	}
	
	// Write Voltage
	LCD_SetPos(2, 30);
	LCD_WriteString(Sensor_GetBatteryTest());
	
	_helper_DisplayRXStatus(5);
	
		
	///////// Flying Mode
	LCD_SetPos (6,0);
	LCD_SelectFont (&font12x16);
	if (Config.QuadFlyingMode==QuadFlyingMode_PLUS)
	{
		LCD_WriteString_P(PSTR ("+"));
	}
	else
	{
		LCD_WriteString_P(PSTR ("X"));
	}
	LCD_SelectFont (NULL);
	///////
	UIEnableStickCommands=true;
	
}

void _hHomeArmed()
{
	
	if (IS_INIT)	
	{
		LCD_SelectFont (&font12x16);
		LCD_SetPos(0,0);
		LCD_WriteString_P(strARMED);
		LCD_SelectFont (NULL);
	}
	
	
	if (KEY4)	// MENU
	{
		Menu_LoadPage(PAGE_MENU);
		return;
	}
	
	
	LCD_SetPos(3,18);
	itoa(MotorOut[0],sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(3,78);
	itoa(MotorOut[3],sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(4,18);
	itoa(MotorOut[1],sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(4,78);
	itoa(MotorOut[2],sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	
	LCD_SetPos(5,18);
	itoa(gyroRoll,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(5,78);
	itoa(gyroPitch,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(6,18);
	itoa(CompAngleX,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(6,78);
	itoa(CompAngleY,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
}


void _hHomeArmedESC (void)
{
	
	if (IS_INIT)	
	{
		LCD_SelectFont (&font12x16);
		LCD_SetPos(0,0);
		LCD_WriteString_P(strARMED);
		LCD_SelectFont (NULL);
	}
	
	if (KEY4)
	{
		//reset ESC Calibration mode.
		Config.IsESCCalibration=ESCCalibration_OFF;
		Save_Config_to_EEPROM();
	}
	
	LCD_SetPos(3,18);
	itoa(MotorOut[0],sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(3,78);
	itoa(MotorOut[3],sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(4,18);
	itoa(MotorOut[1],sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(4,78);
	itoa(MotorOut[2],sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
}

void _hSensorTest()
{
	
	//limits for sensor testing
	#define AccLowLimit			450
	#define AccHighLimit		850
	#define GyroLowLimit		500		
	#define GyroHighLimit		630

	
	LCD_SetPos(0, 48);
	LCD_WriteString(Sensors_Test(GYRO_X_PNUM,GyroLowLimit,GyroHighLimit));
	LCD_SetPos(1, 48);
	LCD_WriteString(Sensors_Test(GYRO_Y_PNUM,GyroLowLimit,GyroHighLimit));
	LCD_SetPos(2, 48);
	LCD_WriteString(Sensors_Test(GYRO_Z_PNUM,GyroLowLimit,GyroHighLimit));
	
	LCD_SetPos(3, 48);
	LCD_WriteString(Sensors_Test(ACC_X_PNUM,AccLowLimit,AccHighLimit));
	LCD_SetPos(4, 48);
	LCD_WriteString(Sensors_Test(ACC_Y_PNUM,AccLowLimit,AccHighLimit));
	LCD_SetPos(5, 48);
	LCD_WriteString(Sensors_Test(ACC_Z_PNUM,AccLowLimit,AccHighLimit));
	LCD_SetPos(6, 48);
	LCD_WriteString(Sensor_GetBatteryTest());
}

void _hReceiverTest()
{
	UIEnableStickCommands=false; // you cannot use sticks here. for arming or as buttons.
	
	RX_CopyLatestReceiverValues();
		
		
	for (uint8_t i = 0; i < RXChannels; i++)
	{
		LCD_SetPos(i+1, 24);
			
		//itoa(RX_Latest[ActiveRXIndex][i], sXDeg, 10);
		LCD_WriteValue(i+1,36,RX_Latest[0][i],5,(!IS_TX1_GOOD));
		LCD_WriteValue(i+1,84,RX_Latest[1][i],5,(!IS_TX2_GOOD));
		
	}			
	
	
	_helper_DisplayRXStatus(6);
	
}



BOOL bError; 
void _hStickCentering()
{
	UIEnableStickCommands=false; // you cannot use sticks here. for arming or as buttons.
	
	if (IS_INIT)
	{
		RX_StickCenterCalibrationInit(ActiveRXIndex);
		LCD_WriteString_Pex(0,0,strSPC1,18,false); // clear the header
	}
	
	if (KEY4)
	{
		if (!bError)
		{
			// Save config
			for (uint8_t i = 0; i < RXChannels; i++)
			{
				Config.RX_Mid[ActiveRXIndex][i] = (RX_MAX_raw[ActiveRXIndex][i]+RX_MIN_raw[ActiveRXIndex][i])/2;
				Config.RX_Min[ActiveRXIndex][i] = RX_MIN_raw[ActiveRXIndex][i];
				
			}		
			
			Config.IsCalibrated= (Config.IsCalibrated | (1 << ActiveRXIndex));	// either 0b00000001 or 0b00000010
			Save_Config_to_EEPROM();
		}
		else
		{
			Beeper_Beep(BEEP_ERROR,3);	
		}
		
	}
	bError = false;
	RX_StickCenterCalibration(ActiveRXIndex);
	for (uint8_t i = 0; i < RXChannels; i++)
	{
		LCD_SetPos(i+1, 30);
		utoa(RX_MAX_raw[ActiveRXIndex][i], sXDeg, 10);
		LCD_WriteString(sXDeg);
		LCD_WriteString_P(strSPC1);
		utoa(RX_MIN_raw[ActiveRXIndex][i], sXDeg, 10);
		LCD_WriteString(sXDeg);
		if ((ActiveRXIndex==0) && (i == RXChannel_AUX))
		{
			LCD_WriteString_P(PSTR("NA"));
		}
		else
		{
			if ((RX_MAX_raw[ActiveRXIndex][i]< RX_MIN_raw[ActiveRXIndex][i]) || (RX_MIN_raw[ActiveRXIndex][i]==0))  // RX_MIN_raw[i]=0 if the Remote is OFF when entering the test
			{
				LCD_WriteString_P(strErr);	
				bError = TRUE;
			}
			else
			{
				LCD_WriteString_P(strSPC4);	
			}
		}	
	}
	

	_helper_DisplayRXStatus(6);
}

void _hSensorCalibration()
{
	NOKEYRETURN;
	uint8_t i;
		
	if (KEY4)
	{
		
		for (i=0; i<10;++i)
		{
			Beeper_Beep(BEEP_SHORT,1);
			delay_ms (500); // delay to avoid click vibration.	
		
		}
	
		Sensors_Calibrate ();
		
		Config.IsCalibrated = (Config.IsCalibrated | CALIBRATED_SENSOR);
		for (i=0;i<6;++i)
		Config.Sensor_zero[i] = nResult[i];
		Save_Config_to_EEPROM();
		currentPage.softkeys = _skBACK;
		writeSoftkeys(currentPage.softkeys);
	}
	
	
	for (i=0; i<6;++i)
	{ // order is aligned with ACC_X_Index & GYRO_X_Index
		LCD_SetPos(i, 48);
		itoa(Config.Sensor_zero[i],Result,10);
		LCD_WriteString(&Result);
		LCD_WriteSpace(5);
	}	
	
}

void _hESCCalibration()
{
	NOKEYRETURN
	
	if (KEY4)
	{
		if ((Config.IsCalibrated & CALIBRATED_SENSOR) && ((Config.IsCalibrated & CALIBRATED_Stick_SECONDARY)==CALIBRATED_Stick_SECONDARY))
		{
			Config.IsESCCalibration=ESCCalibration_ON;
			Save_Config_to_EEPROM();
			while (true); // loop forever	
		}
				
	}
	
}
void _hModeSettings ()
{
	NOKEYRETURN;
	PageKey(1);
	
	if (KEY4)
	{
		bValueChanged = true;
		currentPage.softkeys = _skMENUSAVE;
		
		switch (subpage)
		{
			case 0: Config.RX_mode=~Config.RX_mode; break;
			
		}
		
	}
	
	if (KEY1)
	{
		_helper_SaveinEEPROM_ifChanged();
	}
	
	if (Config.RX_mode == RX_mode_BuddyMode)
	{
		LCD_WriteString_Pex(0,84,PSTR("Buddy"),5,(subpage==0));
	}
	else
	{
		LCD_WriteString_Pex(0,84,PSTR("UART "),5,0==subpage);
	}

}


void _hMiscSettings()
{
	NOKEYRETURN;
	PageKey(2);
	
		
	if (KEY4)
	{
		bValueChanged = true;
		currentPage.softkeys = _skMENUSAVE;
		
		switch (subpage)
		{
			case 0: startEditMode(&(Config.AutoDisarm),0,10,TYPE_UINT8); return ;
			case 1: startEditMode(&(Config.VoltageAlarm),0,100,TYPE_UINT8);  return ;
		}
		
	}
	
	if (KEY1)
	{
		_helper_SaveinEEPROM_ifChanged();
	}
	
	LCD_WriteValue(0,84,Config.AutoDisarm,3,0==subpage);
	LCD_WriteValue(1,84,Config.VoltageAlarm,3,1==subpage);
		
}

/*
* check : http://code.google.com/p/ardupirates/wiki/PID_Setting_SuperStable_Code
*/ 
void _hStabilization()
{
	
	NOKEYRETURN;
	PageKey(8);
	
	if (KEY4)
	{
		if (subpage!=0) bValueChanged = true;
		currentPage.softkeys = _skMENUSAVE;
		
		switch (subpage)
		{
			case 0: if (subindex==0) subindex=1; else subindex=0; break;
			case 1: startEditMode(&(Config.GyroParams[subindex]._P),-500,500,TYPE_INT16); return ;
			case 2: startEditMode(&(Config.GyroParams[subindex]._PLimit),0,500,TYPE_INT16); return ;
			case 3: startEditMode(&(Config.GyroParams[subindex]._I),-500,500,TYPE_INT16);  return ;
			case 4: startEditMode(&(Config.GyroParams[subindex]._ILimit),0,500,TYPE_INT16); return ;
			case 5: startEditMode(&(Config.GyroParams[subindex]._D),-500,500,TYPE_INT16);  return ; // negative D
			case 6: startEditMode(&(Config.GyroParams[subindex]._DLimit),0,500,TYPE_INT16); return ;
			case 7: startEditMode(&(Config.GyroParams[subindex].ComplementaryFilterAlpha),0,999,TYPE_INT16); return ;
		}
		
		
	}
	
	if (KEY1)
	{
		_helper_SaveinEEPROM_ifChanged();
	}
	
	lcdReverse(subpage == 0);
	if (subindex==0)
	{
		strcpy_P(sXDeg,PSTR("Pitch & Roll"));
	}
	else
	{
		strcpy_P(sXDeg,PSTR("YAW         "));
	}
	LCD_WriteStringex (0,0,sXDeg,0==subpage);
	LCD_WriteValue(1,30,Config.GyroParams[subindex]._P,3,1==subpage);
	LCD_WriteValue(1,78,Config.GyroParams[subindex]._PLimit,3,2==subpage);
	LCD_WriteValue(2,30,Config.GyroParams[subindex]._I,3,3==subpage);
	LCD_WriteValue(2,78,Config.GyroParams[subindex]._ILimit,3,4==subpage);
	LCD_WriteValue(3,30,Config.GyroParams[subindex]._D,3,5==subpage);
	LCD_WriteValue(3,78,Config.GyroParams[subindex]._DLimit,3,6==subpage);
	LCD_WriteValue(4,30,Config.GyroParams[subindex].ComplementaryFilterAlpha,3,7==subpage);
	
	//Pitch_Ratio = ((double)(Config.GyroParams[0].maxDest - Config.GyroParams[0].minDest)/(double)(Config.GyroParams[0].maxSource - Config.GyroParams[0].minSource));
	//Yaw_Ratio = ((double)(Config.GyroParams[1].maxDest - Config.GyroParams[1].minDest)/(double)(Config.GyroParams[1].maxSource - Config.GyroParams[1].minSource));
	
	
	
}



void _hSelfLeveling()
{

	
	NOKEYRETURN;
	PageKey(8);
	
	if (KEY4)
	{
		if (subpage!=0) bValueChanged = true;
		currentPage.softkeys = _skMENUSAVE;
		
		switch (subpage)
		{
			case 0: if (subindex==0) subindex=1; else subindex=0; break;
			case 1: startEditMode(&(Config.AccParams[subindex]._P),-500,500,TYPE_INT16); return ;
			case 2: startEditMode(&(Config.AccParams[subindex]._PLimit),0,500,TYPE_INT16); return ;
			case 3: startEditMode(&(Config.AccParams[subindex]._I),-500,500,TYPE_INT16);  return ;
			case 4: startEditMode(&(Config.AccParams[subindex]._ILimit),0,500,TYPE_INT16); return ;
			case 5: startEditMode(&(Config.AccParams[subindex]._D),-500,500,TYPE_INT16);  return ; // negative D
			case 6: startEditMode(&(Config.AccParams[subindex]._DLimit),0,500,TYPE_INT16); return ;
			case 7: startEditMode(&(Config.AccParams[subindex].ComplementaryFilterAlpha),0,999,TYPE_INT16); return ;
		}
		
		
	}
	
	if (KEY1)
	{
		_helper_SaveinEEPROM_ifChanged();
	}
	
	lcdReverse(subpage == 0);
	if (subindex==0)
	{
		strcpy_P(sXDeg,PSTR("ACC X & Y"));
	}
	else
	{
		strcpy_P(sXDeg,PSTR("ACC Z    "));
	}
	LCD_WriteStringex (0,0,sXDeg,0==subpage);
	LCD_WriteValue(1,30,Config.AccParams[subindex]._P,3,1==subpage);
	LCD_WriteValue(1,78,Config.AccParams[subindex]._PLimit,3,2==subpage);
	LCD_WriteValue(2,30,Config.AccParams[subindex]._I,3,3==subpage);
	LCD_WriteValue(2,78,Config.AccParams[subindex]._ILimit,3,4==subpage);
	LCD_WriteValue(3,30,Config.AccParams[subindex]._D,3,5==subpage);
	LCD_WriteValue(3,78,Config.AccParams[subindex]._DLimit,3,6==subpage);
	LCD_WriteValue(4,30,Config.AccParams[subindex].ComplementaryFilterAlpha,3,7==subpage);
	
}

int16_t AccTotal;
int16_t OldAcc;






void _hDebug()
{
	if (IS_INIT)
	{
  	    //timer = TCNT1;	
		  
		//dtime = 1;
		//dt=150;
		LCD_Clear();
		LCD_SetPos(1,6);
		LCD_WriteString_P(PSTR("GY")); // A Rate
		LCD_SetPos(2,6);
		LCD_WriteString_P(PSTR("GX")); // A Deg
		LCD_SetPos(3,6);
		LCD_WriteString_P(PSTR("AY")); 
		LCD_SetPos(4,6);
		LCD_WriteString_P(PSTR("AX"));
		LCD_SetPos(5,6);
		LCD_WriteString_P(PSTR("GyroY"));
		LCD_SetPos(6,6);
		LCD_WriteString_P(PSTR("GyroX"));
	
		OldAcc = ADCPort_Get(ACC_X_PNUM);
	}
	else
	{
	
		if (KEY2)
		{
			//dt+=1;
			//gyroXangle=0;
			CompAngleX=0;
			//gyroYangle=0;
			CompAngleY=0;
		}
		if (KEY3)
		{
			//dt-=1;
			//gyroXangle=0;
			CompAngleX=0;
			//gyroYangle=0;
			CompAngleY=0;
		}	
	//IMU_CalculateAngles();
	//
	
	LCD_WriteValue_double(1,48,CompGyroY,false);
	LCD_WriteValue_double(2,48,CompGyroX,false);
	
	
	/*
	itoa((Sensors_Latest[ACC_X_Index] * 2.08), sXDeg,10);
	LCD_SetPos(3,48);
	strcat_P(sXDeg,strSPC3);
	LCD_WriteString(sXDeg);
	
	itoa( CompAngleX, sXDeg,10);//itoa( term_I[0], sXDeg,10);
	LCD_SetPos(4,48);
	strcat_P(sXDeg,strSPC3);
	LCD_WriteString(sXDeg);
	
	
	itoa( Sensors_Latest[ACC_Y_Index] * 2.08, sXDeg,10);//itoa( term_I[0], sXDeg,10);
	LCD_SetPos(5,48);
	strcat_P(sXDeg,strSPC3);
	LCD_WriteString(sXDeg);
	
	
	
	itoa( gyroRoll, sXDeg,10);
	LCD_SetPos(6,48);
	strcat_P(sXDeg,strSPC3);
	LCD_WriteString(sXDeg);
	*/
	
/*		 
	int16_t t=ADCPort_Get(ACC_X_PNUM);
	AccTotal += t -OldAcc ; //Config.Sensor_zero[ACC_X_Index];
	OldAcc = t;
	utoa(AccTotal, sXDeg,10);

	IMU_CalculateAngles ();
	dtostrf( CompAngleY, 4, 2, sXDeg);
	LCD_SetPos(3,48);
	strcat_P(sXDeg,strSPC3);
	LCD_WriteString(sXDeg);
	
	gyroPitch= CompAngleY*10; //ScaleSensor ((CompAngleY*10),&(Config.AccParams));
	itoa( gyroPitch, sXDeg,10);
	LCD_SetPos(4,48);
	strcat_P(sXDeg,strSPC3);
	LCD_WriteString(sXDeg);
	
	gyroPitch= ScaleSensor (gyroPitch,&(Config.AccParams),Acc_Ratio);

	itoa( gyroPitch, sXDeg,10);
	LCD_SetPos(5,48);
	strcat_P(sXDeg,strSPC3);
	LCD_WriteString(sXDeg);
*/
	}
}
void _hFactoryReset()
{
	if (IS_INIT)
	{
		LCD_SetPos(3, 18);
		LCD_WriteString_P(strAreYouSure);
	}
	else if (KEY4)	// Yes
	{
		Save_Default_Config_to_EEPROM();
		//RST_CTRL
		while(1); // Loop for reset
	}
}

void Menu_MenuShow()
{
	
	
	_mykey = Keyboard_Read();
	_mykey = _mykey | _TXKeys;

	
	LCD_Disable();
	if (oldPage != page)
	{	// if this is a new page then KEY_INIT = true
		_mykey |= KEY_INIT;
		subpage = 0;
		subindex = 0;
		oldPage = page;
	}
	defaultHandler();
	LCD_Enable();

	
	
	if (KEY1 && !editMode)	// BACK
	{
		if (page > PAGE_MENU) // if any page then go to main menu
			Menu_LoadPage(PAGE_MENU);
		else if (page == PAGE_MENU)  // if menu page then goto HomePage
			{
				if (IsArmed == true)
				{
					Menu_LoadPage(PAGE_HOME_ARMED);
				}
				else
				{
					Menu_LoadPage(PAGE_HOME);
				}
			}							
	}
	
	
	if (KEYPRESS)
		Beeper_Beep(BEEP_KEY,1);
		
	_TXKeys = 0; // No Key Pressed
}

void Menu_MenuInit()
{
	oldPage=0xff;
	Menu_LoadPage(PAGE_HOME);
}

PGM_P tsmMain(uint8_t index)
{
	return (PGM_P)pgm_read_word(&lstMenu[index]);
}

PGM_P tsmLoadModelLayout(uint8_t index)
{
	//return (PGM_P)pgm_read_word(&mixerTable[index].Name);
}
