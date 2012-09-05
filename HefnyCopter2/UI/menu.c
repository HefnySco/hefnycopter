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
// 		else if (editValueType == TYPE_INT16)
// 			*(int16_t*)editValuePtr = editValue;
		
		//Save_Config_to_EEPROM();
		
		LCD_SelectFont(NULL);
		defaultHandler();
		return;
	}
	if (ANYKEY)
	{
		if (KEY2)	// DOWN?
		{
			if (editValue > editLoLimit)
				editValue--;
		}
		else if (KEY3)	// UP?
		{
			if (editValue < editHiLimit)
				editValue++;
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
// 	else if (valueType == TYPE_INT16)
// 		editValue = *(int16_t*)valuePtr;
	
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
	char s[7];
	if (KEY4)	// MENU
	{
		Menu_LoadPage(PAGE_MENU);
		return;
	}
	
	if (IS_INIT)
	{
		
		LCD_SetPos(5, 60);
		if (!(Config.IsCalibrated & CALIBRATED_SENSOR)) 
		{
			LCD_WriteString_P(strErr);
		}
		else
		{
			LCD_WriteString_P(strOK);
		}
		LCD_SetPos(5, 102);
		if (!(Config.IsCalibrated & CALIBRATED_Stick)) 
		{
			
			LCD_WriteString_P(strErr);
		}
		else
		{
			
			LCD_WriteString_P(strOK);
		}
		
	}
	// Write Voltage
	LCD_SetPos(2, 30);
	LCD_WriteString(Sensor_GetBatteryTest());
	
	// Write RX Status
	LCD_SetPos(5, 18);
	if (RX_Good != TX_GOOD)
	{
		LCD_WriteString_P(strErr);
	}
	else
	{
		LCD_WriteString_P(strOK);
	}
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
	
	LCD_SetPos(3,18);
	itoa(MotorOut1,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(3,78);
	itoa(MotorOut4,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(4,18);
	itoa(MotorOut2,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(4,78);
	itoa(MotorOut3,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	
	LCD_SetPos(5,18);
	itoa(gyroPitch,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(5,78);
	itoa(gyroRoll,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(6,18);
	itoa(accPitch,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
	
	LCD_SetPos(6,78);
	itoa(accRoll,sXDeg,10);
	LCD_WritePadded(sXDeg,5);
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
	LCD_SetPos(6, 48);
	LCD_WriteString(Sensor_GetBatteryTest());
}

void _hReceiverTest()
{
	
	RX_CopyLatestReceiverValues();
		
		
	for (uint8_t i = 0; i < RXChannels; i++)
	{
		LCD_SetPos(i, 40);
			
		itoa(RX_Latest[i], sXDeg, 10);
		LCD_WriteString(sXDeg);
		LCD_WriteString_P(strSPC4);
	}			
	
	
	LCD_SetPos(6, 40);	
	if (TX_FOUND_ERR!=0)
	{
		LCD_WriteString_P(strNoSignalFound);
	}
	else if (TX_CONNECTED_ERR!=0)	
	{
		LCD_WriteString_P(strNoSignalDis);
	}
	else
	{
		LCD_WriteString_P(PSTR("signal-ok    "));	
	}
	
}



BOOL bError; 
void _hStickCentering()
{
	
	if (IS_INIT)
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
		}
		else
		{
			Beeper_Beep(200,3);	
		}
		
	}
	bError = false;
	RX_StickCenterCalibration();
	for (uint8_t i = 0; i < RXChannels; i++)
	{
		LCD_SetPos(i, 30);
		utoa(RX_MAX_raw[i], sXDeg, 10);
		LCD_WriteString(sXDeg);
		LCD_WriteString_P(PSTR(" "));
		utoa(RX_MIN_raw[i], sXDeg, 10);
		LCD_WriteString(sXDeg);	
		if ((RX_MAX_raw[i]< RX_MIN_raw[i]) || (RX_MIN_raw[i]==0))  // RX_MIN_raw[i]=0 if the Remote is OFF when entering the test
		{
			LCD_WriteString(strErr);	
			bError = TRUE;
		}
		else
		{
			LCD_WriteString_P(strSPC4);	
		}
		
	}		
}

void _hSensorCalibration()
{

	NOKEYRETURN
	
	if (KEY4)
	{
		
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
		
		_mykey |= KEY_INIT;
	}
	
}

void _hESCCalibration()
{
	if (ANYKEY)
	{
		if (subpage >= length(scrESCCal))
			Menu_LoadPage(PAGE_MENU);
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



void _hStabilization()
{
	
	NOKEYRETURN;
	PageKey(4);
	
	if (KEY4)
	{
		bValueChanged = true;
		currentPage.softkeys = _skMENUSAVE;
		
		switch (subpage)
		{
			case 0: startEditMode(&(Config.GyroParams[0].Gain),0,200,TYPE_UINT8);  return ;
			case 1: startEditMode(&(Config.GyroParams[0].Limit),0,200,TYPE_UINT8); return ;
			case 2: startEditMode(&(Config.GyroParams[1].Gain),0,200,TYPE_UINT8);  return ;
			case 3: startEditMode(&(Config.GyroParams[1].Limit),0,200,TYPE_UINT8); return ;
		}
		
	}
	
	if (KEY1)
	{
		if (bValueChanged==true)
		{
			Save_Config_to_EEPROM();
			bValueChanged = false;
			return ;
		}
	}
	
	
	
	
	
	
	LCD_WriteValue(1,80,Config.GyroParams[0].Gain,3,0==subpage);
	LCD_WriteValue(2,80,Config.GyroParams[0].Limit,3,1==subpage);
	LCD_WriteValue(3,80,Config.GyroParams[1].Gain,3,2==subpage);
	LCD_WriteValue(4,80,Config.GyroParams[1].Limit,3,3==subpage);
}



void _hSelfLeveling()
{

	NOKEYRETURN;
	PageKey(3);
	
	if (KEY4)
	{
		bValueChanged = true;
		currentPage.softkeys = _skMENUSAVE;
		
		switch (subpage)
		{
			case 0: if (Config.SelfLevelMode==IMU_SelfLevelMode) Config.SelfLevelMode=0; else Config.SelfLevelMode=IMU_SelfLevelMode; break;
			case 1: startEditMode(&(Config.AccGain),0,200,TYPE_UINT8); return ;   // we make return to avoid printing main screen while the editing editing
			case 2: startEditMode(&(Config.AccLimit),0,200,TYPE_UINT8); return ;
		}
	}

		
	if (KEY1)
	{
		if (bValueChanged==true)
		{
			Save_Config_to_EEPROM();
			bValueChanged = false;
			return ;
		}
	}

	
	
	
	lcdReverse(subpage == 0);
	if (Config.SelfLevelMode==IMU_SelfLevelMode)
	{
		strcpy_P(sXDeg,strYes);
	}
	else
	{
		strcpy_P(sXDeg,strNo);
	}
	LCD_WriteStringex (1,80,sXDeg,0==subpage);
	LCD_WriteValue(2,80,Config.AccGain,3,1==subpage);
	LCD_WriteValue(3,80,Config.AccLimit,3,2==subpage);
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
		LCD_WriteString_P(PSTR("LX")); // A Rate
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
	
 
	IMU_CalculateAngles ();
 
	int16_t t=ADCPort_Get(ACC_X_PNUM);
	AccTotal += t -OldAcc ; //Config.Sensor_zero[ACC_X_Index];
	OldAcc = t;

	utoa(AccTotal, sXDeg,10);

	LCD_SetPos(1,48);
	strcat_P(sXDeg,strSPC3);
	LCD_WriteString(sXDeg);	
 
 	dtostrf( CompAngleX, 3, 4, sXDeg);

	LCD_SetPos(4,48);
	strcat_P(sXDeg,strSPC3);
	LCD_WriteString(sXDeg);

	dtostrf( CompAngleY, 3, 4, sXDeg);
	LCD_SetPos(5,48);
	strcat_P(sXDeg,strSPC3);

	LCD_WriteString(sXDeg);
	LCD_WriteString_P(strSPC4);

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
			Menu_LoadPage(PAGE_HOME);
	}
	
	
	if (KEYPRESS)
		Beeper_Beep(70,1);
		
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
