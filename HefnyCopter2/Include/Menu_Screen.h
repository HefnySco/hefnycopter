/*
 * menu_screen_t.h
 *
 * Created: 02.08.2012 08:12:11
 *  Author: 
 *			M.S. Hefny
 *			OliverS - http://code.google.com/p/kk2-multicopter/
 *
 * 
 */ 


#ifndef MENU_SCREEN_H_
#define MENU_SCREEN_H_

#include "../Include/Version.h"



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



typedef struct
{
	uint8_t X, Y;
	void *valuePtr;
	int16_t loLimit, hiLimit;
	uint8_t len;
} edit_element_t;




////////////////////////////////////SCREEN TEXT/////////////////////////////////////

P_STR scrHomePage[] = 
	"\n"
	"\n"
	"Batt:     v\n"
	"\n"
	"\n"
	"RX: -  SN:    ST: -\n"
	"Fly:   Board:";
	
P_STR scrHomePageRestart[] = 
	"\n"
	"RESTART\n"
	"      BOARD"
	;
	
P_STR scrHomePageArmed[] = 
	"\n"
	"\n"
	"\n"
	/*
	"M1:       M4:\n"
	"M2:       M3:\n"
	"GX:       GY:\n"
	"AX:       AY:\n"
	*/
	;


P_STR scrHomePageArmedESCCalibration[] = 
	"\n"
	"\n"
	"\n"
	"M1:       M4:\n"
	"M2:       M3:\n"
	;

	
P_STR scrSelfLeveling[] =
	"\n"
	"LPF:\n"
	;
	
	
P_STR scrStabilization[]=
	"\n"
	"P:       Lim:\n"
	"I:       Lim:\n"
	"D:       Lim:\n"
	"Noise Filter:\n"
	"Trim P:    R:"    
	;

P_STR scrReceiverTest[] = 
	" CH   RX-1    RX-2\n"
	"Ail:\n"
	"Ele:\n"
	"Rud:\n"
	"Thr:\n"
	"Aux:\n"
	"RX:";

P_STR scrSensorTest[] = 
	"Gyro X:\n"
	"Gyro Y:\n"
	"Gyro Z:\n"
	"Acc  X:\n"
	"Acc  Y:\n"
	"Acc  Z:\n"
	"Batt:";

P_STR scrModeSettings[] = 
	"Buddy System:\n"
	"Board Dir:\n"
	"Fly   Dir:\n"
	;

//P_STR scrStickScaling[] = 
	//"Stick Scaling\n"
	//"\n"
	//"Roll (Ail):\n"
	//"Pitch (Ele):\n"
	//"Yaw (Rud):\n"
	//"Throttle:";

P_STR scrMiscSettings[] = 
	"Auto Disarm:\n"
	"Volt Alarm:      /10v\n"
	"Throttle min:\n"
	"Stick Scale:"
	;



P_STR scrCPPMSettings[] = 
	"Roll (Ail):\n"
	"Pitch (Ele):\n"
	"Yaw (Rud):\n"
	"Throttle:\n"
	"Aux:";


P_STR scrESCCal0[] = 
	"-TAKE OFF PROPELLERS!!\n"
	"-Press continue\n"
	"-Turn Off Quadcopter.\n"
	"\n"
	"-Quad will started\n"
	"in ARMED TAKECARE state\n";
	
P_STR scrESCCal1[] = 
	"Press Continue after calibration.\n"
	"\n"
	"unplug battery.\n";


/*P_STR scrRadioCal0[] = 
	"Move Thr & Ele up - down\n"
	"Move Rud & Ail left - right\n"
	"to max limits slowly.\n"
	"\n"
	"Press CONTINUE to\n"
	"save.";
*/
P_STR scrMixerEditor[] = 
	"Throttle:       Ch:\n"
	"Aileron:\n"
	"Elevator:\n"
	"Rudder:\n"
	"Offset:\n"
	"Type:       Rate:";


//////////////////////////////////////////////////////////////////////////
// softkeys
static const prog_char _skHOME[]      = "                 MENU";
static const prog_char _skMENU[]      = "BACK UP   DOWN  ENTER";
static const prog_char _skMENUSAVE[]  = "SAVE UP   DOWN  ENTER";
static const prog_char _skBACK[]      = "BACK                 ";
static const prog_char _skCONTINUE[]  = "BACK         CONTINUE";
static const prog_char _skCANCELYES[] = "NO                YES";
static const prog_char _skPAGE[]      = "BACK PREV NEXT CHANGE";
static const prog_char _skBACKNEXT[]  = "BACK NEXT			  ";
static const prog_char _skCANCEL[]    = "CANCEL				  ";
static const prog_char _skEDIT[]      = "CANCEL  DOWN   UP  OK";
static const prog_char _skSAVE[]      = "                 DONE";
static const prog_char _skCLRDONE[]   = "BACK CLR         DONE";
//////////////////////////////////////////////////////////////////////////
// Menu Structure

#define MENU_START_INDEX 5
void _hHomePage();
void _hMenu();
void _hHomeArmed();
void _hHomeArmedESC();
void _hHomeRestart();
void _hStabilization();
void _hSelfLeveling();
void _hReceiverTest();
void _hSensorTest();
void _hSensorCalibration();
void _hESCCalibration();
void _hStickCentering();
void _hShowModelLayout();
//void _hLoadModelLayout();
void _hModeSettings ();
void _hMiscSettings ();
void _hDebug();
void _hFactoryReset();

static const page_t pages[] PROGMEM = {
/*  0 */	{ _skHOME, _hHomePage, scrHomePage},						// non-menu item
/*  1 */	{ _skMENU, _hMenu, NULL},									// non-menu item
/*	2 */	{ _skHOME, _hHomeArmed, scrHomePageArmed},					// non-menu item
/*	3 */	{ _skSAVE, _hHomeArmedESC, scrHomePageArmedESCCalibration},	// non-menu item
/*	4 */	{ _skSAVE, _hHomeRestart, scrHomePageRestart},				// non-menu item
{ _skMENU, _hStabilization, scrStabilization},			// in case of extra adding non menu items  MENU_START_INDEX constant should be updated to indicate the start of the menu
{ _skMENU, _hSelfLeveling, scrStabilization},	
{ _skMENU, _hModeSettings, scrModeSettings},
{ _skPAGE, _hMiscSettings, scrMiscSettings},
{ _skBACK, _hSensorTest, scrSensorTest},
{ _skBACK, _hReceiverTest, scrReceiverTest},
{ _skCONTINUE, _hSensorCalibration, scrSensorTest},
{ _skCLRDONE, _hStickCentering, scrReceiverTest},
{ _skCONTINUE, _hESCCalibration, scrESCCal0},
///* 11 */	{ _skPAGE, NULL, scrCPPMSettings},
///* 13 */	{ _skPAGE, NULL, scrMixerEditor},
///* 15 */	{ _skMENU, _hLoadModelLayout },
{ _skBACK, _hDebug, NULL},
{ _skCANCELYES, _hFactoryReset, NULL},
};

 

static const prog_char *lstMenu[] PROGMEM = {
	strStabilization,
	strSelflevel,
	strModeSettings,
	strMiscSettings,
	strSensorTest,
	strReceiverTest,
	strSensorCalibration,
	strRadioCalibration,
	strESCCalibration,
	strDebug,
	strFactoryReset,
};

int8_t menuEnabled[16];

PGM_P tsmMain(uint8_t);
PGM_P tsmLoadModelLayout(uint8_t);

static uint8_t page, subpage,subindex;
static uint16_t _tStart;
static page_t currentPage;
static menu_t mnuMain = {length(lstMenu), tsmMain};
static menu_t mnuMLayout = {22, tsmLoadModelLayout};
int16_t editValue, editLoLimit, editHiLimit;
uint8_t editMode, editValueType;
uint8_t* editValuePtr;



#define TYPE_UINT8		2
#define TYPE_INT8		1
#define TYPE_INT16		3
#define TYPE_UINT16		4
#endif /* MENU_SCREEN_H_ */