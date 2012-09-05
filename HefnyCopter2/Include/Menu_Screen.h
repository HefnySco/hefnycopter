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

/*P_STR scrPIEditor[] = 
	"Axis:\n"
	"\n"
	"P Gain:\n"
	"P Limit:\n"
	"I Gain:\n"
	"I Limit:";
*/

P_STR scrHomePage[] = 
	"HefnyCopter v0.2\n"
	"\n"
	"Batt:\n"
	"\n"
	"\n"
	"RX:    SN:    ST:"
	;
	
P_STR scrHomePageArmed[] = 
	"\n"
	"\n"
	"\n"
	"M1:       M4:\n"
	"M2:       M3:\n"
	"GX:       GY:\n"
	"AX:       AY:\n"
	;
	
P_STR scrSelfLeveling[] =
	"\n"
	"Active:\n"
	"Acc Gain:\n"
	"Acc Limit:\n"
	;
	
	
P_STR scrStabilization[]=
	"\n"
	"P&R Gain:\n"
	"P&R Limit:\n"
	"Yaw-Gain:\n"
	"Yaw-Limit:\n"
	;

P_STR scrReceiverTest[] = 
	"Ail:\n"
	"Ele:\n"
	"Rud:\n"
	"Thr:\n"
	"Aux:";

P_STR scrSensorTest[] = 
	"Gyro X:\n"
	"Gyro Y:\n"
	"Gyro Z:\n"
	"Acc  X:\n"
	"Acc  Y:\n"
	"Acc  Z:\n"
	"Batt:";

P_STR scrModeSettings[] = 
	"Auto Disarm:"
	;

P_STR scrStickScaling[] = 
	"Stick Scaling\n"
	"\n"
	"Roll (Ail):\n"
	"Pitch (Ele):\n"
	"Yaw (Rud):\n"
	"Throttle:";

P_STR scrMiscSettings[] = 
	"Minimum throttle:\n"
	"Height Dampening:\n"
	"Height D. Limit:\n"
	"Alarm 1/10 volts:\n"
	"Servo filter:";



P_STR scrCPPMSettings[] = 
	"Roll (Ail):\n"
	"Pitch (Ele):\n"
	"Yaw (Rud):\n"
	"Throttle:\n"
	"Aux:";
	
P_STR scrSensorCal0[] = 
	"Place the aircraft on\n"
	"a level surface and\n"
	"press CONTINUE.\n"
	"The FC will then wait\n"
	"5 sec to let the\n"
	"aircraft settle down.";
P_STR scrSensorCal1[] = 
	"Calibration failed.\n"
	"\n"
	"Make sure the air-\n"
	"craft is level and\n"
	"stationary, and try\n"
	"again.";

PROGMEM const prog_char* scrSensorCal[] = 
	{
		scrSensorCal0, 
		scrSensorCal1
	};

P_STR scrESCCal0[] = 
	"1 TAKE OFF THE\n"
	"  PROPELLERS!!\n"
	"2 Check the throttle\n"
	"  direction in the\n"
	"  receiver test menu.\n"
	"  Reverse if\n"
	"  necessary.";
P_STR scrESCCal1[] = 
	"3 Memorize the rest\n"
	"  of the instructions\n"
	"  because the next\n"
	"  step is to turn off\n"
	"  the power.\n"
	"4 Turn off the power.\n"
	"5 Turn on the radio.";
P_STR scrESCCal2[] = 
	"6 Set the throttle to\n"
	"  full.\n"
	"7 Press down button\n"
	"  1 and 4 and keep\n"
	"  them down until the\n"
	"  last step.\n"
	"8 Turn on power.";	
P_STR scrESCCal3[] = 
	"9 Wait for the ESCs\n"
	"  full-throttle cali-\n"
	"  bration sound.\n"
	"10 Set the throttle\n"
	"  to idle.\n"
	"11 Wait for the idle";		
P_STR scrESCCal4[] = 
	"  calibration sound.\n"
	"12 Release the\n"
	"  buttons.\n"
	"13 Check if the mot-\n"
	"  ors start at the\n"
	"  same time. If not,\n"
	"  adjust the";	
P_STR scrESCCal5[] = 
	"  'Minimum throttle'\n"
	"  in the 'Misc. Set-\n"
	"  tings menu.\n"
	"\n"
	"  You are now done!";

PROGMEM const prog_char* scrESCCal[] = 
	{
		scrESCCal0,
		scrESCCal1,
		scrESCCal2,
		scrESCCal3,
		scrESCCal4,
		scrESCCal5,
	};

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
static const prog_char _skHOME[]     = "                 MENU";
static const prog_char _skMENU[]      = "BACK UP   DOWN ENTER";
static const prog_char _skMENUSAVE[]  = "SAVE UP   DOWN ENTER";
static const prog_char _skBACK[]      = "BACK";
static const prog_char _skCONTINUE[]  = "BACK         CONTINUE";
static const prog_char _skCANCELYES[] = "NO	             YES";
static const prog_char _skPAGE[]      = "BACK PREV NEXT CHANGE";
static const prog_char _skBACKNEXT[]  = "BACK NEXT";
static const prog_char _skCANCEL[]    = "CANCEL";
static const prog_char _skEDIT[]      = "CANCEL  DOWN  UP  OK";
//////////////////////////////////////////////////////////////////////////
// Menu Structure

#define MENU_START_INDEX 3
void _hHomePage();
void _hMenu();
void _hHomeArmed();
void _hStabilization();
void _hSelfLeveling();
void _hReceiverTest();
void _hSensorTest();
void _hSensorCalibration();
void _hESCCalibration();
void _hStickCentering();
void _hShowModelLayout();
void _hLoadModelLayout();
void _hDebug();
void _hFactoryReset();

static const page_t pages[] PROGMEM = {
/*  0 */	{ _skHOME, _hHomePage, scrHomePage },		// non-menu item
/*  1 */	{ _skMENU, _hMenu},							// non-menu item
/*	2 */	{ _skBACK, _hHomeArmed, scrHomePageArmed},	// non-menu item
{ _skMENU, _hStabilization, scrStabilization},			// in case of extra adding non menu items  MENU_START_INDEX constant should be updated to indicate the start of the menu
{ _skMENU, _hSelfLeveling, scrSelfLeveling},	
{ _skPAGE, NULL, scrModeSettings},
{ _skPAGE, NULL, scrMiscSettings},
{ _skBACK, _hSensorTest, scrSensorTest},
{ _skBACK, _hReceiverTest, scrReceiverTest},
{ _skCONTINUE, _hSensorCalibration, scrSensorTest},
{ _skCONTINUE, _hStickCentering, scrReceiverTest},
///* 10 */	{ _skCONTINUE, _hESCCalibration, scrESCCal0},
///* 11 */	{ _skPAGE, NULL, scrCPPMSettings},
///* 13 */	{ _skPAGE, NULL, scrMixerEditor},
///* 15 */	{ _skMENU, _hLoadModelLayout },
{ _skBACK, _hDebug },
{ _skCANCELYES, _hFactoryReset },
};

 

static const prog_char *lstMenu[] PROGMEM = {
	strStabilization,
	strSelflevel,
	strModeSettings,
	strMiscSettings,
	//strSelflevelSettings,
	strSensorTest,
	strReceiverTest,
	strSensorCalibration,
	strRadioCalibration,
	//strESCCalibration,
	//strCPPMSettings,
	//strMixerEditor,
	//strLoadMotorLayout,
	strDebug,
	strFactoryReset,
};

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
#endif /* MENU_SCREEN_H_ */