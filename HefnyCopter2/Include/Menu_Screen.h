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

P_STR scrSelfLeveling[] =
	"Active:\n"
	"Gain:\n"
	"Trim Roll:\n"
	"Trim Pitch:";


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
static const prog_char _skBACK[]      = "BACK";
static const prog_char _skCONTINUE[]  = "BACK         CONTINUE";
static const prog_char _skCANCELYES[] = "NO	             YES";
static const prog_char _skPAGE[]      = "BACK PREV NEXT CHANGE";
static const prog_char _skBACKNEXT[]  = "BACK NEXT";
static const prog_char _skCANCEL[]    = "CANCEL";
static const prog_char _skEDIT[]      = "CANCEL  DOWN  UP  OK";
//////////////////////////////////////////////////////////////////////////
// Menu Structure

void _hHomePage();
void _hMenu();
void _hReceiverTest();
void _hSelfLeveling();
void _hSensorTest();
void _hSensorCalibration();
void _hESCCalibration();
void _hStickCentering();
void _hShowModelLayout();
void _hLoadModelLayout();
void _hDebug();
void _hFactoryReset();

static const page_t pages[] PROGMEM = {
/*  0 */	{ _skHOME, _hHomePage, scrHomePage },
/*  1 */	{ _skMENU, _hMenu},
/*  2 */	{ _skBACK, _hSelfLeveling, scrSelfLeveling},
/*  3 */	{ _skBACK, _hReceiverTest, scrReceiverTest},
/*  4 */	{ _skPAGE, NULL, scrModeSettings},
/*  5 */	{ _skPAGE, NULL, scrStickScaling},
/*  6 */	{ _skPAGE, NULL, scrMiscSettings},
///*  7 */	{ _skPAGE, NULL, scrSelflevelSettings},
/*  8 */	{ _skBACK, _hSensorTest, scrSensorTest},
/*  9 */	{ _skCONTINUE, _hSensorCalibration, scrSensorTest},
/* 12 */	{ _skCONTINUE, _hStickCentering, scrReceiverTest},
///* 10 */	{ _skCONTINUE, _hESCCalibration, scrESCCal0},
///* 11 */	{ _skPAGE, NULL, scrCPPMSettings},
///* 13 */	{ _skPAGE, NULL, scrMixerEditor},
/* 14 */	{ _skBACKNEXT, _hShowModelLayout},
///* 15 */	{ _skMENU, _hLoadModelLayout },
/* 16 */	{ _skBACK, _hDebug },
/* 16 */	{ _skCANCELYES, _hFactoryReset },
};

 

static const prog_char *lstMenu[] PROGMEM = {
	strSelflevel,
	strReceiverTest,
	strModeSettings,
	strStickScaling,
	strMiscSettings,
	//strSelflevelSettings,
	strSensorTest,
	strSensorCalibration,
	strRadioCalibration,
	//strESCCalibration,
	//strCPPMSettings,
	//strMixerEditor,
	strShowMotorLayout,
//  strLoadMotorLayout,
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


#define PAGE_HOME			0
#define PAGE_MENU			1
#define PAGE_SHOW_LAYOUT	14


#define TYPE_UINT8		2
#define TYPE_INT8		1
#define TYPE_INT16		3
#endif /* MENU_SCREEN_H_ */