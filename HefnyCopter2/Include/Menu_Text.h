/*
 * menu_text.h
 *
 * Created: 02.08.2012 08:10:28
 *  Author: 
 *			M.S. Hefny
 *			OliverS - http://code.google.com/p/kk2-multicopter/
 *
 * 
 */ 
 
#ifndef MENU_TEXT_H_
#define MENU_TEXT_H_

#include "../Include/typedefs.h"


#define PAGE_HOME			0
#define PAGE_MENU			1
#define PAGE_HOME_ARMED		2
#define PAGE_SHOW_LAYOUT	14



P_STR strSAFE[] =	"SAFE";
P_STR strARMED[] =  "ARMED";
P_STR strOFF[] =	"OFF";
P_STR strON[] =		"ON ";
P_STR strNot[] =    "Not";
P_STR strErr[] =	"Err";
P_STR strSPC1[] =	" ";
P_STR strSPC3[] =	"   ";
P_STR strSPC4[] =	"    ";

P_STR strOK[] =		"OK ";
P_STR strYes[] =	"Yes";
P_STR strNo[] =		"No ";

// main menu
//P_STR strPIEditor[] = "PI Editor";
P_STR strStabilization[]= "Stabilization";
P_STR strReceiverTest[] = "Receiver Test";
P_STR strModeSettings[] = "Mode Settings";
P_STR strStickScaling[] = "Stick Scaling";
P_STR strRadioCalibration[] = "Stick Centering";
P_STR strMiscSettings[] = "Misc. Settings";

P_STR strSelflevel[] = "Self Leveling";
P_STR strSensorTest[] = "Sensor Test";
P_STR strSensorCalibration[] = "Sensor Calibration";
//P_STR strCPPMSettings[] = "CPPM Settings";
//P_STR strESCCalibration[] = "ESC Calibration";

P_STR strMixerEditor[] = "Mixer Editor";
P_STR strShowMotorLayout[] = "Show Model Layout";
P_STR strLoadMotorLayout[] = "Load Model Layout";
P_STR strDebug[] = "Debug";
P_STR strFactoryReset[] = "Factory Reset";



P_STR strIofPI[] = "I of PI";
P_STR strSpIsSp[] = " is ";

P_STR strRollAil[] = "Roll (Aileron)";
P_STR strPitchEle[] = "Pitch (Elevator)";
P_STR strYawRud[] = "Yaw (Rudder)";
P_STR strStick[] = "Stick";
P_STR strAUX[] = "AUX";
P_STR strESC[] = "ESC";
P_STR strServo[] = "Servo";
P_STR strHigh[] = "Hi";
P_STR strLow[] = "Lo";
P_STR strNoSignalFound[] = "no signal";
P_STR strNoSignalDis[] = "signal-lost";
P_STR strLeft[] = "Left";
P_STR strRight[] = "Right";
P_STR strBack[] = "Back";
P_STR strForward[] = "Frwrd";
P_STR strIdle[] = "Idle";
P_STR strFull[] = "Full";
P_STR strBattery[] = "Battery:";
P_STR strError[] = "Error:";

P_STR strRoll[]  = "Roll";
P_STR strPitch[] = "Pitch";
P_STR strYaw[]   = "Yaw";
P_STR strThro[]  = "Thro";

P_STR strWait[]       = "Waiting";
P_STR strSec[]        = "sec";
P_STR strCalSucc[]    = "Bad Calibration";
P_STR strAreYouSure[] = "Are you sure?";

P_STR strMotor[]	  = "Motor:";
P_STR strOutput[]     = "Output:";
P_STR strDirSeen[]    = "Direction\nseen from\nabove:";
P_STR strCW[]		  = "CW";
P_STR strCCW[]        = "CCW";
P_STR strALL[]        = "ALL";
P_STR strUnused[]     = "Unused.";

P_STR strSensorNotCal[] = "No calibration";

#endif /* MENU_TEXT_H_ */