/*
 * version.h
 *
 * Created: 18-Sep-12 5:10:21 AM
 *  Author: M.Hefny
 */ 


#ifndef VERSION_H_
#define VERSION_H_


static const prog_char strVersionInfo[] = "HefnyCopter2 v0.9.1";

/*
version 0.1
* incomplete version no flying yet.

version 0.2
* incomplete version very unstable flying parameters.
* Menu structure has started to be stable.

version 0.3
* incomplete version
* Complementry Filter implemented.
* Kalman Filter implemented.
* First stable flying trials.

version 0.4
* Sensor Readings data is displayed while calibration. After calibration still display live sensor readings instead of Sensor_Zeros values.
  The data should be 0 +-1 except the AccZ which should be 100+-1
* Enable ESC calibration.
* Remove version message -not needed-.. already in home page.
* Quadcopter Flying Modes:
	- display configuration on homepage.
* Disarm Quad when playing with Menu if Armed.

version 0.45
* Smart Wiring - you can use short cable connection or normal connection [ORIGINAL_INPUT_CABLES/PRIMARY_INPUT_RX]
* Use Motor5,6,7,8 as RX2 Input. This releases UART1 for another use, or can use parallel RX for training. [SECONDARY_INPUT_RX]
* Buddy System. You can use two receivers at the same time and switch between them using AUX.
* Compile for XBEE
* Disable switch actions when entering Stick related menus.
* Disable switch actions when sticks are not calibrated.
* Only Secondary RX can have sticks-button.
* Add Voltage Monitor Alarm + menu in misc.

version 0.46
* Misc & Mode menu updated.
* Acc Menu is used for Complementary filter of Gyro
* Add Gyro reverse by allowing negative P in P2D
* Complementary filter to Gyro and update screen menu 

version 0.47
* PID instead of P2D
* New screen PID for ACC
* Complementary filter values for Gyro XY , Gyro X, ACC XY, ACC Z.
* fixed UI issue in Sensor Calibration

version 0.48
* PID_I zero when throttle off -reset-.
* calibration condition bug fixed.
* while (TCNT2 < 45); //=8us : used 45 instead of 50 to reduce default pulse width...

version 0.5
* enable sending config data via USART
* Restructure config to optimize transmitted config through UART - dont send unwanted data at the end of structure.
* Adjust minor text issue of menu -  make sure all are of the same length


version 0.51
* Fail Alarm: if signal is lost alarm is on.

version 0.52
* Fixed a major issue in calculating I in PID. [YAW issue disappeared after fixing using I in YAW PID]
* Split PID for Gyro & Acc

version 0.6
* Adding RUD, AIL, ELE deviation of ACC data in Plus mode.


version 0.7
* UI:
*	. Many improvement on Menu UI wording and styles.
*	.  Disabled menu items with flashing to allow more smoother setup.
*	.  Adding Board Positioning X or Plus... more flexible to your frame
*	.  Disable some menu items if Sticks and Sensors are not calibrated to ease setting process.
*	.  Adding Restart Screen notification.
*	.  If Voltage error then voltage on homepage is highlighted.
* IMU:
*	. Update Landing Logic.
*	. Acc_Z =0 instead of 100 as a default value.
*	. Increase (I) only when there is a condition: if (abs(Value) > abs(PID_Term->Error))
*	. Update Logic for PID_ACC to ZERO I....removed lated it caused wobbles. 
* Function:
*	. Adjusting ESC Calibration... was not working correct... now the signal comes from ISR of RX to M1..4 directly to guarantee correct programming signal length.
*	. RX_Main handles acts as buttons even if RX_Trainer is selected..
*		.. You can arm only with the active RX.
*		.. Even if RX_Main is selected you cannot ARM if RX_Trainer signal does not exist.
*

version 0.8
* IMU:
*	. Adding Board-Orientation and saved in EEPRom
* Function:
*	. When turning off TX during armed ... motors off and buzzer starts.
*	. When turning off TX during disarmed ... no beeps - issue fixed-.
*   . Adding stick scaling

version 0.8.1
* IMU:
*	. Adding Differential Value in I instead of Value.

version 0.8.4
* UART
*	. Fixing minor issue in RX logic.
*	. Update values from UART TX...GUI Tool on windows have been updated and tested.

version 0.8.5
* IMU:
*	. Adding Trims for ACC

version 0.9.0
* Function:
*	. Motors Layout is compatible with KK2 M1,M2,M3,M4 TOP,RIGHT,BOTTOM,LEFT
*	. You can save default flying mode X or Plus so u no longer to switch to X each time if u fly X constantly.

version 0.9.1
* IMU:
*	. Tuning Gyro & ACC reading in leveling mode.
*	. fixing Board X & Flying Plus.
* Function:
*	. Disable command when armed.

=========================================================================================
* Move voltage port to default (ADC0) connected to VCC by default to free ADC1

* Motor Mixing 
* Enhanced Receiver Test menu.... show left & right words next to controls.


* Self Landing when battery is weak.

*/

#endif /* VERSION_H_ */