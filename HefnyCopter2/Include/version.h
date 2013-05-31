/*
 * version.h
 *
 * Created: 18-Sep-12 5:10:21 AM
 *  Author: M.Hefny
 */ 


#ifndef VERSION_H_
#define VERSION_H_


static const prog_char strVersionInfo[] = "HefnyCopter2 v1.0.0";

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


version 0.9.2
* IMU:
*	. update rate of RX divide by 2.5 -sensitivity settings-.
*	. Allow flying Plus mode while board orientation is X. 

version 0.9.3
* Function:
*	. Change default mode to UART mode -UART 1 receiver + optional data AUX controls ACRO or STABLE-
*	. Stop motor from starting if you select ESC Calibration menu using smart sticks -TX Sticks-.
* UI
*	. Sensor test: can you fix strange dots behind numbers.... thanks Samsog in RcGroup [http://www.rcgroups.com/forums/showthread.php?t=1735340&page=5#post23981357]
*	. HomePage: display volt in v units using floating point.
*	. ESC Calibration Mode: update screen text to tell we are in ESC Cal Mode.

version 0.9.4
* Function:
*	. Allow SONAR for altitude hold.
*	. Allow switching to LEVELING & ACRO using ELE Stick up & down even in Buddy mode ... thanks Samog in RCGroup
* UI
*	. Fix Highlighted RX numbers ... active is highlighted ... thanks Samog in RCGroup
*	. Fix voltage > 9.99 display issue ...  thanks Samog in RCGroup



version 0.9.5
* Function:
*	. Adding AltHold Mode.

version 0.9.6
* IMU:
*	. Update stick factors on Level to 2
*	. Adding new PID section for Sonar
* UI:
*	. New menu Item for SONAR


version 0.9.7
* Function:
*	. Reduce sensitivity for ACRO than STABLE mode under same scale.
*   . True scaling of stick position to avoid different TX ranges.
IMU:
*	. Make accelerometer works always even in ACRO mode to enable good balancing switch when switching back to Stable mode.
*   . Improve Sonar Logic


version 0.9.8
IMU:
*	.YAW decoupling ... pls check http://scolton.blogspot.com/2012/09/fun-with-complementary-filter-multiwii.html

version 0.9.9
Function:
*	. TRI copter support.															: tested
*	. YAW DIRECTION																	: tested
*	. Flyi ing in A & Y mode.														: tested
*	. Change scale to 1 .. 20 ....[users may need to update stick sensitivity]
*   . Refuse Arming when configuration is TRI & Board is in X orientation.			: tested
*	. Fix LCD startup issue.
*	. Adding baud_rate variable based on [http://www.rcgroups.com/forums/showpost.php?p=24904574&postcount=258] ... requires compiling the code.

UI
*	. Adding YAW Direction. .... known bug {N & R doesnot change until we move the cursor}
*	. Mode: A & Y mode for TRI 
*	. Mode: A & Y mode in HomePage for TRI

version 1.0.0
Function:
*	. Pitch & Roll separated.
*	.ACC-Z complementary filter issue fixed.








=========================================================================================
* Move voltage port to default (ADC0) connected to VCC by default to free ADC1

* Motor Mixing 
* Enhanced Receiver Test menu.... show left & right words next to controls.


* Self Landing when battery is weak.

*/

#endif /* VERSION_H_ */