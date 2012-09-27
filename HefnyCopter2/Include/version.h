/*
 * version.h
 *
 * Created: 18-Sep-12 5:10:21 AM
 *  Author: M.Hefny
 */ 


#ifndef VERSION_H_
#define VERSION_H_


static const prog_char strVersionInfo[] = "HefnyCopter2 v0.45";

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
* -Buddy System. You can use two receivers at the same time and switch between them using AUX.
* -Compile for XBEE
* -Disable switch actions when entering Stick related menus.
* Disable switch actions when sticks are not calibrated.
* Only Secondary RX can have sticks-button.
* Add Voltage Monitor Alarm + menu in misc.


ToDO:
version 0.5
* Disable StickLanguage while setting receiver range [important]
* Add Timout for Disarm
* Move voltage port to default (ADC0) connected to VCC by default to free ADC1
* Motor Layout
* Store Quadcopter Flying Modes in eeprom using menu.

version 0.6
* Motor Mixing 
* Enable reduced RX cable configuration...should be implemented through mixer + text renaming for signals.
* Enhanced Receiver Test menu.... show left & right words next to controls.

version 0.7
* Self Landing when battery is weak.

*/

#endif /* VERSION_H_ */