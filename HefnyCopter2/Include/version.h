/*
 * version.h
 *
 * Created: 18-Sep-12 5:10:21 AM
 *  Author: M.Hefny
 */ 


#ifndef VERSION_H_
#define VERSION_H_


static const prog_char strVersionInfo[] = "HefnyCopter2 v0.4";

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

ToDO:
version 0.5
* Disable StickLanguage while setting receiver range [important]
* Move voltage port to default (ADC0) connected to VCC by default to free ADC1
* Motor Layout
* Store Quadcopter Flying Modes in eeprom using menu.

version 0.6
* Motor Mixing 
* Enable reduced RX cable configuration...should be implemented through mixer + text renaming for signals.
* Enhanced Receiver Test menu.... show left & right words next to controls.


*/

#endif /* VERSION_H_ */