/*
 * LED.h
 *
 * Created: 8/14/2012 1:02:18 AM
 *  Author: hefny
 */ 


#ifndef LED_H_
#define LED_H_

#include <inttypes.h>

class LED 
{
	public:
	LED();
	void FlashLED (uint16_t msDuration, uint8_t Times);

};



#endif /* LED_H_ */