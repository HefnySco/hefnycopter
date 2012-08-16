/*
 * Beeper.h
 *
 * Created: 8/13/2012 11:07:12 PM
 *  Author: hefny
 */ 


#ifndef BEEPER_H_
#define BEEPER_H_

#include <inttypes.h>


class Beeper 
{
	public:
	Beeper();
	void Beep (uint16_t msDuration, uint8_t Times);
	
	
};



#endif /* BEEPER_H_ */