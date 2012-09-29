/*
 * Beeper.h
 *
 * Created: 8/13/2012 11:07:12 PM
 *  Author: hefny
 */ 


#ifndef BEEPER_H_
#define BEEPER_H_

#include <inttypes.h>

	#define BEEP_LONG	700
	#define BEEP_SHORT	70
	#define BEEP_KEY	70
	#define BEEP_ERROR	200
	
	void Beeper_Beep (uint16_t msDuration, uint8_t Times);
	



#endif /* BEEPER_H_ */