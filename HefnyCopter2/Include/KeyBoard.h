/*
 * KeyBoard.h
 *
 * Created: 8/16/2012 8:23:04 AM
 *  Author: hefny
 */ 


#ifndef KEYBOARD_H_
#define KEYBOARD_H_


#define KEY_1	_BV(KEY_S1_BIT_PNUM	)
#define KEY_2	_BV(KEY_S2_BIT_PNUM	)
#define KEY_3	_BV(KEY_S3_BIT_PNUM	)
#define KEY_4	_BV(KEY_S4_BIT_PNUM	)

#define ALLKEYS (KEY_1 | KEY_2 | KEY_3 | KEY_4)



	void KeyBoard_Init (void);
	uint8_t Keyboard_Read(void);
	
	
	uint8_t lastKeys;
	
	
	uint8_t Keyboard_State();




#endif /* KEYBOARD_H_ */