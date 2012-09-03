/*
 * KeyBoard.h
 *
 * Created: 8/16/2012 8:23:04 AM
 *  Author: hefny
 */ 
 

#ifndef KEYBOARD_H_
#define KEYBOARD_H_
 

// KeyBoard
uint8_t _mykey;
uint8_t _TXKeys;
#define KEY_INIT	1
#define KEY_REFRESH	2
#define IS_INIT			(_mykey & KEY_INIT)
#define IS_KEYREFRESH	(_mykey & KEY_REFRESH)
#define KEY1			(_mykey & KEY_1)
#define KEY2			(_mykey & KEY_2)
#define KEY3			(_mykey & KEY_3)
#define KEY4			(_mykey & KEY_4)  
#define ANYKEY			(_mykey)
#define KEYPRESS		(_mykey & (KEY_1|KEY_2|KEY_3|KEY_4))
#define NOKEYRETURN		{if (!_mykey) return;}



#define KEY_1		_BV(KEY_S1_BIT_PNUM	)
#define KEY_2		_BV(KEY_S2_BIT_PNUM	)
#define KEY_3		_BV(KEY_S3_BIT_PNUM	)
#define KEY_4		_BV(KEY_S4_BIT_PNUM	)

#define ALLKEYS		(KEY_1 | KEY_2 | KEY_3 | KEY_4)



	void KeyBoard_Init (void);
	uint8_t Keyboard_Read(void);
	
	
	uint8_t lastKeys;
	
	
	uint8_t Keyboard_State();




#endif /* KEYBOARD_H_ */