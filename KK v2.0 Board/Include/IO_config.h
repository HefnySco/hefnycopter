/*
 * IO_config.h
 *
 * Created: 8/13/2012 6:34:09 PM
 *  Author: hefny
 */ 


#ifndef IO_CONFIG_H_
#define IO_CONFIG_H_
#include <avr/io.h>

#include "typedefs.h"



///////////// LCD
#define LCD_CS1					REGISTER_BIT(PORTD,5)
#define LCD_CS1_DIR				REGISTER_BIT(DDRD,5)
#define LCD_RES					REGISTER_BIT(PORTD,6)
#define LCD_RES_DIR				REGISTER_BIT(DDRD,6)
#define LCD_A0					REGISTER_BIT(PORTD,7)
#define LCD_A0_DIR				REGISTER_BIT(DDRD,7)
#define LCD_SClK				REGISTER_BIT(PORTD,4)
#define LCD_SClK_DIR			REGISTER_BIT(DDRD,4)
#define LCD_SI					REGISTER_BIT(PORTD,1)
#define LCD_SI_DIR				REGISTER_BIT(DDRD,1)

#define LCD_CS		_REG_BIT(LCD_PORT, LCD_CS_BIT)
#define LCD_RST		_REG_BIT(LCD_PORT, LCD_RST_BIT)
#define LCD_A0		_REG_BIT(LCD_PORT, LCD_A0_BIT)
#define LCD_SCL		_REG_BIT(LCD_PORT, LCD_SCL_BIT)
#define LCD_SDA		_REG_BIT(LCD_PORT, LCD_SDA_BIT)
#define LCD_CS_DIR	_REG_BIT(LCD_DDR, LCD_CS_BIT)
#define LCD_RST_DIR	_REG_BIT(LCD_DDR, LCD_RST_BIT)
#define LCD_A0_DIR	_REG_BIT(LCD_DDR, LCD_A0_BIT)
#define LCD_SCL_DIR	_REG_BIT(LCD_DDR, LCD_SCL_BIT)
#define LCD_SDA_DIR	_REG_BIT(LCD_DDR, LCD_SDA_BIT)

#define LCD_PORT	PORTD
#define LCD_DDR		DDRD
#define LCD_PIN		PIND
#define LCD_CS_BIT	5
#define LCD_RST_BIT	6
#define LCD_A0_BIT	7
#define LCD_SCL_BIT	4
#define LCD_SDA_BIT	1

/// Keyboard
#define KEYBOARD_PORT			PORTB
#define KEYBOARD_DDR			DDRB
#define KEYBOARD_PIN			PINB
#define KEY_S1_BIT_PNUM				7
#define KEY_S2_BIT_PNUM				6
#define KEY_S3_BIT_PNUM				5
#define KEY_S4_BIT_PNUM				4

#define KEY_S1_BIT				REGISTER_BIT(PORTB,7)
#define KEY_S1_BIT_DIR			REGISTER_BIT(DDRB,7)
#define KEY_S2_BIT				REGISTER_BIT(PORTB,6)
#define KEY_S2_BIT_DIR			REGISTER_BIT(DDRB,6)
#define KEY_S3_BIT				REGISTER_BIT(PORTB,5)
#define KEY_S3_BIT_DIR			REGISTER_BIT(DDRB,5)
#define KEY_S4_BIT				REGISTER_BIT(PORTB,4)
#define KEY_S4_BIT_DIR			REGISTER_BIT(DDRB,4)


///////////// LED
#define LED_Orange				REGISTER_BIT(PORTB,3)
#define LED_Orange_DIR 			REGISTER_BIT(DDRB,3)

///////////// Buzzer
#define Buzzer 					REGISTER_BIT(PORTB,1)
#define Buzzer_DIR 				REGISTER_BIT(DDRB,1)



///////////// Digital Inputs
// inputs from receiver



#define RX_ROLL_vect		INT1_vect
#define RX_ROLL    			REGISTER_BIT(PIND,3)	
#define RX_ROLL_DIR   		REGISTER_BIT(DDRD,3)

#define RX_PITCH_vect		INT0_vect
#define RX_PITCH    		REGISTER_BIT(PIND,2)	
#define RX_PITCH_DIR   		REGISTER_BIT(DDRD,2)

#define RX_COLL_vect		PCINT3_vect
#define RX_COLL    			REGISTER_BIT(PIND,0)	
#define RX_COLL_DIR   		REGISTER_BIT(DDRD,0) //pd0

#define RX_YAW_vect			INT2_vect
#define RX_YAW    			REGISTER_BIT(PINB,2)    
#define RX_YAW_DIR   		REGISTER_BIT(DDRB,2)


#define RX_AUX_vect			PCINT1_vect
#define RX_AUX    			REGISTER_BIT(PINB,0)    
#define RX_AUX_DIR   		REGISTER_BIT(DDRB,0) //pb0



///////////// Sensors
#define ACC_X				REGISTER_BIT(PORTA,5)		
#define ACC_Y				REGISTER_BIT(PORTA,6)		
#define ACC_Z				REGISTER_BIT(PORTA,7)		
#define GYRO_X				REGISTER_BIT(PORTA,1)		
#define GYRO_Y				REGISTER_BIT(PORTA,4)		
#define GYRO_Z				REGISTER_BIT(PORTA,2)		
#define V_BAT				REGISTER_BIT(PORTA,3)		
#define V_VCC				REGISTER_BIT(PORTA,0)	


///////////// Motors
#define M1		  				REGISTER_BIT(PORTC,6)
#define M2		  				REGISTER_BIT(PORTC,4)
#define M3		  				REGISTER_BIT(PORTC,2)
#define M4		  				REGISTER_BIT(PORTC,3)
#define M5		  				REGISTER_BIT(PORTC,1)
#define M6		  				REGISTER_BIT(PORTC,0)
#define M7		  				REGISTER_BIT(PORTC,5)
#define M8		  				REGISTER_BIT(PORTC,7)
#define M1_DIR 					REGISTER_BIT(DDRC,6)
#define M2_DIR 					REGISTER_BIT(DDRC,4)
#define M3_DIR 					REGISTER_BIT(DDRC,2)
#define M4_DIR 					REGISTER_BIT(DDRC,3)
#define M5_DIR 					REGISTER_BIT(DDRC,1)
#define M6_DIR 					REGISTER_BIT(DDRC,0)
#define M7_DIR 					REGISTER_BIT(DDRC,5)
#define M8_DIR 					REGISTER_BIT(DDRC,7)




#endif /* IO_CONFIG_H_ */