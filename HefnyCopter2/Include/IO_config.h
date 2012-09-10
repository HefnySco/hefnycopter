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


#define HEFNYCOPTER2_SIGNATURE 0xfe

///////////// LCD
#define LCD_PORT	PORTD
#define LCD_DDR		DDRD
#define LCD_PIN		PIND
#define LCD_CS_BIT	5
#define LCD_RST_BIT	6
#define LCD_A0_BIT	7
#define LCD_SCL_BIT	4
#define LCD_SDA_BIT	1

#define LCD_CS					_REG_BIT(LCD_PORT, LCD_CS_BIT)
#define LCD_RST					_REG_BIT(LCD_PORT, LCD_RST_BIT)
#define LCD_A0					_REG_BIT(LCD_PORT, LCD_A0_BIT)
#define LCD_SCL					_REG_BIT(LCD_PORT, LCD_SCL_BIT)
#define LCD_SDA					_REG_BIT(LCD_PORT, LCD_SDA_BIT)
#define LCD_CS_DIR				_REG_BIT(LCD_DDR, LCD_CS_BIT)
#define LCD_RST_DIR				_REG_BIT(LCD_DDR, LCD_RST_BIT)
#define LCD_A0_DIR				_REG_BIT(LCD_DDR, LCD_A0_BIT)
#define LCD_SCL_DIR				_REG_BIT(LCD_DDR, LCD_SCL_BIT)
#define LCD_SDA_DIR				_REG_BIT(LCD_DDR, LCD_SDA_BIT)

 

/// Keyboard
#define KEYBOARD_PORT			PORTB
#define KEYBOARD_DDR			DDRB
#define KEYBOARD_PIN			PINB
#define KEY_S1_BIT_PNUM				7
#define KEY_S2_BIT_PNUM				6
#define KEY_S3_BIT_PNUM				5
#define KEY_S4_BIT_PNUM				4

#define KEY_S1_BIT				REGISTER_BIT(PORTB,KEY_S1_BIT_PNUM)
#define KEY_S1_BIT_DIR			REGISTER_BIT(DDRB,KEY_S1_BIT_PNUM)
#define KEY_S2_BIT				REGISTER_BIT(PORTB,KEY_S2_BIT_PNUM)
#define KEY_S2_BIT_DIR			REGISTER_BIT(DDRB,KEY_S2_BIT_PNUM)
#define KEY_S3_BIT				REGISTER_BIT(PORTB,KEY_S3_BIT_PNUM)
#define KEY_S3_BIT_DIR			REGISTER_BIT(DDRB,KEY_S3_BIT_PNUM)
#define KEY_S4_BIT				REGISTER_BIT(PORTB,KEY_S4_BIT_PNUM)
#define KEY_S4_BIT_DIR			REGISTER_BIT(DDRB,KEY_S4_BIT_PNUM)


///////////// LED
#define LED_Orange				REGISTER_BIT(PORTB,3)
#define LED_Orange_DIR 			REGISTER_BIT(DDRB,3)

///////////// Buzzer
#define Buzzer 					REGISTER_BIT(PORTB,1)
#define Buzzer_DIR 				REGISTER_BIT(DDRB,1)



///////////// Digital Inputs
// inputs from receiver

#define PWM_MIN				800		// was 922
#define PWM_LOW				1120	// used if output = 0
#define PWM_MID				1520
#define PWM_MAX				2200	// was 2118
#define PWM_CAL_RANGE		100
#define PPM_SYNC_LENGTH		3000

#define ORIGINAL_INPUT_CABLES

#ifdef ORIGINAL_INPUT_CABLES

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


#else

// Reduced cables configurations:
#define RX_ROLL_vect		INT0_vect
#define RX_ROLL    			REGISTER_BIT(PIND,2)	
#define RX_ROLL_DIR   		REGISTER_BIT(DDRD,2)

#define RX_PITCH_vect		PCINT3_vect
#define RX_PITCH    		REGISTER_BIT(PIND,0)	
#define RX_PITCH_DIR   		REGISTER_BIT(DDRD,0)

#define RX_COLL_vect		INT1_vect
#define RX_COLL    			REGISTER_BIT(PIND,3)	
#define RX_COLL_DIR   		REGISTER_BIT(DDRD,3) //pd0

#define RX_YAW_vect			INT2_vect
#define RX_YAW    			REGISTER_BIT(PINB,2)    
#define RX_YAW_DIR   		REGISTER_BIT(DDRB,2)


#define RX_AUX_vect			PCINT1_vect
#define RX_AUX    			REGISTER_BIT(PINB,0)    
#define RX_AUX_DIR   		REGISTER_BIT(DDRB,0) //pb0


#endif

// Index for RX[] variable
#define RXChannel_AIL			0
#define RXChannel_ELE			1
#define RXChannel_RUD			2
#define RXChannel_THR			3
#define RXChannel_AUX			4
#define RXChannels				5

///////////// Sensors
#define ACC_X				REGISTER_BIT(PORTA,5)		
#define ACC_Y				REGISTER_BIT(PORTA,6)		
#define ACC_Z				REGISTER_BIT(PORTA,7)		
#define GYRO_X				REGISTER_BIT(PORTA,1)		
#define GYRO_Y				REGISTER_BIT(PORTA,4)		
#define GYRO_Z				REGISTER_BIT(PORTA,2)		
#define V_BAT				REGISTER_BIT(PORTA,3)		
#define V_VCC				REGISTER_BIT(PORTA,0)	

#define ACC_X_PNUM			5
#define ACC_Y_PNUM			6
#define ACC_Z_PNUM			7
#define GYRO_X_PNUM			1
#define GYRO_Y_PNUM			4
#define GYRO_Z_PNUM			2
#define V_BAT_PNUM			3

// Order is aligned with Menu Screens
#define ACC_X_Index			3
#define ACC_Y_Index			4
#define ACC_Z_Index			5
#define GYRO_X_Index		0
#define GYRO_Y_Index		1
#define GYRO_Z_Index		2
#define V_BAT_Index			6
#define SENSORS_ALL			7

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