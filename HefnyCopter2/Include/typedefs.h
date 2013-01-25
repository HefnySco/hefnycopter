#ifndef TYPE_DEFS_H_
#define TYPE_DEFS_H_

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>




#define PAGE_HOME					0
#define PAGE_MENU					1
#define PAGE_HOME_ARMED				2
#define PAGE_HOME_ESC_CALIBRATION	3
#define PAGE_RESTART				4


#define PAGE_STABILIZATION			5
#define PAGE_SELF_LEVELING			6
#define PAGE_MODE_SETTING			7
#define PAGE_MISC_SETTING			8
#define PAGE_SENSOR_TEST			9
#define PAGE_RECEIVER_TEST			10
#define PAGE_SENSOR_CALIBRATION		11
#define PAGE_STICK_CENTER			12
#define PAGE_ESC_CALIBRATION		13
#define PAGE_DEBUG					14
#define PAGE_SHOW_LAYOUT			14


#define P_STR static const prog_char





//set bit	or PORTB |= (1<<3); 
//#define set_bit(port, bit) ((port) |= (uint8_t)(1 << bit))
//clear bit
//#define clr_bit(port, bit) ((port) &= (uint8_t)~(1 << bit))	

#define INPUT	0
#define OUTPUT	1
#define LOW		0
#define HIGH	1
#define ON		1
#define OFF		0	

typedef enum _BOOL { FALSE = 0, TRUE } BOOL;
#define length(array) (sizeof(array)/sizeof(*array))


// Code courtesy of: stu_san on AVR Freaks

typedef struct
{
  unsigned int bit0:1;
  unsigned int bit1:1;
  unsigned int bit2:1;
  unsigned int bit3:1;
  unsigned int bit4:1;
  unsigned int bit5:1;
  unsigned int bit6:1;
  unsigned int bit7:1;
} _io_reg; 


#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt



typedef struct  
{
	unsigned int bit0 : 1;
	unsigned int bit1 : 1;
	unsigned int bit2 : 1;
	unsigned int bit3 : 1;
	unsigned int bit4 : 1;
	unsigned int bit5 : 1;
	unsigned int bit6 : 1;
	unsigned int bit7 : 1;
} volatile _bitreg8;
#define _REG_BIT2(r,b)	((*(_bitreg8*)&r).bit ## b)
#define _REG_BIT(r,b)	_REG_BIT2(r,b)


#define P_STR static const prog_char

#define uint8_tswap(a, b) { uint8_t t = a; a = b; b = t; } 


#endif
