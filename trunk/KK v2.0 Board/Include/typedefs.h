#ifndef TYPE_DEFS_H_
#define TYPE_DEFS_H_

#include <stdint.h>
#include <avr/io.h>

#define F_CPU	20000000


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

/* Example:

#define BUTTON_PIN  REGISTER_BIT(PINB,3)
#define LED_PORT    REGISTER_BIT(PORTB,4)

#define BUTTON_DIR  REGISTER_BIT(DDRB,3)
#define LED_DIR     REGISTER_BIT(DDRB,4)
 
main() 
{
	uint8_t is_button = BUTTON_PIN;
	// this actually is expanded by the C preprocessor to:
	// uint8_t is_button = ((volatile _io_reg*)&PINB)->bit3; 

  LED_DIR = 1;
	// which after the preprocessor looks like: 
	// ((volatile _io_reg*)&DDRB)->bit4 = 1; 

  BUTTON_DIR = 0;

  while (1) {
    LED_PORT = BUTTON_PIN;
  }
} 

*/


#define uint8_tswap(a, b) { uint8_t t = a; a = b; b = t; } 

#endif