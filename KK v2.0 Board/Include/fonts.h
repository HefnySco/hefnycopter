/*
 * fonts.h
 *
 * Created: 8/16/2012 9:31:52 AM
 *  Author: hefny
 
 Thanks to "Oliver Shulz" schulz@os-x.eu
 */ 

#ifndef FONTS_H_
#define FONTS_H_



#include <stdint.h>
#include <avr/pgmspace.h>

typedef struct  
{
	uint8_t sizeX, sizeY;
	PGM_P (*selector)(unsigned char);
} fontdescriptor_t;


extern fontdescriptor_t font4x6;
extern fontdescriptor_t font6x8;
extern fontdescriptor_t font12x16;

extern const fontdescriptor_t *fonts[];


#endif /* FONTS_H_ */