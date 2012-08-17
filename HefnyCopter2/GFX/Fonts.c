/*
 * fonts.c
 *
 * Created: 14.08.2012 17:41:29
 *  Author: OliverS
 *
 * $Id: fonts.c 27 2012-08-15 18:18:23Z olischulz24@googlemail.com $
 */ 

#include <avr/pgmspace.h>

#include "../Include/fonts.h"


const fontdescriptor_t *fonts[] PROGMEM = {
	&font4x6,
	&font6x8,
	&font12x16,
};

