/*
 * Fonts.cpp
 *
 * Created: 8/16/2012 9:33:21 AM
 *  Author: hefny
 */ 

#include "../Include/fonts.h"
#include <avr/pgmspace.h>


const fontdescriptor_t *fonts[] PROGMEM = {
	&font4x6,
	&font6x8,
	&font12x16,
};

