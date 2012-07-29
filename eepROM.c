/*
 * eepROM.c
 *
 * Created: 23-Jul-12 3:38:49 PM
 *  Author: M.Hefny
 */ 

#include <avr/io.h>  
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h> 

#include "typedefs.h"
#include "io_cfg.h"
#include "Commons.h"
#include "eepROM.h"


void Initial_EEPROM_Config_Load(void)
{
	// load up last settings from EEPROM
	if(eeprom_read_byte((uint8_t*) EEPROM_DATA_START_POS )!=0x47)
	{
		Config.setup = 0x47;
		Set_EEPROM_Default_Config();
		// write to eeProm
		Save_Config_to_EEPROM_Init();
	} else {
		// read eeprom
		eeprom_read_block(&Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT)); 
	}
}


void Set_EEPROM_Default_Config(void)
{
	Config.RollGyroDirection 	= GYRO_REVERSED;
	Config.PitchGyroDirection	= GYRO_REVERSED;
	Config.YawGyroDirection		= GYRO_REVERSED;

	Config.RxChannel1ZeroOffset	= 1540;
	Config.RxChannel2ZeroOffset	= 1540;
	Config.RxChannel3ZeroOffset	= 1120;
	Config.RxChannel4ZeroOffset	= 1540;
}

void Save_Config_to_EEPROM(void)
{
	// write to eeProm
	cli();
	eeprom_write_block_changes( (const void*) &Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));	//current_config CONFIG_STRUCT
	sei();
}


void Save_Config_to_EEPROM_Init(void)
{
	// write to eeProm
	
	eeprom_write_block_changes( (const void*) &Config, (void*) EEPROM_DATA_START_POS, sizeof(CONFIG_STRUCT));	//current_config CONFIG_STRUCT
	
}

void eeprom_write_block_changes( const uint8_t * src, void * dest, size_t size )
{ 
	size_t len;

	for(len=0;len<size;len++)
	{
		eeprom_write_byte_changed( dest,  *src );

		src++;
		dest++;
	}
}


void eeprom_write_byte_changed( uint8_t * addr, uint8_t value )
{ 
	if(eeprom_read_byte(addr) != value)
	{
		eeprom_write_byte( addr, value );
	}
}
