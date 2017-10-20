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
#include <util/atomic.h>
#include <avr/interrupt.h> 
#include <avr/pgmspace.h>


#include "../Include/typedefs.h"
#include "../Include/IO_config.h"
#include "../Include/GlobalValues.h"
#include "../Include/eepROM.h"
#include "../Include/Beeper.h"
#include "../Include/MPU6050.h"
#include "../Include/version.h"

 /*
 PI
 PG:50
 PLimit:100
 IGain:25
 ILimit:50
 
 YAW:
 PG:50
 PLimit:20
 IGain:50
 ILimit:10
 
 
 Stick Scaling:
 R:30
 P:30
 Y:50
 Thr:90
 
 */

static config_t const defaultConfig PROGMEM = 
{
	.signature					= HEFNYCOPTER2_SIGNATURE,
	.Version					= VERSION_16INT,
	.IsCalibrated				= 0,
	.FrameType					= FRAMETYPE_QUADCOPTER,
	.RX_mode					= RX_mode_SingleMode,
	.BoardOrientationMode		= QuadFlyingMode_PLUS,
	.QuadFlyingMode				= QuadFlyingMode_PLUS,
	.GyroParams[PITCH_INDEX]	= {35,200,10,150,-4,40,500},		// pitch roll gyro.
	.GyroParams[ROLL_INDEX]		= {35,200,10,150,-4,40,500},		// pitch roll gyro.
	.GyroParams[YAW_INDEX]		= {50,100,50,100,0,20,200},		// Yaw gyro
	.AccParams[PITCH_INDEX]		= {40,200,5,170,0,20,990},		// ACC_PITCH & Y
	.AccParams[ROLL_INDEX]		= {40,200,5,170,0,20,990},		// ACC_PITCH & Y
	.AccParams[YAW_INDEX]		= {15,35,0,30,0,20,200},		// ACC_Z
	.SonarParams[0]				= {15,25,0,10,0,10,0},
	.ReverseYAW					= GYRO_NORMAL,
	.RCExpo						= 50,
	.RCLimit					= 90,
	.AutoDisarm					= 3,  // auto disarm
	.MiscSensors				= NO_MISC_SENSORS,
	.ThrottleMin				= 150,
	.VoltageAlarm				= 0, // off		[40 means 4 volt]
	.Acc_Pitch_Trim				= 0,
	.Acc_Roll_Trim				= 0,
	.PitchRollLinked			= 0x01,
	.MPU6050_LPF				= MPU60X0_DLPF_BW_256,
	.MPU6050_Gyro_Range			= MPU60X0_GYRO_FS_2000,
	.MPU6050_Acc_Range			= MPU60X0_ACCFS4G,
};


void Initial_EEPROM_Config_Load(void)
{
	
	eeprom_read_block(&Config, (void*) EEPROM_DATA_START_POS, sizeof(config_t)); 
	
	// load up last settings from EEPROM
	if(
		(Config.signature != HEFNYCOPTER2_SIGNATURE)
		#ifdef REQUIRE_FACTORY_RESET
		||		
		(Config.Version  != VERSION_16INT)
		#endif
	  )		
	{
		Save_Default_Config_to_EEPROM();
		
	} else {
		// read eeprom
		eeprom_read_block(&Config, (void*) EEPROM_DATA_START_POS, sizeof(config_t)); 
	}
}

void Save_Default_Config_to_EEPROM (void)
{
	
	// copy default config to config.
	Set_EEPROM_Default_Config();

	// Init values.
	for (uint8_t i = 0; i < RXChannels; i++)
	{
		Config.RX_Mid[0][i] = PWM_MID;
		Config.RX_Min[0][i] = PWM_LOW;
		Config.RX_Mid[1][i] = PWM_MID;
		Config.RX_Min[1][i] = PWM_LOW;
	}
	
	
	// write to eeProm
	Save_Config_to_EEPROM();
}

void Set_EEPROM_Default_Config(void)
{
	memcpy_P ( &Config, &defaultConfig, sizeof(config_t));
}

void Save_Config_to_EEPROM(void)
{
	// write to eeProm
	cli();
	eeprom_write_block_changes( (const void*) &Config, (void*) EEPROM_DATA_START_POS, sizeof(config_t));	//current_config CONFIG_STRUCT
	sei();
	
	Beeper_Beep(BEEP_LONG,1);	
}


void Load_Config_from_EEPROM(void)
{
	// write to eeProm
	
	eeprom_write_block_changes( (const void*) &Config, (void*) EEPROM_DATA_START_POS, sizeof(config_t));	//current_config CONFIG_STRUCT
	
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
