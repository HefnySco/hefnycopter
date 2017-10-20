//***********************************************************
//* i2c.c
//***********************************************************

//***********************************************************
//* Includes
//***********************************************************

#include <avr/io.h>
#include "../include/IO_Config.h"
#include "../include/i2cmaster.h"


//************************************************************
// Prototypes
//************************************************************

void writeI2Cbyte(uint8_t address, uint8_t location, uint8_t value);
void readI2CbyteArray(uint8_t address, uint8_t location, uint8_t *array,uint8_t size);

#ifdef KK21
//************************************************************
// KK2.1 gyro-specifc code
//************************************************************

void writeI2Cbyte(uint8_t address, uint8_t location, uint8_t value)
{
    i2c_start_wait((address << 1 ));				// Set up device address 
    i2c_write(location);							// Set up register address 
    i2c_write(value); 								// Write byte
    i2c_stop();
}

void readI2CbyteArray(uint8_t address, uint8_t location, uint8_t *array,uint8_t size)
{
    i2c_start_wait((address << 1 ));
    i2c_write(location);							// Set up register address 
    i2c_rep_start((address << 1 ) | 0x01);
	int i=0;
	while(i < size)
	{
		if ((i+1)!=size)
			array[i]=i2c_readAck();
		else array[i]=i2c_readNak(); 				// Read without ACK on last byte
		i++;
	}
    i2c_stop();
}

#endif
