/*
 * eepROM.h
 *
 * Created: 23-Jul-12 3:38:36 PM
 *  Author: M.Hefny
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_


#define EEPROM_DATA_START_POS 0		// allows moving (not really necessary)


void Initial_EEPROM_Config_Load(void);

void Load_Config_from_EEPROM(void);
void Save_Config_to_EEPROM(void);
void Save_Default_Config_to_EEPROM (void);
void Set_EEPROM_Default_Config(void);
void eeprom_write_byte_changed(uint8_t * addr, uint8_t value);
void eeprom_write_block_changes(const uint8_t * src, void * dest, uint16_t size);



#endif /* EEPROM_H_ */