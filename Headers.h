/*
 * Headers.h
 *
 * Created: 20-Jul-12 5:55:10 PM
 *  Author: M.Hefny
 */ 


#ifndef HEADERS_H_
#define HEADERS_H_



void setup(void);
void loop(void);

void Initial_EEPROM_Config_Load(void);
void Save_Config_to_EEPROM(void);
void Set_EEPROM_Default_Config(void);

void eeprom_write_byte_changed( uint8_t *  addr, uint8_t value );
void eeprom_write_block_changes( const uint8_t * src, void * dest, size_t size );

void RxGetChannels(void);
void Init_ADC(void);
void delay_us(uint8_t time);
void delay_ms(uint16_t time);



#endif /* HEADERS_H_ */