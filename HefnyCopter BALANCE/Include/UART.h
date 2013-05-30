/*
 * UART.h
 *
 * Created: 19-Sep-12 11:01:53 AM
 *  Author: M.Hefny
 */ 


#ifndef UART_H_
#define UART_H_

/*
Protocol:
#CMD#: 1 byte
		'V'		: SETVALUE:   0 byte
					DataOffset: 2 bytes		[0-65536]
					DataLength: 1 byte		[0,1,2,3,4]
					DataValue:  4 bytes		[Word-32]
					CHKSUM:		1 byte
		'C'		: SETCMD      0 byte
					CMD:		1 byte
								0	--		DISARM	
								1   --		BLINK
								2	--		X-MODE
								3	--		P-MODE
								4	--		CALIBRATE_ACC
								5	--		
								6	--	
								7	--	
								8	--	
								9	--	
								10	--	
								
					DataValue:	6
					CHKSUM:		1 byte
*/

//#define SERIAL_OFFSET_ThrottleMin	
#define SERIAL_CMD_LED_BLINK		0x01
#define SERIAL_CMD_PID_CONFIG		0x02
#define SERIAL_CMD_READ_CONFIG		0x05
#define SERIAL_CMD_SAVE_CONFIG		0x06
#define SERIAL_CMD_CALIBRATE_ACC	0x07

#define SERIAL_HEADER_SETVALUE	'V'
#define	SERIAL_HEADER_DOCMD		'C'
#define SERIAL_HEADER			0
#define SERIAL_CMD_ID			1
#define SERIAL_DATA_OFFSET		1
#define SERIAL_DATA_LENGHT		3
#define SERIAL_DATA_VALUE		4
#define SERIAL_BUFFERSIZE_1		8
#define SERIAL_BUFFERSIZE		9
#define SERIAL_CRC_OFFSET		8

volatile uint8_t  RXBuffer[SERIAL_BUFFERSIZE];
volatile uint8_t  DisplayBuffer [10]; 
void UART_Init( unsigned int ubrr);

void send_byte(uint8_t u8Data);


void Send_Data_TEST (void * msg, uint8_t len);
void Send_Data (void * msg, uint8_t len);

//volatile uint8_t UART_Buffer_RTS;
//volatile char UART_Buffer_TX[256];
uint8_t *DataPtr;
uint8_t DataCounter;
#endif /* UART_H_ */