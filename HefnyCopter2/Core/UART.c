/*
 * UART.c
 *
 * Created: 19-Sep-12 11:01:35 AM
 *  Author: M.Hefny
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <avr/wdt.h>
#include <util/atomic.h>


#include "../Include/GlobalValues.h"
#include "../Include/IO_config.h"
#include "../Include/UART.h"
#include "../Include/eepROM.h"
#include "../Include/Arming.h"
#include "../Include/Sensors.h"
#include "../Include/LED.h"

volatile uint8_t  RXIndex=0;
volatile uint16_t LastRXTime;
volatile uint8_t RXChar;
static enum _serial_state {
    IDLE,
    HEADER_RX,
   
  } c_state = IDLE;
  
  
void ParseCommand();

ISR (USART1_RX_vect)
{
	 while ( !(UCSR1A & (1<<RXC1))); 
	 RXChar = UDR1;
	 
	  if ((c_state!=IDLE) && (TCNT1_X - LastRXTime) > 20) 
	   {
		   // timeout ..
		   c_state = IDLE;
		   RXIndex =0;
		   DisplayBuffer[4]=0;
		   //memcpy(DisplayBuffer,"OUT",3);
		   //return ;
	   }
	   RXBuffer[RXIndex]=RXChar;
	   switch (c_state)
	   {
		   case IDLE:
		   // Waiting for new command
			   RXIndex=0;
			   
			   
			   c_state = HEADER_RX;
			   ATOMIC_BLOCK(ATOMIC_FORCEON)
			   {
				 LastRXTime = TCNT1_X;
			   }
			  
		   break;
		   
		   case HEADER_RX:
		   // Copy Data
				
			    if (RXIndex==SERIAL_BUFFERSIZE_1)
				{
					ParseCommand();
				}
				else
				{
					ATOMIC_BLOCK(ATOMIC_FORCEON)
				   {
					 LastRXTime = TCNT1_X;
				   }
				}								
		   break;
	   }
	
	 RXIndex+=1;
	 if (RXIndex==SERIAL_BUFFERSIZE) 
	 {
		RXIndex=0;
		c_state = IDLE;
	 }		
}



void ParseCommand ()
{
	uint16_t Length;
	
	if (IsArmed==true)
	{
		//Just ignore for safty reasons....some commands generate beeps or flash led that call delay function which will crash your quadcopter if called.
		return ;
	}		
	int8_t CRC=0;
	for (int i=0;i < SERIAL_BUFFERSIZE_1; ++i)
	{
		CRC +=RXBuffer[i];
	}
	if (CRC == (int8_t)RXBuffer[SERIAL_CRC_OFFSET])
	{
		switch (RXBuffer[SERIAL_HEADER])
		{
			case SERIAL_HEADER_DOCMD:
				switch (RXBuffer[SERIAL_CMD_ID])
				{
					case SERIAL_CMD_LED_BLINK: // never  call while ARMED .... calling delay function will affect motor speed.
						LED_FlashOrangeLED (LED_SHORT_TOGGLE,4);
					break;
					case SERIAL_CMD_PID_CONFIG:
						Length = sizeof(pid_param_t) * 7;
						Send_Byte('C');
						Send_Byte (SERIAL_CMD_PID_CONFIG);
						Send_Data(&Length,2);
						Send_Data(&(Config.GyroParams),Length);
						Send_Byte('E');
					break;
					case SERIAL_CMD_SAVE_CONFIG:
						Save_Config_to_EEPROM();
					break;
					case SERIAL_CMD_CALIBRATE_ACC:
						Sensors_Calibrate();								
					break;
				}
			break;
			case SERIAL_HEADER_SETVALUE:
				//LED_FlashOrangeLED (LED_SHORT_TOGGLE,4);
				memcpy ((void *)&Config + RXBuffer[SERIAL_DATA_OFFSET] + (RXBuffer[SERIAL_DATA_OFFSET+1] * 0xff),(void *) &RXBuffer[SERIAL_DATA_VALUE], (int8_t) RXBuffer[SERIAL_DATA_LENGHT]);
			break;
		default:
			// COMMAND ERROR
			//memcpy(DisplayBuffer,"ERR",3);
			break;
		}
	}
	else
	{
		// UART ERROR
		//DisplayBuffer[1]+='0';
		//memcpy(DisplayBuffer,"CRC",3);
	}		
	
	
}

void UART_Init( unsigned int ubrr)
{
	if (Config.RX_mode==RX_mode_UARTMode)
	{
		//memcpy(DisplayBuffer,"STR",3);
		RXIndex=0;
		/*Set baud rate */
		UBRR1H = (unsigned char)(ubrr>>8);	
		UBRR1L = (unsigned char)ubrr;
		
		//UCSR1A = (1 << U2X1); // doubling divisor rate;
		// Set frame format to 8 data bits, no parity, 1 stop bit
		UCSR1C = (0<<USBS1)|(1<<UCSZ11)|(1<<UCSZ10);
		// Enable receiver and transmitter
		UCSR1B =  (1<<TXEN1)	// Enable USART TX
				//| (1<<TXCIE1)   // Enable TX interrupt
				  | (1<<RXEN1)
				  | (1<< RXCIE1)
				;
	}
}

void Send_Byte(uint8_t u8Data)
{

	// Wait if a byte is being transmitted
	while((UCSR1A & (1<<UDRE1)) == 0);
	// Transmit data
	/////////delay_ms(10);
	UDR1 = u8Data;
}


void Send_Data (void * msg, uint8_t len)
{
	
	for (int i=0; i<len;++i)
	{
		Send_Byte (((uint8_t *)msg)[i]);
	}
	
	
}

