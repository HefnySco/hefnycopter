/*
 * UART.c
 *
 * Created: 19-Sep-12 11:01:35 AM
 *  Author: M.Hefny
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <avr/wdt.h>
#include <util/atomic.h>


#include "../Include/GlobalValues.h"
#include "../Include/IO_config.h"
#include "../Include/UART.h"

//ISR (USART1_TX_vect)
//{
	//UART_Buffer_RTS=0;
//}
//



void UART_Init( unsigned int ubrr)
{
	if (Config.RX_mode==RX_mode_UARTMode)
	{
		/*Set baud rate */
		UBRR1H = (unsigned char)(ubrr>>8);	
		UBRR1L = (unsigned char)ubrr;
		
		//UCSR1A = (1 << U2X1); // doubling divisor rate;
		// Set frame format to 8 data bits, no parity, 1 stop bit
		UCSR1C = (0<<USBS1)|(1<<UCSZ11)|(1<<UCSZ10);
		// Enable receiver and transmitter
		UCSR1B =  (1<<TXEN1)	// Enable USART TX
				//| (1<<TXCIE1)   // Enable TX interrupt
				;
	}
}

void send_byte(uint8_t u8Data)
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
		send_byte (((uint8_t *)msg)[i]);
	}
	
}