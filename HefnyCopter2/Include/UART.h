/*
 * UART.h
 *
 * Created: 19-Sep-12 11:01:53 AM
 *  Author: M.Hefny
 */ 


#ifndef UART_H_
#define UART_H_


void UART_Init( unsigned int ubrr);

void send_byte(uint8_t u8Data);


void Send_Data (void * msg, uint8_t len);

//volatile uint8_t UART_Buffer_RTS;
//volatile char UART_Buffer_TX[256];
uint8_t *DataPtr;
uint8_t DataCounter;
#endif /* UART_H_ */