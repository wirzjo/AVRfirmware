/*
 * serial.c
 *
 * This file contains functions for serial communication using the ATMEL's USART interface
 *
 * Note: For reference use: http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/Der_UART 
 *       or the datasheet starting at page 171
 *
 * Created: 02.04.2015 10:12:25
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 

#include <stdbool.h>
#include <avr/io.h>

#include "config.h"
#include "serial.h"




/************************************************************************/
/* F U N C T I O N    P R O T O T Y P E S                               */
/************************************************************************/

/* @brief Flush the receive buffer (only for error-states) */
void flush(void); 






/************************************************************************/
/* P U B L I C    F U N C T I O N S                                     */
/************************************************************************/

/**
 * Init the use of the serial interface (USART)
 *
 * @return true, if initialization was successful  
 */
bool serial_init(unsigned int baud) {
	
	//Use "Asynchronous double speed mode U2X0 = 1) 
	UCSR0A |= (1<<U2X0); 

	//Calculate the baudrate
	uint8_t ubrr = (double)(F_CPU/16/baud-1); 
	
	//Set baudrate 
	UBRR0H = (unsigned char)(ubrr>>8); 
	UBRR0L = (unsigned char)ubrr; 
	
	//Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); 
	
	//Set frame format: 8data, 2stop bit 
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); 
	
	return true; 
}


/**
 * Send a data byte
 *
 * @param data: byte to be sent 
 */
void serial_send_byte(uint8_t data) {
	
	//Wait for empty transmit buffer 
	while (!(UCSR0A & (1<<UDRE0))); 
	
	//Put data into the buffer and send it 
	UDR0 = data; 
}


/**
 * Receive Data 
 *
 * @return received data byte 
 */
uint8_t serial_receive_byte(void) {
	
	//Wait for data to be reveived 
	while(!(UCSR0A & (1<<RXEN0))); 
	
	//Get and return the received data from the buffer 
	return UDR0; 
}





/************************************************************************/
/* P R I V A T E    F U N C T I O N S                                   */
/************************************************************************/

/** 
 * Flush the receive buffer 
 *
 * Note: This function should only be used if an error occurres, all data in the
 *       receive buffer will be lost! 
 */ 
void flush(void) {
	uint8_t dummy;
	
	//Read the buffer until it is empty  
	while(UCSR0A & (1<<RXEN0)) {
		dummy = UDR0; 
	}
}