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

#include "config.h"
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#include "serial.h"
#include "pixhawk.h"


/************************************************************************/
/* V A R I A B L E S                                                    */
/************************************************************************/





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
	//UCSR0A |= (1<<U2X0); double speed mode 
	UCSR0A &= ~(1<<U2X0); //normal speed mode 
	

	//Calculate the baudrate
	unsigned int ubrr = (double)(F_CPU/16/baud-1);  //Note change 16 to 8 for double speed mode 
	
	//Set baudrate 
	UBRR0H = (unsigned char)(ubrr>>8); 
	UBRR0L = (unsigned char)ubrr; 
	
	//Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); 
	
	//Set frame format: 8data, 2stop bit 
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); 
	
	//Allow receive interrupts 
	UCSR0B |= (1<<RXCIE0); 
	
	
	//Wait a moment to Set registers 
	_delay_ms(1000); 
	
	//Everything is OK => return true
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

void serial_send_string(char buf[]) {
	
	#if DEBUG_SERIAL == 1
	//We only send strings to command line, if the DEBUG-Flag is set 
	
	uint8_t ind; 
	ind = 0; 
	
	while(buf[ind] != 0x00) {
		serial_send_byte(buf[ind]); 
		ind=ind+1;
	}
	  
	serial_send_byte('\n'); 
	serial_send_byte(0x0D);	//Send new line 
	
	#endif
	
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
/* I N T E R R U P T    H A N D L E R S                                 */
/************************************************************************/

/** 
 * Interrupt for complete reception. 
 * This interrupt occurs, as soon as a character was successfully read. 
 */
ISR(USART_RX_vect) {
	
	//Store data locally 
	uint8_t data = UDR0; 
	
	//Notify the Pixhawk-Module that new data is available 
	pixhawk_parse(data);
	
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