/*
 * I2C.c
 *
 * This file handels the communication over I2C. It writes and reads bytes from the I2C. 
 * Note: ATMEL calls the I2C bus TWI (Two wire interface). 
 *
 * It is assumed that the processor is the Slave that controlls a Slave 
 *
 * Note: This code is partly based on the code from  Peter Fleury <pfleury@gmx.ch>
 * http://jump.to/fleury
 *
 * Created: 01.04.2015 16:34:08
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 

#include "config.h"
#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include <compat/twi.h>

#include "I2C.h"
#include "port.h"
//#include "serial.h"

#include <avr/delay.h>



/************************************************************************/
/* TWI STATUS CODES                                                     */
/************************************************************************/
// General TWI Master status codes
#define TWI_START					0x08  // START has been transmitted
#define TWI_REP_START				0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST				0x38  // Arbitration lost

// TWI Master Transmitter status codes
#define TWI_MTX_ADR_ACK				0x18  // SLA+W has been tramsmitted and ACK received
#define TWI_MTX_ADR_NACK			0x20  // SLA+W has been tramsmitted and NACK received
#define TWI_MTX_DATA_ACK			0x28  // Data byte has been tramsmitted and ACK received
#define TWI_MTX_DATA_NACK			0x30  // Data byte has been tramsmitted and NACK received

// TWI Master Receiver status codes
#define TWI_MRX_ADR_ACK				0x40  // SLA+R has been tramsmitted and ACK received
#define TWI_MRX_ADR_NACK			0x48  // SLA+R has been tramsmitted and NACK received
#define TWI_MRX_DATA_ACK			0x50  // Data byte has been received and ACK tramsmitted
#define TWI_MRX_DATA_NACK			0x58  // Data byte has been received and NACK tramsmitted

// TWI Slave Transmitter status codes
#define TWI_STX_ADR_ACK				0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST	0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK			0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK			0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE	0xC8  // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received

// TWI Slave Receiver status codes
#define TWI_SRX_ADR_ACK				0x60  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST	0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK				0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST	0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK		0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK		0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK		0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK		0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART		0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
#define TWI_NO_STATE				0xF8  // No relevant state information available; TWINT = “0”
#define TWI_BUS_ERROR				0x00  // Bus error due to an illegal START or STOP condition



#define BITRATE 100000L		//100kHz maximum Bitrate
#define TIMEOUT 100         //Number or times I2C retries to get a connection before a timeout occures => prevents the I2C from beeing blocking 
static uint8_t timeout = 0; //Number of times I2C actually tried to get a connection 

/**
 * Init the use of I2C 
 *
 * @param bitrate [Hz]
 * @return false, if Bitrate is too high 
 */ 
bool I2C_init(uint32_t bitrate) {
	
	//Init the I2C bitrate and prescaler   
	TWSR = 0;                       //no prescaler => presaclaer = 1
	TWBR = ((F_CPU/BITRATE)-16)/2;  //should be >10 for stable operation 
	
	//Check if TWBR is high enough (>10)
	if(TWBR>10) {
		return true; 
	}		
	
	//Some error occured => return false 	
	return false;
}



/** 
 * Start the I2C Master 
 *
 * @param address: 7bit slave address 
 * @param access: read or write (1 = read, 0 = write) 
 */
bool I2C_start(uint8_t address, uint8_t access) {
	
	timeout = 0; //Reset the number of retries 
	
	uint8_t   twst;
	
	//serial_send_string(" I2C Start...\n"); 

	while(true) {

		// send START condition
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

		// wait until transmission completed
		while(!(TWCR & (1<<TWINT))) {
			timeout++; 
			
			if(timeout>TIMEOUT) {
				return false; 
			}
		}
		
		//serial_send_string("   transmission completed"); 

		// check value of TWI Status Register. Mask prescaler bits.
		twst = TW_STATUS & 0xF8;
		if ( (twst != TW_START) && (twst != TW_REP_START)) return false;

		//send device address
		//address = address | access;
		TWDR = ((address<<1) | access);
		TWCR = (1<<TWINT) | (1<<TWEN);

		// wail until transmission completed and ACK/NACK has been received
		while(!(TWCR & (1<<TWINT))){
			timeout++;
			
			if(timeout>TIMEOUT) {
				return false;
			}
		}
		
		//serial_send_string("   ACK/NACK received"); 

		// check value of TWI Status Register. Mask prescaler bits.
		twst = TW_STATUS & 0xF8;	
	
		if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) {
			//We received NACK => the device is busy => we continue polling it 
			//serial_send_string("   LIDAR busy!");
			
			//device busy, send stop condition to terminate write operation
			TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
		
			// wait until stop condition is executed and bus released
			while(TWCR & (1<<TWSTO)) {
				timeout++;
				
				if(timeout>TIMEOUT) {
					return false;
				}
			}				
		
			continue;
		}
		//serial_send_string(" ...start OK! ");

		break;
	}

	//serial_send_string("...end!\n"); 
	return true;
}



/**
 * Stop the I2C Master 
 *
 */
void I2C_stop(void) {
	
	timeout = 0; //Reset the number of retries 
	
    //Send Stop condition 
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    
    // wait until stop condition is executed and bus released
    while(TWCR & (1<<TWSTO)) {
		timeout++;
		
		if(timeout>TIMEOUT) {
			return false;
		}
	}
}	



/**
 * Write a Byte to the slave 
 *
 * @param bytes 
 */
bool I2C_write_byte(uint8_t byte) {
	
	timeout = 0; //Reset the number of retries 
	
	uint8_t   twst;
	
	//Send data to the previously addressed device
	TWDR = byte;
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	//serial_send_string("  write byte...");
	
	//Wait until transmission completed
	while (!(TWCR & (1<<TWINT))) {
		timeout++;
		
		if(timeout>TIMEOUT) {
			return false;
		}
	}
	
	//serial_send_string("  byte written!"); 

	//Check value of TWI Status Register. Mask prescalor bits
	twst = TWSR & 0xF8;
	if (twst != TWI_MTX_DATA_ACK) {
		return true;
	}	
		
	return false;
}


/**
 * Read Bytes from the Bus
 *
 * @return byte read
 */
uint8_t I2C_read_byte(void) {
	
	timeout = 0; //Reset the number of retries 
	
	//serial_send_string("  read byte..."); 
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while (!(TWCR & (1<<TWINT))) {
		timeout++;
		
		if(timeout>TIMEOUT) {
			return false;
		}
	}

	//serial_send_string("  byte read"); 

	return TWDR;
}


/**
 * Read the last byte from the slave 
 * 
 * @return last byte read
 */
uint8_t I2C_read_last_byte(void) {
	
	timeout = 0; //Reset the number of retries 
	
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT))) {
		timeout++;
		
		if(timeout>TIMEOUT) {
			return false;
		}
	}
	
	return TWDR;
}
