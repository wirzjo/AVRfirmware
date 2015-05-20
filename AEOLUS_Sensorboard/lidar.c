/*
 * Lidar.c
 *
 * This file handles the communication with the LIDAR sensor by using I2C in Master-Mode. 
 *
 * Note: The Sensor specifications are as follows: 
 *	-Bitrate: 100kHz
 *  -7bit Slave address: 0x62
 *  -8bit Address for Write: 0xC4 => access-bit is 0
 *  -8bit Address for Read: 0xC5 => access-bit is 1
 *  -8 data bits + 1 ACK bit
 *  -A write operation is used both for read and write transfers 
 *  -If bit 7 of the register address to be read is 1, two consecutive registers are read 
 * 
 * Note: An example of the I2C protocol can be found on: 
 * https://github.com/kent-williams/LIDAR-Lite-DSS-I2C-Library-State-Machine/blob/master/LIDAR-Lite-DSS-I2C-Library-State-Machine/LIDAR-Lite-DSS-I2C-Library-State-Machine.ino
 *
 * The general Protocoll structure is as follows: 
 * START | 7bit Slave address | 1bit access (0 = write/1 = read) | Register |
 *
 * Created: 01.04.2015 17:21:30
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


#include "I2C.h"
#include "config.h"
#include <avr/delay.h>
//#include "serial.h"

/************************************************************************/
/* V A R I A B L E S                                                    */
/************************************************************************/

#define BITRATE 100000L		//100kHz maximum Bitrate 
#define SLAVE_ADDR 0x62		//Slave address 

#define WRITE 0			//Write access to the register 
#define READ 1			//Read access to the register 


static struct {
	uint16_t last_distance;		//Last measured distance 
} state = {
	.last_distance = 0
};



/************************************************************************/
/* C O M M A N D S                                                      */
/************************************************************************/

//INTERNAL REGISTERS (read and write) 
#define I_COMMAND_REG 0x00		//Command control register => write commands to these register
#define I_STATUS	  0x01		//Returns the status 
#define I_DIST        0x0f		//Returns the measured distance [cm] (Note this is a 16bit value => read two registers!)


//EXTERNAL REGISTERS (read or write only) 
#define e_range_crit 0x4b		//Range processing criteria for two echos. Max Signal or Max/Min Range 



/************************************************************************/
/* F U N C T I O N    P R O T O T Y P E S                               */
/************************************************************************/

/* @brief Write data to a register using I2C */ 
bool write_register(uint8_t reg, uint8_t data); 

/* @brief Read data from a register using I2C */
bool read_register(uint8_t reg, uint8_t numofbytes, uint8_t arraytosafe[2]); 









/************************************************************************/
/* P U B L I C     F U N C T I O N S                                    */
/************************************************************************/

/**
 * Init the LIDAR Sensor 
 *
 * @return true, iff initialization was successful 
 */
bool lidar_init(void) {
	
	bool status = true;		//Status of the initialization (we assume that everything is OK at the beginning) 
	
	//Start the I2C Interface 
	if(!I2C_init(BITRATE)) {
		//An error occurred during initialization of the I2C interface 
		//Nothing we can do about this... might flag unhappy... 
		
		return false; 
	}
	
	
	//Reset the lidar to defaults for Distance Measurements 
	status = status && write_register(I_COMMAND_REG, 0x00); 
	
	//Return the status after all initialization is done
	return status; 
}



/**
 * Read the distance from the LIDAR Sensor 
 *
 * @return the measured distance [cm] (Note: 16bit value!) 
 */ 
uint16_t lidar_measure(void) {
	uint8_t result[2]; 
	
	if(!write_register(0x00,0x04)) {
		//serial_send_string("error measure"); 
		return 0; 
	}
	
	//This delay is needed accoring to the LIDAR I2C Protocol. The Module will respond with a NACK, if a read or 
	//write request is sent using I2C. 
	//Maybe it is possible to do something else in this time...
	//But be careful. Moving the Servo in this time can lead to a wrong measurement! 
	_delay_ms(20); //Note: This delay is very important! (as soon as it is removed, the software crashes at some point!) 
	
	//Read the Distance from the Register using I2C
	if(!read_register(0x0f,2,result)) {
		//The reading of the registers was NOT successful 
		//Nothing we can do about this... might flag unhappy...
		
		return 0; 
	}

	
	//Since we read a new distance from the Sensor, we can store it as the local state 
	state.last_distance = ((result[0] << 8) | result[1]);
 
 
	//Check the result: s
	//The LIDAR returns zero, if the measurement was NOT successful. This means that no object is detected inside the measurement range of the LIDAR.
	//In such a case set the measured distance to the maximum range in order to not confuse the filtering process. 
	if(state.last_distance == 0) {
		//The LIDAR did not detect anything inside the measurement range => we assign the maximum range 
		
		state.last_distance = LIDAR_MAX_DISTANCE; 
	}
	
	
	//serial_send_string("LIDAR OK!"); 
	//Return the current distance 
	return state.last_distance;  
	
}



/**
 * Get the last known distance from the sensor 
 *
 * @return The latest known distance 
 */
uint16_t lidar_get_distance(void) {
	
	return state.last_distance; 
}





/************************************************************************/
/* P R I V A T E     F U N C T I O N S                                  */
/************************************************************************/

/** 
 * Write a value to a register 
 *
 * @param reg: Name of the register 
 * @param data: Data to be written to the register  
 */ 
bool write_register(uint8_t reg, uint8_t data) {
	
	//serial_send_string("WRITE..."); 
	
	//Start the I2C Master interface. 
	//We want to write a register => access-type is WRITE 
	if(!I2C_start (SLAVE_ADDR, WRITE)) {
		//I2C could not be started => nothing we can do against this, might flag unhappy...
		//Anyway, stop the I2C Master interface
		
		//serial_send_string("can not write"); 
		I2C_stop();
		
		return false; 
	} else {
		//I2C Master Interface is started => we can transfer the bytes to the slave 
		
		//Send Register address 
		I2C_write_byte(reg); 
		
		//Send Value the register should contain 
		I2C_write_byte(data);
		
		//Close the Master-Interface 
		I2C_stop(); 
	}
	
	//serial_send_string("Write OK");
	//_delay_ms(3);  
	
	//Everything is OK => return true
	return true; 
}


/**
 * Read a value from a register 
 *
 * @param reg: Name of the register 
 * @param numofbytes: number of Bytes to be read (1,2) 
 * @param arraytosafe: Array with two bytes, where the result is stored 
 */
bool read_register(uint8_t reg, uint8_t numofbytes, uint8_t arraytosafe[2]) {
	
	//serial_send_string("READ..."); 
	
	//Start the I2C Master interface
	//We want tor write a register first => access-type is WRITE 
	if(!I2C_start(SLAVE_ADDR, WRITE)) {
		//I2C could not be started => nothing we can do against this, might flag unhappy...
		//Anyway, stop the I2C Master interface
				
		I2C_stop();
		
		//serial_send_string("I2C Error");
		
		return false; 
	} else {
		//I2C Master Interface is started => we can transfer the bytes to the slave
		//serial_send_string("send Bytes"); 
		
		//If two consecutive registers should be read, the address must contain a 1 as bit7
		if(numofbytes == 2) {
			//We want to write two consecutive registers => we must set bit7 of the register address to 1
			reg = 0x80 | reg; 
		}
		
		//Send the register address to be read
		I2C_write_byte(reg); 
		
		//Stop the Interface 
		I2C_stop();
		
		//Start the I2C Master interface
		//This time we want tor read a register => access-type is READ
		if(!I2C_start(SLAVE_ADDR, READ)) {
			//I2C could not be started => nothing we can do against this, might flag unhappy... 
			//Anyway, stop the I2C Master interface 
			
			I2C_stop(); 
			
			//serial_send_string("Error: no read");
			
			return false; 
		} else {
			//I2C Master Interface is started => we can read the bytes from the slave 
			
			//serial_send_string(" try to read"); 
			
			//Read one or two bytes from the Slave 
			if(numofbytes == 1) {
				//Only one byte is to be read 
				
				arraytosafe[0] = I2C_read_byte(); //The first byte is the last byte
			} else if(numofbytes == 2) {
				//Two bytes are to be read 
				
				arraytosafe[0] = I2C_read_byte();		//Read first byte 
				//serial_send_string(" read first byte"); 
				arraytosafe[1] = I2C_read_byte();		//Read second byte <=> last byte 
				//serial_send_string(" read second byte");  
				
				//Stop the Master Interface for Reading 
				//I2C_stop(); 
			} else {
				I2C_stop(); 
				
				return false; 
			}
			
			//Stop the Master-interface for Writing 
			I2C_stop(); 
		} //END OF I2C_READ
		
	} //END OF I2C_WRITE
	
	//serial_send_string("EOF read"); 
	
	//Everything is OK => return true
	return true; 
}