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

/************************************************************************/
/* V A R I A B L E S                                                    */
/************************************************************************/

#define BITRATE 100000		//100kHz maximum Bitrate 
#define SLAVE_ADDR 0x62		//Slave address 

#define WRITE 0				//Write access to the register 
#define READ 1				//Read access to the register 



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
uint16_t lidar_get_distance(void) {
	uint8_t result[2]; 
	
	//Read the Distance from the Register using I2C
	if(!read_register(0x0f,2,result)) {
		//The reading of the registers was NOT successful 
		//Nothing we can do about this... might flag unhappy...
		
		return 0; 
	}
	
	//We read two 8bit values => convert to a 16bit value
	return ((result[0] << 8) | result[1]);  
	
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
	
	//Start the I2C Master interface. 
	//We want to write a register => access-type is WRITE 
	if(!I2C_start (SLAVE_ADDR, WRITE)) {
		//I2C could not be started => nothing we can do against this, might flag unhappy...
		//Anyway, stop the I2C Master interface
		
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
	
	//Start the I2C Master interface
	//We want tor write a register first => access-type is WRITE 
	if(!I2C_start(SLAVE_ADDR, WRITE)) {
		//I2C could not be started => nothing we can do against this, might flag unhappy...
		//Anyway, stop the I2C Master interface
				
		I2C_stop();
		
		return false; 
	} else {
		//I2C Master Interface is started => we can transfer the bytes to the slave
		
		//If two consecutive registers should be read, the address must contain a 1 as bit7
		if(numofbytes == 2) {
			//We want to write two consecutive registers => we must set bit7 of the register address to 1
			reg = 0x80 | reg; 
		}
		
		//Send the register address to be read
		I2C_write_byte(reg); 
		
		//Start the I2C Master interface
		//This time we want tor read a register => access-type is READ
		if(!I2C_start(SLAVE_ADDR, READ)) {
			//I2C could not be started => nothing we can do against this, might flag unhappy... 
			//Anyway, stop the I2C Master interface 
			
			I2C_stop(); 
			
			return false; 
		} else {
			//I2C Master Interface is started => we can read the bytes from the slave 
			
			//Read one or two bytes from the Slave 
			if(numofbytes == 1) {
				//Only one byte is to be read 
				
				arraytosafe[0] = I2C_read_last_byte(); //The first byte is the last byte
			} else if(numofbytes == 2) {
				//Two bytes are to be read 
				
				arraytosafe[0] = I2C_read_byte();		//Read first byte 
				arraytosafe[1] = I2C_read_last_byte();	//Read second byte <=> last byte  
			} else {
				I2C_stop(); 
				
				return false; 
			}
			
			//Stop the Master-interface
			I2C_stop(); 
		}
	}
	
	//Everything is OK => return true
	return true; 
}