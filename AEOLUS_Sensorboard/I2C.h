/*
 * I2C.h
 *
 * Created: 01.04.2015 16:34:49
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch> 
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <stdbool.h>
#include <stdint.h>

/* @brief Init the use of I2C */ 
bool I2C_init(uint32_t bitrate); 

/* @brief Start the I2C Interface as a Master */ 
bool I2C_start(uint8_t address, uint8_t access); 

/* @brief Stop the I2C Interface as a Master */ 
void I2C_stop(void); 

/* @brief Write a Byte to the Slave */
bool I2C_write_byte(uint8_t byte);

/* @brief Read a Byte from the Slave */
uint8_t I2C_read_byte(void);

/* @brief Read the last byte from the Slave */
uint8_t I2C_read_last_byte(void); 


#endif /* I2C_H_ */