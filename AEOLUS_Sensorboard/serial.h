/*
 * serial.h
 *
 * Created: 02.04.2015 10:12:37
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


#ifndef SERIAL_H_
#define SERIAL_H_

/* @brief Initialize the use of the serial communication */ 
bool serial_init(unsigned int baud); 

/* @brief Send a byte using the serial interface */ 
void serial_send_byte(uint8_t data);

#endif /* SERIAL_H_ */