/*
 * pixhawk.h
 *
 * Created: 02.04.2015 12:00:55
 *  Author: Jonas
 */ 


#ifndef PIXHAWK_H_
#define PIXHAWK_H_

#include <stdint.h>

/* @brief Init the communication with the pixhawk */ 
bool pixhawk_init(void); 

/* @brief Parse data from the serial rx_buffer */ 
bool pixhawk_parse(uint8_t data);


#endif /* PIXHAWK_H_ */