/*
 * pixhawk.h
 *
 * Created: 02.04.2015 12:00:55
 *  Author: Jonas
 */ 


#ifndef PIXHAWK_H_
#define PIXHAWK_H_

#include <stdint.h>

//Definition of an obstacle-object 
typedef struct obstacle_s {
	float bearing;		//bearing of the obstacle (element of [-180°...0°...+180°]
						//positive bearing <=> starboard side of the boat, 0° directly ahead)
	float distance;		//distance to an obstacle [m]
	uint8_t obst_id;	//unique identifier of the obstacle 
} obstacle; 


/* @brief Init the communication with the pixhawk */ 
bool pixhawk_init(void); 

/* @brief Parse data from the serial rx_buffer */ 
bool pixhawk_parse(uint8_t data);


#endif /* PIXHAWK_H_ */