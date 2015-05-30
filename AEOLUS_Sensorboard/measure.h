/*
 * measure.h
 *
 * Created: 10.04.2015 16:22:32
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


#ifndef MEASURE_H_
#define MEASURE_H_

/* @brief Do a measurement by controlling the Sensor and do Distance measurements using the LIDAR */ 
void measure_new(void); 

/* @brief Handle repetitive tasks for measurement */ 
void measure_handler(void); 

/* @brief Init the measurement */ 
bool measure_init(void);

/* @brief Return the identified obstacles from the buffer */ 
bool measure_get_obstacles(uint16_t *angle, uint16_t *dist);

/* @brief Return the distance at a given Angle */ 
uint16_t measure_get_distance(uint16_t angle); 

/* @brief Return the distance at a given Angle from the small Matrix*/ 
uint16_t measure_get_distance_small(uint16_t ind);

/* @brief Return the heading for which the small distance Matrix is valid */ 
uint16_t measure_get_heading_valid(void);

/* @brief Set the threshold for the obstacle Detction */ 
bool measure_set_threshold(uint16_t threshold); 



#endif /* MEASURE_H_ */