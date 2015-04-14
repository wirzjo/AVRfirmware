/*
 * Lidar.h
 *
 * Created: 01.04.2015 17:21:42
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


#ifndef LIDAR_H_
#define LIDAR_H_

#include <stdbool.h>
#include <stdint.h>

/* @brief Init the use of the lidar-sensor */ 
bool lidar_init(void);

/* @brief Do a new measurement with the LIDAR sensor */ 
uint16_t lidar_measure(void); 

/* @brief Get the latest known distance measurement from the lidar-sensor */ 
uint16_t lidar_get_distance(void);


#endif /* LIDAR_H_ */