/*
 * config.h
 *
 * This file contains important definitions of variables valid for the whole program 
 *
 * Created: 01.04.2015 17:56:06
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


#ifndef CONFIG_H_
#define CONFIG_H_

#include "buffer.h"

/** INTERVAL [°]
 * Angle between two distance measurements in degrees (smallest interval possible is 1, integer types only) */ 
#define INTERVAL 2 

/** RANGE [°]
 * Angle between boat middle axis and end of sector for measurement */ 
#define RANGE 90 


/** CPU-Frequency [Hz] */
#define F_CPU 8000000L 


/** LIDAR MAX DISTANCE RANGE [cm] 
 * Maximum Distance the LIDAR can measure. Above this distance the LIDAR returns zero */ 
#define LIDAR_MAX_DISTANCE 700 //25m


/** MAX OBSTACLE NUMBER 
 * Maximum number of obstacles that can be stored */ 
#define MAX_OBSTACLE_NUMBER 20


/** DEBUG FLAGS */
#define DEBUG_MATLAB 1  //Debugging in Matlab. A measurement Step is only done, when the distance data was transferred (1 == Debugging in Matlab is active) 
#define DEBUG_FILTER 0  //Turn off preprocessing (filtering) of the obstacles before sending them to the Pixhawk (1 == Filter turned off) 



/** COMMONLY USED VARIABLES */
extern CircularBuffer obst_buffer; 



#endif /* CONFIG_H_ */