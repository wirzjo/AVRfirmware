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

/** INTERVAL [�]
 * Angle between two distance measurements in degrees (smallest interval possible is 1, integer types only) */ 
#define INTERVAL 2 

/** RANGE [�]
 * Angle between boat middle axis and end of sector for measurement */ 
#define RANGE 90 


/** CPU-Frequency [Hz] */
#define F_CPU 8000000L 



/** DEBUG FLAGS */
#define DEBUG_MATLAB 1//Debugging in Matlab. A measurement Step is only done, when the distance data was transferred. 



#endif /* CONFIG_H_ */