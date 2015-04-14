/*
 * measure.c
 *
 * This file handles the main measurement process by controlling the LIDAR and the servo 
 *
 * Created: 10.04.2015 16:16:21
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 

#include "config.h"
#include <avr/delay.h>
#include "lidar.h"
#include "servo.h"
#include "serial.h"

#define SERVOFACTOR 1 //Factor the Angle has to be decreased for the Servo => the servo has a maximum range of -90°,90° <=> 0°,180°
					  //If 2*RANGE is bigger than this, we must reduce the servo angle and increase it mechanically! 

static uint8_t obst_prob[RANGE*2/INTERVAL]; 
static uint16_t last_center; 

struct {
	uint16_t angle;		//Current angle to be checked => starboard border is 0°
	uint8_t direction;	//Increasing or Decreasing of the angle (starboard --> backboard = 1; backboard --> starboard = -1)
	uint16_t last_center; //Center angle for which the data in the array is valid 
	uint16_t curr_center; //Current center known from the Pixhawk 
} state = {
	.angle = 0, 
	.direction = 1
};


/* @brief Filter the data and try to find outliers */ 
void filter(uint16_t *dist); 



/**
 * Init the measurement function. 
 * => Move the Servo to Starboard
 *
 */
void measure_init(void) {
	
	//Move the Servo to start-position 
	servo_set(0); 
	
	//Init the Angle (we start on Starboard) 
	state.angle = 0; 
	
	//Set the direction (Starboard to Backboard) 
	state.direction = 1; 
	
}


/**
 * Control the measurement process
 * Move the servo to start position. Then increment until the end is reached. 
 * In each step do a distance measurement. 
 *
 * => This function should be called in every iteration step in the main-function or by a timer interrupt, dependent on the 
 *    scheduling strategy.  
 */
void measure_handler(void) {
	
	
	uint16_t dist = lidar_measure();
	_delay_ms(500); 
	
	/*
	//Check if we already finished one round 
	if(state.angle>=2*RANGE) {
		//We are at the end => on backbord-side 
		
		state.angle = 2*RANGE; 
		state.direction = -1; 
	}
	
	if(state.angle<=0) {
		//We are at the end => on starboard-side
		
		state.angle = 0; 
		state.direction = 1; 
	}
	
	
	//MOVE THE SERVO TO THE NEW ANGLE 
	servo_set(state.angle * SERVOFACTOR); 
	
	//DO THE MEASUREMENT  
	uint16_t dist = lidar_measure();
	
	//TELL THE VALUE TO THE FILTER-UNIT
	filter(&dist); 
	
	//Increase the Angle 
	state.angle += state.direction * INTERVAL; 	
	*/
}



/**
 * Filter the measured data and try to find outliers, obstacles respectively
 * 
 * 
 */
void filter(uint16_t *dist) {
	
}