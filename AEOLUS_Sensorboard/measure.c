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


//static uint8_t obst_prob[(uint8_t)(RANGE*2/INTERVAL)]; 
static uint8_t dist_mat[(uint16_t)(360/INTERVAL)];
static uint16_t head_mat[(uint16_t)(360/INTERVAL)]; 
static uint16_t last_center; 

static struct {
	uint16_t angle;		//Current angle to be checked => starboard border is 0°
	int8_t direction;	//Increasing or Decreasing of the angle (starboard --> backboard = 1; backboard --> starboard = -1)
	uint16_t last_center; //Center angle for which the data in the array is valid 
	uint16_t curr_center; //Current center known from the Pixhawk 
	
	uint16_t max_tn_angle_ind;	//Maximum index that ocuured during the measurement process 
	uint16_t min_tn_angle_ind;  //Minimum index that occured during the measurement process 
} state = {
	.angle = 0, 
	.direction = 1,
	
	.max_tn_angle_ind = 0x0000,
	.min_tn_angle_ind = 0xFFFF 
};

static struct {
	uint8_t threshold; //Threshold above which the correlated value is considered as an obstacle
} config = {
	.threshold = 20
};


/* @brief Filter the data and try to find outliers */ 
void filter();

/* @brief Store a value in the distance Matrix */ 
void push2matrix(uint16_t dist); 

/* @brief Take the modulo for 360° */
uint16_t mod(int16_t); 



/**
 * Init the measurement function. 
 * => Move the Servo to Starboard
 *
 */
bool measure_init(void) {
	
	//Move the Servo to start-position 
	servo_set(0); 
	
	//Init the Angle (we start on Starboard) 
	state.angle = 0; 
	
	//Set the direction (Starboard to Backboard) 
	state.direction = 1; 
	
	return true; 
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
	
	/*
	uint16_t dist = lidar_measure();
	_delay_ms(500); 
	*/

	
	//Check if we already finished one round 
	if(state.angle>=2*RANGE) {
		//We are at the end => on backbord-side 
		
		state.angle = 2*RANGE; 
		state.direction = -1; 
		
		//filter();		//Filter the data and find the obstacles 
	}
	
	if(state.angle<=0) {
		//We are at the end => on starboard-side
		
		state.angle = 0; 
		state.direction = 1; 
		
		//filter();		//Filter the data and find the obstacles 
	}
	
	
	//MOVE THE SERVO TO THE NEW ANGLE 
	servo_set(state.angle); 
	//_delay_ms(100); 
	
	//DO THE MEASUREMENT  
	uint16_t dist = lidar_measure();
	
	//TELL THE VALUE TO THE FILTER-UNIT
	push2matrix(dist); 
	
	//Increase the Angle 
	state.angle += (state.direction * INTERVAL); 	
	
}



/**
 * Push the value into the distance measurement matrix 
 * 
 * 
 */
void push2matrix(uint16_t dist) {
	
	//CALCULATE ANGLE WRT TRUE NORTH 
	uint16_t curr_course = pixhawk_get_heading(); 
	uint16_t angle_tn = state.angle;
	int16_t alpha = 0; 
	if(state.angle > RANGE) {
		//This is minus => to the backboard side of the boat
	
		alpha = state.angle - RANGE;
	
		angle_tn = mod(curr_course - alpha);
	}	

	if(state.angle < RANGE) {
		//This is plus => to the starboard side of the boat
	
		alpha = RANGE-state.angle;
	
		angle_tn = mod(curr_course + alpha);
	}

	if(state.angle == RANGE) {
		//The obstacle lays direct in front of us
	
		angle_tn = curr_course;
	}
	
	
	//PUSH THE VALUE INTO THE DISTANCE MATRIX 
	uint16_t index = angle_tn/INTERVAL;		//Index in Distance Matrix 
	
	dist_mat[index] = dist;					//Store value in Matrix		
	
	
	//UPDATE MIN AND MAX INDEX VALUES 
	//This way, the search for obstacles in the correlation matrix can be reduced to the 
	//effectively written region. 
	if(index > state.max_tn_angle_ind) {
		state.max_tn_angle_ind = index; 
	}		
	
	if(index < state.min_tn_angle_ind) {
		state.min_tn_angle_ind = index; 
	}
}	



/**
 * Filter the data using a template and try to identify obstacles 
 * NOTE: This function is called as soon as a full cycle with the Servo is performed.
 *
 */
void filter(void) {
	
	//port_led_blink(2); 
	
	//Template for the matching process 
	int8_t template[5] = {0,-1,0,1,0};
	#define template_length 5 //Length of the template  
	#define template_midInd 2 //Middle Index of the template  
		
		
	//DEBUG: Output the distance matrix 
	/*for(uint16_t ind=0; ind<(360/INTERVAL); ind++) {
		serial_send_byte((uint8_t)dist_mat[ind]); 
	}*/			
		
	
	//Correlate the measurement data with the template and rate obstacles 
	//TODO change this in the way such that only measured headings are taken into account => state.min_index, state.max_index	
	//Check what happens, at discontinuity 0->360°
	for(uint16_t ind=template_midInd; ind <= (360/INTERVAL)-(template_midInd+1); ind++) {
	
		int16_t sum1 = 0;

		for (int8_t t_ind=(-template_midInd); t_ind < template_midInd; t_ind++) {
			sum1 = sum1 + template[t_ind+template_midInd]*dist_mat[t_ind+ind];
		}
	
		//serial_send_byte(((int8_t)(sum1>>8))); 
		//serial_send_byte(((int8_t)(sum1))); 
	
		//Find local maximum
		if(sum1 > config.threshold) {
			//We found an obstacle => store it in the dynamic buffer 
			
			port_led_blink(1); 
		}		
	
	}
	
	
	//The filtering is finished => we can delete the data 
	for(uint16_t ind = state.min_tn_angle_ind; ind <= state.max_tn_angle_ind; ind++) {
		
		dist_mat[ind] = 0; 
	}
	
	state.max_tn_angle_ind = 0x0000; 
	state.min_tn_angle_ind = 0xFFFF; 
	
	
}




/**
 * Take the modulo for compass courses 
 *
 */
uint16_t mod(int16_t angle) {


		while(angle>360) {
			angle = angle -  360;
		}

		while(angle<(-360)) {
			angle = angle + 360;
		}

		while(angle<0) {
			angle = 360 + angle;
		}

		return angle;
}	
  