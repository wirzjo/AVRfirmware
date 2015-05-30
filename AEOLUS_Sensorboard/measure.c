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
#include "buffer.h"
#include "pixhawk.h"


//static uint8_t obst_prob[(uint8_t)(RANGE*2/INTERVAL)]; 
static uint8_t dist_mat[(uint16_t)(360/INTERVAL)];
//static uint16_t head_mat[(uint16_t)(360/INTERVAL)]; 
static uint16_t last_center; 

//Small Version of the Distance Matrix, only the Measurements currently done are stored 
static uint16_t dist_mat_small[2*RANGE/INTERVAL]; 
static uint16_t head_valid; 


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
	int16_t threshold; //Threshold above which the correlated value is considered as an obstacle
} config = {
	.threshold = 10
};

CircularBuffer obst_buffer;	//Circular Buffer holding the detected obstacles 



/* @brief Filter the data and try to find outliers */ 
void filter();

/* @brief Store a value in the distance Matrix */ 
void push2matrix(uint16_t dist, uint16_t angle); 

/* @brief Store a value in the small distance Matrix */ 
void push2matrix_small(uint16_t dist); 

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
	
	//Initialize the Buffer
	//obst_buffer = buffer_init(MAX_OBSTACLE_NUMBER); 
	
	//Init the Distance Matrix with Zero 
	for(uint16_t i = 0; i<360/INTERVAL; i++) {
		dist_mat[i] = 0;
	}
	
	//Set the initial threshold
	config.threshold = 40; 
	
	
	#if DEBUG_CHEAPSERVO
		servo_set(90); 
		state.angle = 90; 
	#endif
	
	
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

	#if DEBUG_CHEAPSERVO == 1
	
		if(state.angle >= 2*RANGE) {
			
			_delay_ms(100);
			
			state.direction = -1;
			state.angle = 90;  
			servo_set(state.angle); 
			_delay_ms(180);   
			
			//We set the Heading of the Boat 
			head_valid = pixhawk_get_heading(); 
		}
		
		if(state.angle <= 0) {
			
			_delay_ms(100);
			
			state.direction = 1; 
			state.angle = 90;
			servo_set(state.angle); 
			_delay_ms(180); 
  
		}
		
	
	#else 
	
		//Check if we already finished one round 
		if(state.angle >= 2*RANGE) {
			//We are at the end => on backbord-side 
		 
			//state.direction = -1;
			state.direction = 1; 
			state.angle = 0;
			servo_set(0);  
			port_led_blink(1);  
		}
	
		if(state.angle <= 0) {
			//We are at the end => on starboard-side
		
			//state.angle = 0; 
			state.direction = 1;	
		}
	
	#endif
	
	//MOVE THE SERVO TO THE NEW ANGLE
	servo_set(state.angle); 

	//DO THE MEASUREMENT  
	uint16_t dist = lidar_measure();
	
	//TELL THE VALUE TO THE FILTER-UNIT
	push2matrix(dist, state.angle); 
	push2matrix_small(dist); 
	
	//Increase the Angle
	state.angle += (state.direction * INTERVAL);
	
}


/**
 * Push the value into the distance measurement matrix 
 * 
 * 
 */
void push2matrix(uint16_t dist, uint16_t angle_tn) {
	
	//CALCULATE ANGLE WRT TRUE NORTH 
	int16_t curr_course = pixhawk_get_heading(); 
	curr_course = 20; 
	//int16_t angle_tn = state.angle;
	int16_t alpha = 0; 	

	if(state.angle < RANGE) {
		//This is plus => to the starboard side of the boat
	
		alpha = RANGE - state.angle;
	
		angle_tn = mod(curr_course + alpha);
	}
	
	if(state.angle > RANGE) {
		//This is minus => to the backboard side of the boat
		
		alpha = state.angle - RANGE;
			
		angle_tn = mod(curr_course - alpha);
	}

	if(state.angle == RANGE) {
		//The obstacle lays direct in front of us
	
		angle_tn = curr_course;
	}
	
	
	
	//PUSH THE VALUE INTO THE DISTANCE MATRIX 
	uint16_t index = (float)angle_tn/(float)INTERVAL;		//Index in Distance Matrix 
	
	dist_mat[index] = dist;					//Store value in Matrix		
}


/**
 * Store the measurement in a Matrix (small Matrix) 
 * 
 */
void push2matrix_small(uint16_t dist) {
	
	uint16_t ind = state.angle/INTERVAL; 
	
	dist_mat_small[ind] = dist; 
	
}	


/**
 * Get the values stored in the small Matrix 
 *
 */
uint16_t measure_get_distance_small(uint16_t ind) {
	
	return dist_mat_small[ind]; 
	
}

/**
 * Get the heading for which the small distance matrix is valid 
 *
 */
uint16_t measure_get_heading_valid(void) {
	
	return head_valid; 
	
}


void filter(void) {
	
	bool start = false;
	uint16_t start_ind = 0;  
	
	for (uint16_t ind = 1; ind < (360/INTERVAL)-1; ind++) {
		
		//Differentiate 
		int16_t diff = (int16_t)(dist_mat[ind-1] - dist_mat[ind]); 
		
		
		//Find Start of an Obstacle 
		if(diff >= config.threshold) {
			start = true; 
			start_ind = ind; 
		}
		
		//Find End of an Obstacle 
		if(((-1)*diff) >= config.threshold) {
			
			if(start == true) {
				//We found a valid Obstacle => store it in the Buffer 
				
				uint16_t obst_index = (uint16_t)((ind-start_ind)/2) + start_ind;
				
				start = false;
				
				buffer_add(&obst_buffer,obst_index*INTERVAL,dist_mat[obst_index]);
			}
			
		}
		
		
	} 
	
}




/**
 * Filter the data using a template and try to identify obstacles 
 * NOTE: This function is called as soon as a full cycle with the Servo is performed.
 *
 */
void filter_correlate(void) {
	
	/*
	//port_led_blink(2); 
	
	//Template for the matching process 
	int8_t template[5] = {0,-1,0,1,0};
	#define template_length 5 //Length of the template  
	#define template_midInd 2 //Middle Index of the template  		
		
	
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
		
			//Add data to the buffer 	
			buffer_add(&obst_buffer,ind*INTERVAL,dist_mat[ind]);
			
			//port_led_blink(1); 
		}		
	
	}
	
	
	//The filtering is finished => we can delete the data 
	for(uint16_t ind = state.min_tn_angle_ind; ind <= state.max_tn_angle_ind; ind++) {
		
		dist_mat[ind] = 0; 
	}
	
	state.max_tn_angle_ind = 0x0000; 
	state.min_tn_angle_ind = 0xFFFF;  */ 
	
	
	//THIS IS THE NEW VERSION WITH START AND END FOR OBSTACLES 
	#define template_length 3								//Length of the template
	int8_t template_start[template_length] = {0,-1,0};		//Template for start-sequence
	int8_t template_end[template_length] = {0,1,0};			//Template for end-sequence
	
	#define template_midInd 1 //Middle Index of the template
		
		
	//TODO: Here we need the to calcualte the first derivative of the distances => f(t-1)-f(t) 	
		
	
	bool start = false;		//Flag signaling that a start-sequence was detected 
	uint16_t start_ind = 0; //Index, where a start-sequence was detected 
		
	//Correlate the measurement data with the template and rate obstacles
	//TODO change this in the way such that only measured headings are taken into account => state.min_index, state.max_index
	//Check what happens, at discontinuity 0->360°
	for(uint16_t ind=template_midInd; ind <= (360/INTERVAL)-(template_midInd+1); ind++) {
		
		int16_t sum1 = 0;
		int16_t sum2 = 0; 
		
		//Calculate the first derivative for there three values 
		int16_t derivative[template_length] = {0,0,0}; 
			
		derivative[0] = dist_mat[ind-1]-dist_mat[ind];
		derivative[1] = dist_mat[ind]-dist_mat[ind+1]; 
		derivative[2] = dist_mat[ind+1]-dist_mat[ind+2];  
		

		//Correlate the Signal with the templates 
		for (int8_t t_ind=(-template_midInd); t_ind < template_midInd; t_ind++) {
			sum1 = sum1 + template_start[t_ind+template_midInd]*(int16_t)derivative[t_ind];
			sum2 = sum2 + template_end[t_ind+template_midInd]*(int16_t)derivative[t_ind]; 
		}	
			
		//serial_send_byte(((int8_t)(sum1)));
			
		//Find local maximum
		if(sum1 > config.threshold && sum2 < config.threshold) {
			//We found a possible start-sequence of an obstacle
			
			start = true; 
			start_ind = ind;
			
			//char str[] = {"START "};
			//serial_send_string(str); 
		}
		
		if(start == true && sum2 > config.threshold && sum1 < config.threshold) {
			//We found a possible end-sequence that follows after a start-sequence of an obstacle 
			
			uint16_t obst_index = (uint16_t)((ind-start_ind)/2) + start_ind; 
			
			start = false; 
			
			//char str[] = {"END "};
			//serial_send_string(str);
			
			//Add the obstacle to the buffer 
			//port_led_blink(1);	//DEBUG: Blink once for every detected obstacle 
			buffer_add(&obst_buffer,obst_index*INTERVAL,dist_mat[ind]);  
			
		}	
			
	} 
	
	
	
	
	
}


/** 
 * Get the obstacles from the Buffer
 * 
 * @param angle: Pointer to the angle, where the obstacle is detected (wrt. true North) [°]
 * @param dist:  Pointer to the distance to the obstacle [cm]
 * @return true, if there are any more obstacles left, false otherwise
 */
bool measure_get_obstacles(uint16_t *angle, uint16_t *dist) {
	
	//Get the Obstacle-Object from the Buffer
	buffer_get_values(&obst_buffer, angle, dist); 
	
	//Return true as soon as the buffer is empty 
	return !buffer_is_empty(&obst_buffer); 
}





/**
 * Take the modulo for compass courses (account for discontinuity at 0°->360°)
 * 
 * >360: angle-360
 * <360: 360-angle
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


/**
 * Get the distance at a given angle 
 *
 * @param ind: index in the matrix 
 */
uint16_t measure_get_distance(uint16_t ind) {
	
	return dist_mat[ind]; 
	
}	



/**
 * Set the threshold for the Obstacle Correlation
 *
 * 
 */
bool measure_set_threshold(uint16_t threshold) {
	
	config.threshold = (int16_t)threshold; 
	
	return true; 
}	
  