/*
 * servo.c
 *
 * This file contains functions used for driving a standard Servo Motor
 *
 * Note: For more info look at https://www.newbiehack.com/MicrocontrollerControlAHobbyServo.aspx (31.03.15) 
 *
 * Created: 31.03.2015 14:52:14
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch> 
 */ 

#include <avr/io.h>
#include <util/delay.h>

#include "config.h"
#include "servo.h"

#define ServoRange 180   //Number of Degrees from fully left to fully right [°]
#define ServoSpeed 2     //Speed of the Servo [ms/°]


#define minPWM 575	//575 (650 was ok) 
#define maxPWM 2350 	//without LIDAR: 2348, with LIDAR: 2375

static struct {
	uint16_t angle; 
} state = {
	.angle = 0
};


/* @brief Wait for a given amount of Milliseconds */ 
void wait(uint16_t time);



/** 
 * Initialize the use of a Servo 	
 */
bool servo_init(void) {
	
	DDRD |= 0xFF;		//Set DDRD as output 
	TCCR1A |= (1<<WGM11) | (1<<COM1A1);	//Set mode 
	
	//Set Wave-Form of PWM 	
	TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS10;
	
	TCCR1B |= (1<<CS11);	//Use Prescalor of 8 => FCPU/8
	TCCR1B &= ~(1<<CS10); 
	TCCR1B &= ~(1<<CS12); 
	
	//Set maximum Timer-Count 
	// ICR1 = F_CPU/(Servo acceptable Value in Hz); 
	ICR1 = 19999; 

	//Init the Servo and make sure it starts in middle Position 
	//OCR1A = ICR1 - (maxPWM-minPWM)/2 + minPWM; 
	//OCR1A = ICR1 - minPWM; 
	//servo_set(180);
	
	//OCR1A = ICR1 - middlePWM;  
	OCR1A = ICR1 - minPWM; 
	
	/*_delay_ms(1000); 
	_delay_ms(1000); 
	_delay_ms(1000); 
	_delay_ms(1000); 
	OCR1A = ICR1 - minPWM; 
	
	_delay_ms(1000);
	_delay_ms(1000);
	_delay_ms(1000);
	_delay_ms(1000);
	OCR1A = ICR1 - maxPWM; */
	
	
	return true; 
}



/**
 * Set the Servo to a given angle
 * 
 * @param deg: angle in degrees the servo should move to 
 */
void servo_set(float deg) {
    
	//Calculate the PWM Signal 
	uint16_t pwm = (((float)((float)maxPWM-(float)minPWM))/(float)ServoRange*(float)deg + (float)minPWM);
	
	//Time for moving to this position 
	int16_t ang_diff = state.angle-deg; 
	if(ang_diff < 0) {
		ang_diff = -ang_diff; 
	}
	uint16_t time = (float)ang_diff/(float)ServoSpeed;	
	
	//Store the angle locally
	state.angle = deg;
	
	//Saturate PWM output
	if(pwm<minPWM) {
		pwm = minPWM; 
	}
	
	if(pwm>maxPWM) {
		pwm = maxPWM;
	}
	
	OCR1A = ICR1 - pwm; 
	
	//Wait for the servo to reach the new position 
	wait(time); 
	
	//OCR1A = ICR1 - deg; 
}





void wait(uint16_t time) {
	for(uint16_t i=0; i<time; i++) {
		_delay_ms(1); 
	}
}