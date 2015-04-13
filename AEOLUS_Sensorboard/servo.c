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

#include "config.h"
#include "servo.h"

#define minPWM 550		//most left position, say 0°
#define maxPWM 2350     //most right position, say 180°


/** 
 * Initialize the use of a Servo 	
 */
bool servo_init(void) {
	
	DDRD |= 0xFF;		//Set DDRD as output 
	TCCR1A |= (1<<WGM11) | (1<<COM1A1);		
	TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS10;
	
	//Set maximum Timer-Count 
	// ICR1 = F_CPU/(Servo acceptable Value in Hz); 
	ICR1 = 15999; 

	//Init the Servo and make sure it starts in 90° Position. 
	servo_set(90); 
	
	return true; 
}



/**
 * Set the Servo to a given angle
 * 
 * @param deg: angle in degrees the servo should move to 
 */
void servo_set(float deg) {
    
	uint16_t pwm =  ICR1 - ((maxPWM-minPWM)/180.0f*deg + minPWM);
	
	//Saturate PWM output
	if(pwm<minPWM) {
		pwm = minPWM; 
	}
	
	if(pwm>maxPWM) {
		pwm = maxPWM;
	}
	
	OCR1A = pwm; 
}