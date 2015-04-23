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

#define minPWM 500		//most left position, say 0° (550) [PWM]
#define maxPWM 2400     //most right position, say 180° (2350) [PWM]
#define ServoRange 210	//Number of Degrees from fully left to fully right [°]


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
	OCR1A = ICR1 - (maxPWM-minPWM)/2 + minPWM; 
	//servo_set(90); 
	
	return true; 
}



/**
 * Set the Servo to a given angle
 * 
 * @param deg: angle in degrees the servo should move to 
 */
void servo_set(float deg) {
    
	uint16_t pwm = ((maxPWM-minPWM)/ServoRange*deg + minPWM);
	
	//Saturate PWM output
	if(pwm<minPWM) {
		pwm = minPWM; 
	}
	
	if(pwm>maxPWM) {
		pwm = maxPWM;
	}
	
	OCR1A = ICR1 - pwm; 
}