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

#include "servo.h"
#include <avr/io.h>


/** 
 * Initialize the use of a Servo 	
 */
void servo_init(void) {
	
	/*TCCR1B = 0x00;
	TCCR1B = (1<<CS10) | (1<<WGM13); //Stellt den Prescaler ein
	ICR1  = 20000;
	TCCR1A = (1<<COM1A1);
	TCCR1A = (1<<COM1A0);*/
	
	
	DDRD |= 0xFF;
	//TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0;
	TCCR1A |= (1<<WGM11) | (1<<COM1A1); 
	TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS10;
	
	//Set maximum Timer-Count 
	// ICR1 = F_CPU/(Servo acceptable Value in Hz); 
	
	ICR1 = 15999; 
	//ICR1 = 19999;

	OCR1A = ICR1 - 2000; //18000
	
	
	
}