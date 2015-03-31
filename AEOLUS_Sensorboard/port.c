/*
 * port.c
 *
 * This file inits the ports and provides functionality for reading and writing output ports 
 *
 * Created: 31.03.2015 14:54:22
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch> 
 */ 

#include <avr/io.h>
#include <stdbool.h>
#include "port.h"



/** 
 * Init the Ports 
 *
 */
void port_init(void) {
	
	//Set data direction (Output/Input) 
	DDRB = 0xff;	//All ports are outputs
	DDRC = 0xff;    //All ports are outputs
	DDRD = 0xff;    //All ports are outputs 
	
	//Set all Ports to logic zero (<=> OFF) 
	PORTB = 0x00; 
	PORTC = 0x00; 
	PORTD = 0x00; 
	
	
} 



/** 
 * Control the LED connected to PORT PB0
 *
 * @param state, true, iff the LED must be turned on, false else  
 */
void port_led(bool state) {
	if(state) {
		PORTB |= (1<<PB0); 
	} else {
		PORTB &= ~(1<<PB0); 
	}
}