/*
 * main.c
 *
 * This software handles a LIDAR Sensor and uses SPI to communicate with the PIXHAWK Autopilote in ETH's 
 * autonomous sailing project. 
 *
 * Created: 31.03.2015 14:43:33
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

#include "port.h"
#include "servo.h"

int main(void)
{
	
	/************************************************************************/
	/* BOOT                                                                 */
	/************************************************************************/
	
	//Init the input/output ports 
	port_init(); 
	
	//Init the use of a Servo
	servo_init(); 
	
	
	
	/************************************************************************/
	/* MAIN WHILE LOOP                                                      */
	/************************************************************************/
    while(1)
    {
		//Toggle LED  
		port_led(true);
		
		//Move Servo from 0 to 180° in Steps of 5°
		uint8_t ang = 0; 
		for(ang = 0; ang <= 180; ang = ang+2) {
		servo_set(ang); 
		_delay_ms(50); 
		}		

		//Toggle LED 
		port_led(false);
		_delay_ms(1000); 
		servo_set(0); 
		_delay_ms(1000);	
		
    }
}