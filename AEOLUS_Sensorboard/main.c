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

int main(void)
{
	
	/************************************************************************/
	/* BOOT                                                                 */
	/************************************************************************/
	
	//Init the input/output ports 
	port_init(); 
	
	
	
	/************************************************************************/
	/* MAIN WHILE LOOP                                                      */
	/************************************************************************/
    while(1)
    {
		
		//Hello World Application 
		port_led(true);
		_delay_ms(1000);
		_delay_ms(1000);
		_delay_ms(1000); 
		port_led(false); 
		_delay_ms(1000); 
		_delay_ms(1000);
		_delay_ms(1000);
		
		
		
    }
}