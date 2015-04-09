/*
 * main.c
 *
 * This software handles a LIDAR Sensor and uses SPI to communicate with the PIXHAWK Autopilote in ETH's 
 * autonomous sailing project. 
 *
 * Created: 31.03.2015 14:43:33
 *  Author: Jonas Wirz <wirzjo@student.ethz.ch>
 */ 


/* TODO: 
 * 
 *	- Add Watchdog
 *  - Add Sleep-Mode
 *  - Add proper scheduling 
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "config.h"
#include "port.h"
#include "servo.h"
#include "lidar.h"
#include "serial.h"

#include <util/delay.h>

int main(void)
{
	
	/************************************************************************/
	/* BOOT                                                                 */
	/************************************************************************/
	
	bool boot_state = true;		//true, if everything is fine during the boot process 
	
	//Disable any Interrupt 
	cli(); 
	
	//Init the input/output ports 
	boot_state = boot_state && port_init(); 
	
	//Init the use of a Servo
	boot_state = boot_state && servo_init(); 
	
	//Init the use of the LIDAR 
	boot_state = boot_state && (lidar_init()||true);     //DEBUG: Remove true, this is only, because no lidar is present by now
	
	
	//Init the use of the Pixhawk 
	pixhawk_init(); 
	
	
	
	/************************************************************************/
	/* MAIN WHILE LOOP                                                      */
	/************************************************************************/
    while(boot_state)
    {
		/* Note this main while-loop is only started if all Sensors are initialized successfully 
		 * Otherwise for safety reasons the main loop is not started at all */ 
		
		
		//Toggle LED => show that program is running  
		port_led(true);
		_delay_ms(1000); 
		
		
		//Send a byte to the pixhawk using the serial interface in every loop 
		serial_send_byte(0x41); //Send a capital A 
		serial_send_byte(0x42); //Send a capital B  
		
		
		port_led(false); 
		_delay_ms(1000); 
		
		
		
		
		/*
		//Move Servo from 0 to 180° in Steps of 5°
		uint8_t ang = 0; 
		for(ang = 0; ang <= 180; ang = ang+5) {
		servo_set(ang); 
		_delay_ms(50); 
		}		

		//Toggle LED 
		port_led(false);
		_delay_ms(1000); 
		servo_set(0); 
		_delay_ms(1000);	
		*/
		
    }
}